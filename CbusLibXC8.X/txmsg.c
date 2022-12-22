/**
 * @file CbusLibXC8 txmsg.c
 * @copyright (C) 2022 Konrad Orlowski     <syspixie@gmail.com>
 * 
 *  CbusLibXC8 is licensed under the:
 *      Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 *      International License.
 *  To view a copy of this license, visit:
 *      http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  Postal address: Creative Commons, PO Box 1866, Mountain View, CA 94042, USA
 * 
 * License summary:
 * 
 *  You are free to:
 *      Share, copy and redistribute the material in any medium or format;
 *      Adapt, remix, transform, and build upon the material.
 *  The licensor cannot revoke these freedoms as long as you follow the license
 *  terms.
 *
 *  Attribution: You must give appropriate credit, provide a link to the
 *  license, and indicate if changes were made. You may do so in any reasonable
 *  manner, but not in any way that suggests the licensor endorses you or your
 *  use.
 * 
 *  NonCommercial: You may not use the material for commercial purposes. **(see
 *  note below)
 *
 *  ShareAlike: If you remix, transform, or build upon the material, you must
 *  distribute your contributions under the same license as the original.
 *
 *  No additional restrictions: You may not apply legal terms or technological
 *  measures that legally restrict others from doing anything the license
 *  permits.
 * 
 * ** For commercial use, please contact the original copyright holder(s) to
 * agree licensing terms.
 * 
 *  This software is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.
 * 
 *******************************************************************************
 * 
 * CbusLibXC8 and CbusLibBootXC8 is code that can form the basis of the firmware
 * required to build a Model Electronic Railway Group (MERG) CBUS compatible
 * module.  It was developed using the Microchip XC8 compiler and MPLAB X IDE,
 * and targets PIC18F MCUs with built-in CAN bus peripherals.
 * 
 * The software was created to meet the CBUS specification as detailed in the
 * "Developer's Guide for CBUS" version 6c.
 * 
 * Credit to Mike Bolton and Gil Fuchs for the development of CBUS, and to
 * Pete Brownlow, Ian Hogg, and many other MERG members, for the development
 * and testing of CBUS modules, hardware and firmware, over the past 15 years.
 * 
 * MERG CBUS: https://www.merg.org.uk/resources/cbus
 */

/**
 * Code to enable a series of response messages to be sent over a period of
 * time.
 * 
 * @author Konrad Orlowski
 * @date December 2022
 * 
 * Provides the facility to schedule the calling of a function a number of
 * times.  Most useful for sending multiple responses to a request, e.g.
 * ENRSP opcode messages in response to a NERD opcode.
 * 
 * Each specified function is queued, along with the number of calls required.
 * As each function is dequeued it is called the requisite number of times, with
 * a defined interval TXMSG_DELAY_MILLIS (~10ms) between calls.
 * 
 * The function will normally populate cbusMsg[] with a message and return 1 to
 * send it; it may call cancelTXMsg() to cut short the call sequence.
 */


#include "txmsg.h"
#include "millis.h"
#include "module.h"


#define TXMSG_QUEUE_CUR_LENGTH ((uint8_t) queueHead - queueTail)


typedef struct {
    txMsgFunction_t function;
    uint8_t p1;
    uint8_t p2;
} tmdrspEntry_t;


// Function queue
tmdrspEntry_t queue[TXMSG_QUEUE_LENGTH];
uint8_t queueHead = 0;
uint8_t queueTail = 0;

uint16_t lastRunMillis;
bool lastRunValid = false;


/**
 * Initialises timed response.
 */
void initTXMsg() {

    // Initial delay whilst things settle down
    lastRunMillis = getMillisShort();
    lastRunValid = true;
}

/**
 * Queues a timed response function.
 * 
 * @param func Function to be queued.
 * @param numCalls Number of times that the function should be called.
 * @return true Function successfully queued.
 */
bool enqueueTXMsg(txMsgFunction_t func, uint8_t p1, uint8_t p2) {

    // Check space on the queue
    if (TXMSG_QUEUE_CUR_LENGTH == TXMSG_QUEUE_LENGTH) return false;

    // Enqueue the function
    uint8_t idx = queueHead % TXMSG_QUEUE_LENGTH;
    queue[idx].function = func;
    queue[idx].p1 = p1;
    queue[idx].p2 = p2;
    ++queueHead;
    return true;
}

/**
 * Checks the timer and calls the queued function when required.
 * 
 * @return 1: send response; 0: no response; <0: send error response.
 * @post cbusMsg[] Outgoing message.
 */
int8_t processTXMsg() {

    // Exit if not scheduled to run yet
    if (lastRunValid) {
        if (getMillisShort() - lastRunMillis < TXMSG_DELAY_MILLIS) return 0;
        lastRunValid = false;
    }

    // Done if no functions queued
    if (TXMSG_QUEUE_CUR_LENGTH == 0) return 0;

    // Dequeue next function
    uint8_t idx = queueTail % TXMSG_QUEUE_LENGTH;
    txMsgFunction_t func = queue[idx].function;
    uint8_t p1 = queue[idx].p1;
    uint8_t p2 = queue[idx].p2;
    ++queueTail;

    // Call function
    int8_t tx = func(p1, p2);

    // Delay next message if transmitting this one
    if (tx) {
        lastRunMillis = getMillisShort();
        lastRunValid = true;
    }

    return tx;
}
