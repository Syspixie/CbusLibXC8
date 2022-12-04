/**
 * @file CbusLibXC8 cbuscan.c
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
 * Code to pass CBUS messages over CAN bus.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 * 
 * CAN peripheral specific code handed off to ecan or can1.
 */


#include "cbuscan.h"
#include "util.h"
#include "millis.h"
#include "module.h"
#include "eeprom.h"
#include "opcode.h"

#if defined(ECAN_BUFFERS_BASE_ADDRESS)
#include "ecan.h"
#define initCan initEcan
#define canSetID ecanSetID
#define canSendRtrRequest ecanSendRtrRequest
#define canSendRtrResponse ecanSendRtrResponse
#define canTransmit ecanTransmit
#define canReceive ecanReceive
#define canTimerIsr ecanTimerIsr
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
#include "can1.h"
#define initCan initCan1
#define canSetID can1SetID
#define canSendRtrRequest can1SendRtrRequest
#define canSendRtrResponse can1SendRtrResponse
#define canTransmit can1Transmit
#define canReceive can1Receive
#define canTimerIsr can1TimerIsr
#endif


// Used to convert a 7-bit CBUS CAN ID to a standard ID
#define CAN_BUS_ID_UPPER_BITS 0b10110000000
// Used to convert a standard ID to a 7-bit CBUS CAN ID
#define CAN_BUS_ID_UPPER_BITS_MASK 0b00001111111


// CAN ID self-enumeration process state
typedef enum {
    selfEnumIsInactive,     // No self-enumeration
    selfEnumIsPending,      // Self-enumeration to start after 'holdoff' time
    selfEnumIsRequested,    // Self-enumeration to start ASAP
    selfEnumIsInProgress    // Self-enumeration in progress
} enum_t;


// extern
uint8_t canBusID;       // CBUS 7-bit CAN ID
uint8_t cbusMsg[8];     // CBUS message (8 bytes)

enum_t selfEnumStatus = selfEnumIsInactive; // Self-enumeration state
bool selfEnumSendResult = false;            // Send result flag
uint16_t selfEnumStartedMillis;             // Self-enumeration start time
uint8_t selfEnumBitmap[CBUSCAN_ENUM_BITMAP_SIZE];   // Bitmap of 'seen' CAN IDs


/**
 * Initialises the CAN bus peripheral.
 */
void initCbusCan() {

    initCan();

    // Use the initial CanBusID
    canSetID(CAN_BUS_ID_UPPER_BITS | canBusID);
}

/**
 * Performs CAN ID self-enumeration.
 * 
 * Basic principle: send a request message; all nodes on the bus respond;
 * CAN ID of respondents logged in a bitmap; unused CAN ID chosen.
 * 
 * @return 1: send response; 0: no response; <0: send error response.
 * @post cbusMsg[] NNACK response.
 */
static int8_t processEnumeration() {

    uint16_t t = getMillisShort();
    uint16_t tSince = t - selfEnumStartedMillis;

    // If immediate start requested, or delayed start has waited long enough...
    if (selfEnumStatus == selfEnumIsRequested
            || (selfEnumStatus == selfEnumIsPending && tSince >= CBUSCAN_ENUM_HOLD_OFF_MILLIS)) {

        // Start self-enumeration
        selfEnumStatus = selfEnumIsInProgress;

        // Clear all bitmap bits except bit 0 (CAN ID 0 not valid)
        selfEnumBitmap[0] = 1;
        utilMemset(&selfEnumBitmap[1], 0, CBUSCAN_ENUM_BITMAP_SIZE - 1);

        // Send RTR request, which will trigger a response from all currently
        // active nodes.
        canSendRtrRequest();

        // Note the start time.
        selfEnumStartedMillis = t;

        // If the wait for response period has expired...
    } else if (selfEnumStatus == selfEnumIsInProgress && tSince >= CBUSCAN_ENUM_PERIOD_MILLIS) {

        // Self-enumeration complete
        selfEnumStatus = selfEnumIsInactive;

        // Find the first 0 bit in the bitmap; use this as our CAN ID
        bool ok = false;
        for (uint8_t i = 0; i < CBUSCAN_ENUM_BITMAP_SIZE; ++i) {
            if (selfEnumBitmap[i] != 0b11111111) {
                uint8_t newCanID = (uint8_t) (i << 3);
                for (uint8_t x = selfEnumBitmap[i]; (x & 0b00000001); x >>= 1) ++newCanID;
                ok = setCbusCanID(newCanID, true);
                break;
            }
        }

        // If result message required...
        if (selfEnumSendResult) {
            selfEnumSendResult = false;

            // If OK, send NNACK
            if (ok) {
                cbusMsg[0] = OPC_NNACK;
                return 1;

                // Otherwise no free CAN ID found
                // Presumably this node, and any other node with the same address,
                // will self-enumerate ad-nauseum?
            } else {
                return -CMDERR_INVALID_EVENT;
            }
        }
    }
    return 0;
}

/**
 * Adds an outgoing message to the transmit FIFO.
 * 
 * @pre cbusMsg[] opcode response.
 * @param tx 1: send response; <0 send error response.
 */
static void queueTransmit(int8_t tx) {

    // Change error code into a proper message
    if (tx < 0) {
        cbusMsg[0] = OPC_CMDERR;
        cbusMsg[3] = (uint8_t) - tx;
    }

    // Add our node number if required by the opcode
    opCodeAddNodeNumber();

    // Add to transmit FIFO
    canTransmit();
}

/**
 * Checks for an incoming message.  If one has been received, it is processed
 * and, if required, a response is sent as an outgoing message.
 * 
 * @return true if message received and processed.
 */
bool receiveCbusCan() {

    static bytes16_t stdID;
    static bool isRtr;
    static uint8_t dataLen;

    // Receive message
    if (!canReceive(&stdID, &isRtr, &dataLen)) return false;

    // Clear upper (priority) bits from CAN ID
    uint8_t id = stdID.value & CAN_BUS_ID_UPPER_BITS_MASK;

    // If received CAN ID is the same as ours...
    if (canBusID > 0 && id == canBusID && selfEnumStatus == selfEnumIsInactive) {

        // Start self-enumeration after a delay
        selfEnumStatus = selfEnumIsPending;
        selfEnumStartedMillis = getMillisShort();

        // Finished
        return true;
    }

    // If RTR...
    if (isRtr) {

        // Zero data RTR...
        if (dataLen == 0) {

            // Some other node is doing self-enumeration
            // Send response which includes our CAN ID
            canSendRtrResponse();

            // If we have self-enumeration pending, extend the wait to allow other
            // process to finish
            if (selfEnumStatus == selfEnumIsPending) selfEnumStartedMillis = getMillisShort();
        }

        // Finished
        return true;
    }

    // If zero data, must be response to our RTR...
    if (dataLen == 0) {

        // Set bitmap bit corresponding to received CAN ID
        selfEnumBitmap[id >> 3] |= 1 << (id & 0b00000111);

        // Finished
        return true;
    }

    // This is a CBUS message - process it
    int8_t tx = processCbusMessage();

    // Send response if there is one
    if (tx) queueTransmit(tx);

    return true;
}

/**
 * Checks for an unsolicited outgoing message.
 * 
 * @return true if message transmitted.
 */
bool transmitCbusCan() {

    int8_t tx = 0;

    // Check self-enumeration, which may output a message when complete
    if (selfEnumStatus != selfEnumIsInactive) tx = processEnumeration();

    // If no message yet, check the rest of the application
    if (!tx) tx = generateCbusMessage();

    // Send message if there is one
    if (tx) queueTransmit(tx);

    return (tx);
}

/**
 * Requests self-enumeration with immediate start.
 * 
 * @param sendResult Flag indicating whether to send response when complete.
 */
void enumerateCbusCanID(bool sendResult) {

    // Start self enumeration if not already started
    if (selfEnumStatus != selfEnumIsInProgress) selfEnumStatus = selfEnumIsRequested;

    // Flag send result if required
    if (sendResult) selfEnumSendResult = true;
}

/**
 * Change our CAN ID.
 * 
 * @param newCanID The new CAN ID
 * @param check Flag indicating whether new CAN ID should be validated.
 * @return true if CAN ID set; false if it failed validation.
 * @post new CAN ID written to EEPROM.
 */
bool setCbusCanID(uint8_t newCanID, bool check) {

    // If checking required...
    if (check) {

        // Make sure that he new CAN ID is within the valid dynamic range,
        // and also that our existing CAN ID is not in the static range
        if (newCanID < CBUSCAN_MIN_DYNAMIC_CANID
                || newCanID > CBUSCAN_MAX_DYNAMIC_CANID
                || canBusID > CBUSCAN_MAX_DYNAMIC_CANID) return false;
    }

    // Use the new CAN ID
    canBusID = newCanID;
    canSetID(CAN_BUS_ID_UPPER_BITS | canBusID);

    // Don't forget it
    writeEeprom8(EEPROM_CAN_ID, canBusID);

    return true;
}


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">


#if !defined(IVT_BASE_ADDRESS)
/**
 * Service routine for CAN interrupts.
 */
void cbusCanIsr() {

    // Call ECAN ISR
    ecanIsr();
}
#endif

/**
 * Performs regular CAN bus operations (called every millisecond).
 */
void cbusCanTimerIsr(void) {

    // Call ECAN or CAN1 ISR
    canTimerIsr();
}


// </editor-fold>


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">

/**
 * Processes opcode 0x75 "Set a CAN_ID in existing FLiM node".
 * 
 * @pre cbusMsg[] CANID message.
 * @return 0: no response; <0: send error response.
 */
int8_t opCodeCANID() {

    // Set CAN ID if it checks out OK
    if (!setCbusCanID(cbusMsg[3], true)) return -CMDERR_INVALID_EVENT;
    return 0;
}

/**
 * Processes opcode 0x5D "Force a self enumeration cycle for use with CAN".
 * 
 * @pre cbusMsg[] ENUM message.
 * @return 0: no response; <0: send error response.
 * 
 * @note When self enumeration is complete, NNACK response will be sent if
 * successful, otherwise error response will be sent.
 */
int8_t opCodeENUM() {

    // Don't self-enumerate if our CAN ID is in the static range
    if (canBusID > CBUSCAN_MAX_DYNAMIC_CANID) return -CMDERR_INVALID_EVENT;

    // Start self-enumeration, and request response message
    enumerateCbusCanID(true);
    return 0;
}


// </editor-fold>
