/**
 * @file CbusLibXC8 can1.c
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
 * @date November 2022
 * 
 * @note Uses hardware CAN1 module.
 *
 * The CAN1 peripheral is configured with 10 TXQ FIFOs and 12 RX FIFOs.  Only
 * standard ID messages are received and transmitted; extended ID messages are
 * filtered out in hardware.  Bus speed is set at 125kbps.
 */


#include "can1.h"
#include "hardware.h"
#include "util.h"
#include "millis.h"
#include "stats.h"
#include "module.h"


#define CBUSCAN_TX_PRIORITY_HIKE_MILLIS 750     // 0.75 seconds
#define CBUSCAN_TX_TIMEOUT_MILLIS 1000          // 1 second

// extern
#ifdef INCLUDE_CBUS_CAN_STATS
volatile stats_t stats = {.bytes =
    {[0 ... sizeof (stats.bytes) - 1] = 0}};
#endif

// Transmit buffer start times for determining timeout
uint16_t txb0StartedMillis;
uint16_t txb1StartedMillis;
uint16_t txb2StartedMillis;


void initCan1() {
}

void can1SendRtrRequest() {

    txb1StartedMillis = getMillisShort();
}

void can1SendRtrResponse() {

    txb2StartedMillis = getMillisShort();
}

void can1Transmit() {
}

int8_t can1Receive(bool (* msgCheckFunc)(uint8_t id, uint8_t dlc, volatile uint8_t* data)) {

    return 0;
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Performs regular CAN bus operations (called every millisecond).
 */
void can1TimerIsr() {

}


// </editor-fold>
