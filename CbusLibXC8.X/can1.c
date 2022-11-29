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

#if defined(CAN1_BUFFERS_BASE_ADDRESS)


typedef struct {
    uint16_t stdID;
    union {
        struct {
            unsigned DLC : 4;
            unsigned IDE : 1;
            unsigned RTR : 1;
            unsigned BRS : 1;   // CAN FD only
            unsigned FDF : 1;   // CAN FD only
        } dlcBits;
        uint8_t dlc;
    };
} canFrame_t;

typedef enum {
    opModeNormalFD = 0,         //CAN FD only
    opModeDisable = 1,
    opModeInternalLoopback = 2,
    opModeListenOnly = 3,
    opModeConfiguration = 4,
    opModeExternalLoopback = 5,
    opModeNormal = 6,
    opModeRestrictedOperation = 7,
} opMode_t;


// extern
#ifdef INCLUDE_CBUS_CAN_STATS
volatile stats_t stats = {.bytes =
    {[0 ... sizeof (stats.bytes) - 1] = 0}};
#endif

// Current standard ID
bytes16_t curStdID;


static bool setOperationMode(const opMode_t requestMode) {

    opMode_t opMode = C1CONUbits.OPMOD;
    if (!(opMode == opModeConfiguration
            || requestMode == opModeDisable
            || requestMode == opModeConfiguration)) return false;

    C1CONTbits.REQOP = requestMode;

    while (C1CONUbits.OPMOD != requestMode) {
        //This condition is avoiding the system error case endless loop
        if (C1INTHbits.SERRIF) return false;
    }

    return true;
}

static bool receiveMessage(canFrame_t* rxCanMsg, uint8_t* data) {

    // Done if nothing in FIFO
    if (!(C1FIFOSTA1L & _C1FIFOSTA1L_TFNRFNIF_MASK)) return false;

    // Pointer to FIFO entry
    uint8_t* rxFifoObj = (uint8_t*) C1FIFOUA1;

    // Get ID
    bytes16_t v;
    v.valueL = rxFifoObj[0];
    v.valueH = rxFifoObj[1] & 0b00000111;
    rxCanMsg->stdID = v.value;

    // Get DLC
    rxCanMsg->dlc = rxFifoObj[4];
    uint8_t len = rxCanMsg->dlcBits.DLC;
    if (len > 8) {
        len = 8;
        rxCanMsg->dlcBits.DLC = len;
    }

    // Get Data
    utilMemcpy(data, &rxFifoObj[8], len);

    // Increment FIFO; clear any overflow flag
    C1FIFOCON1H |= _C1FIFOCON1H_UINC_MASK;
    C1FIFOSTA1L &= ~_C1FIFOSTA1L_RXOVIF_MASK;

    return true;
}

static bool transmitMessage(canFrame_t* txCanMsg, uint8_t* data) {

    // Done if FIFO full
    if (!(C1TXQSTAL & _C1TXQSTAL_TXQNIF_MASK)) return false;

    // Pointer to FIFO entry
    uint8_t* txFifoObj = (uint8_t*) C1TXQUA;

    // Put ID
    txFifoObj[0] = ((bytes16_t) txCanMsg->stdID).valueL;
    txFifoObj[1] = ((bytes16_t) txCanMsg->stdID).valueH;

    // Put DLC
    txFifoObj[4] = txCanMsg->dlc;

    // Put data
    utilMemcpy(&txFifoObj[8], data, txCanMsg->dlcBits.DLC);

    // Request transmission; increment FIFO
    C1TXQCONH |= (_C1TXQCONH_TXREQ_MASK | _C1TXQCONH_UINC_MASK);

    return true;
}

/**
 * Initialises the CAN1 peripheral.
 */
void initCan1() {

    /* Enable the CAN module */
    C1CONHbits.ON = 1;

    if (!setOperationMode(opModeConfiguration)) return;

    /* Initialize the C1FIFOBA with the start address of the CAN FIFO message object area. */
    C1FIFOBA = CAN1_BUFFERS_BASE_ADDRESS;

    C1CONL = 0x60;      // CLKSEL0 disabled; PXEDIS enabled; ISOCRCEN enabled; DNCNT 0;
    C1CONH = 0x97;      // ON enabled; FRZ disabled; SIDL disabled; BRSDIS enabled; WFT T11 Filter; WAKFIL enabled;
    C1CONU = 0x10;      // TXQEN enabled; STEF disabled; SERR2LOM disabled; ESIGM disabled; RTXAT disabled;

    C1NBTCFGL = 0x02;   // SJW 2;
    C1NBTCFGH = 0x02;   // TSEG2 2;
    C1NBTCFGU = 0x03;   // TSEG1 3;
    C1NBTCFGT = 0x3F;   // BRP 63;

    C1TXQCONL = 0x10;   // TXATIE enabled; TXQEIE disabled; TXQNIE disabled;
    C1TXQCONH = 0x04;   // FRESET enabled; UINC disabled;
    C1TXQCONU = 0x60;   // TXAT 3; TXPRI 1;
    C1TXQCONT = 0x09;   // PLSIZE 8; FSIZE 10;

    C1FIFOCON1L = 0x08; // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE enabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE disabled;
    C1FIFOCON1H = 0x04; // FRESET enabled; TXREQ disabled; UINC disabled;
    C1FIFOCON1U = 0x20; // TXAT Three retransmission attempts; TXPRI 1;
    C1FIFOCON1T = 0x0B; // PLSIZE 8; FSIZE 12;

    C1FLTOBJ0L = 0x00;
    C1FLTOBJ0H = 0x00;
    C1FLTOBJ0U = 0x00;
    C1FLTOBJ0T = 0x00;  // EXIDE clear: allow standard ID only
    C1MASK0L = 0x00;
    C1MASK0H = 0x00;
    C1MASK0U = 0x00;
    C1MASK0T = 0x40;    // MIDE set: filter on EXIDE
    C1FLTCON0L = 0x81;  // FLTEN0 enabled; F0BP FIFO 1; 

    IPR0bits.CANIP = 0;
    PIR0bits.CANIF = 0;

    C1INTL = 0x00;      // MODIF disabled; TBCIF disabled;
    C1INTH = 0x00;      // IVMIF disabled; WAKIF disabled; CERRIF disabled; SERRIF disabled;
    C1INTU = 0x08;      // TEFIE disabled; MODIE enabled; TBCIE disabled; RXIE disabled; TXIE disabled;
    C1INTT = 0xFC;      // IVMIE enabled; WAKIE enabled; CERRIE enabled; SERRIE enabled; RXOVIE enabled; TXATIE enabled;

    PIE0bits.CANIE = 1;

    setOperationMode(opModeNormal);
}

/**
 * Sets the current CAN BUS ID
 * 
 * @param id CBUS CAN ID + priority bits
 */
void can1SetID(uint16_t stdID) {

    curStdID.value = stdID;
}

/**
 * Sends an RTR request.
 */
void can1SendRtrRequest() {

    canFrame_t msg;
    msg.stdID = curStdID.value;
    msg.dlc = 0b00100000;
    transmitMessage(&msg, NULL);
}

/**
 * Sends an RTR response.
 */
void can1SendRtrResponse() {

    canFrame_t msg;
    msg.stdID = curStdID.value;
    msg.dlc = 0;
    transmitMessage(&msg, NULL);
}

/**
 * Sends a CBUS message.
 * 
 * @pre message in cbusMsg[]
 */
void can1Transmit() {

    canFrame_t msg;
    msg.stdID = curStdID.value;
    msg.dlc = (cbusMsg[0] >> 5) + 1;
    transmitMessage(&msg, cbusMsg);
}

/**
 * Receives a message.
 * 
 * @param msgCheckFunc function to pre-process message and check for CBUS message
 * @return -1: no message; 0: not a CBUS message; 1: is a CBUS message
 * @post cbusMsg[] CBUS message
 */
int8_t can1Receive(bool(* msgCheckFunc)(uint16_t stdID, uint8_t dataLen, volatile uint8_t* data)) {

    // Get message; done if none
    canFrame_t msg;
    uint8_t data[8];
    if (!receiveMessage(&msg, data)) return -1;

    // Check for things we can't handle (should never happen if hardware config OK)
    if (msg.dlc & 0b11010000) return -1;

    // Callback check for CBUS message
    bool haveMsg = msgCheckFunc((uint16_t) msg.stdID,
            msg.dlcBits.DLC, (msg.dlcBits.RTR) ? NULL : data);

    return haveMsg ? 1 : 0;
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


void __interrupt(irq(CAN), base(IVT_BASE_ADDRESS), low_priority) CAN1_ISR(void) {

    if (1 == C1INTHbits.IVMIF) {
        //CAN1_InvalidMessageHandler();
        C1INTHbits.IVMIF = 0;
    }

    if (1 == C1INTHbits.WAKIF) {
        //CAN1_BusWakeUpActivityHandler();
        C1INTHbits.WAKIF = 0;
    }

    if (1 == C1INTHbits.CERRIF) {
        //CAN1_BusErrorHandler();
        C1INTHbits.CERRIF = 0;
    }

    if (1 == C1INTLbits.MODIF) {
        //CAN1_ModeChangeHandler();
        C1INTLbits.MODIF = 0;
    }

    if (1 == C1INTHbits.SERRIF) {
        //CAN1_SystemErrorHandler();
        C1INTHbits.SERRIF = 0;
    }

    if (1 == C1INTHbits.TXATIF) {
        //CAN1_TxAttemptHandler();
        if (1 == C1TXQSTALbits.TXATIF) {
            C1TXQSTALbits.TXATIF = 0;
        }
    }

    if (1 == C1INTHbits.RXOVIF) {
        //CAN1_RxBufferOverflowHandler();
        if (1 == C1FIFOSTA1Lbits.RXOVIF) {
            C1FIFOSTA1Lbits.RXOVIF = 0;
        }
    }

    PIR0bits.CANIF = 0;
}

/**
 * Performs regular CAN1 operations (called every millisecond).
 */
void can1TimerIsr() {
}


// </editor-fold>


#endif  /* CAN1_BUFFERS_BASE_ADDRESS */
