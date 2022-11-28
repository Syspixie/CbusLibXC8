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
    uint16_t stdID;    // Standard ID (used for receive only)
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
    uint8_t data[8];
} canFrame_t;


// From can_types.h


typedef enum {
    CAN_TX_MSG_REQUEST_SUCCESS = 0, // Transmit message object successfully placed into Transmit FIFO
    CAN_TX_MSG_REQUEST_FIFO_FULL = 3, // Transmit FIFO is Full
} CAN_TX_MSG_REQUEST_STATUS;

typedef enum {
    CAN_NORMAL_FD_MODE = 0, //Supported only in CAN FD mode
    CAN_DISABLE_MODE = 1,
    CAN_INTERNAL_LOOPBACK_MODE = 2,
    CAN_LISTEN_ONLY_MODE = 3,
    CAN_CONFIGURATION_MODE = 4,
    CAN_EXTERNAL_LOOPBACK_MODE = 5,
    CAN_NORMAL_2_0_MODE = 6,
    CAN_RESTRICTED_OPERATION_MODE = 7,
} CAN_OP_MODES;

typedef enum {
    CAN_OP_MODE_REQUEST_SUCCESS, // Requested Operation mode set successfully
    CAN_OP_MODE_REQUEST_FAIL, // Requested Operation mode set failure. Set configuration mode before setting CAN normal or debug operation mode.
    CAN_OP_MODE_SYS_ERROR_OCCURED // System error occurred while setting Operation mode.
} CAN_OP_MODE_STATUS;

typedef enum {
    CAN_TX_FIFO_FULL,
    CAN_TX_FIFO_AVAILABLE,
} CAN_TX_FIFO_STATUS;

typedef enum {
    /*DLC_0 to DLC_8 for CAN 2.0 and CAN FD*/
    DLC_0,
    DLC_1,
    DLC_2,
    DLC_3,
    DLC_4,
    DLC_5,
    DLC_6,
    DLC_7,
    DLC_8,

    //Supported only in CAN FD mode
    /*DLC_12 to DLC_64 for CAN FD*/
    DLC_12,
    DLC_16,
    DLC_20,
    DLC_24,
    DLC_32,
    DLC_48,
    DLC_64,
} CAN_DLC;


// From can1.h


// Transmit FIFO's Custom Name
#define CAN1_TX_TXQ TXQ

typedef enum {
    TXQ = 0
} CAN1_TX_FIFO_CHANNELS;


// From can1.c


#define RX_FIFO_MSG_DATA                (8U)
#define NUM_OF_RX_FIFO                  (1U)

#define SID_LOW_WIDTH                   (8U)
#define SID_HIGH_MASK                   (0x07U)
#define EID_LOW_WIDTH                   (5U)
#define EID_LOW_POSN                    (3U)
#define EID_LOW_MASK                    (0xF8U)
#define EID_MID_WIDTH                   (8U)
#define EID_HIGH_WIDTH                  (5U)
#define EID_HIGH_MASK                   (0x1FU)
#define IDE_POSN                        (4U)
#define RTR_POSN                        (5U)

#define DLCToPayloadBytes(x)            (DLC_BYTES[(x)])
#define PLSIZEToPayloadBytes(x)         (DLCToPayloadBytes(8u + (x)))

struct CAN_FIFOREG {
    uint8_t CONL;
    uint8_t CONH;
    uint8_t CONU;
    uint8_t CONT;
    uint8_t STAL;
    uint8_t STAH;
    uint8_t STAU;
    uint8_t STAT;
    uint32_t UA;
};

typedef enum {
    CAN_RX_MSG_NOT_AVAILABLE = 0U,
    CAN_RX_MSG_AVAILABLE = 1U,
    CAN_RX_MSG_OVERFLOW = 8U
} CAN_RX_FIFO_STATUS;

typedef enum {
    FIFO1 = 1
} CAN1_RX_FIFO_CHANNELS;

struct CAN1_RX_FIFO {
    CAN1_RX_FIFO_CHANNELS channel;
    volatile uint8_t fifoHead;
};


//CAN RX FIFO Message object data field 
static uint8_t rxMsgData[RX_FIFO_MSG_DATA];

static struct CAN1_RX_FIFO rxFifos[] ={
    {FIFO1, 0u}
};

static volatile struct CAN_FIFOREG * const FIFO = (struct CAN_FIFOREG *) &C1TXQCONL;
static const uint8_t DLC_BYTES[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

// Current standard ID
bytes16_t curStdID;


static void CAN1_RX_FIFO_ResetInfo(void) {

    uint8_t index;

    for (index = 0; index < NUM_OF_RX_FIFO; index++) {
        rxFifos[index].fifoHead = 0;
    }
}

static void CAN1_RX_FIFO_Configuration(void) {

    // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE enabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE disabled; 
    C1FIFOCON1L = 0x08;

    // FRESET enabled; TXREQ disabled; UINC disabled; 
    C1FIFOCON1H = 0x04;

    // TXAT Three retransmission attempts; TXPRI 1; 
    C1FIFOCON1U = 0x20;

    // PLSIZE 8; FSIZE 12; 
    C1FIFOCON1T = 0x0B;
}

static void CAN1_RX_FIFO_FilterMaskConfiguration(void) {

    C1FLTOBJ0L = 0x00;
    C1FLTOBJ0H = 0x00;
    C1FLTOBJ0U = 0x00;
    C1FLTOBJ0T = 0x00;  // EXIDE clear: allow standard ID only
    C1MASK0L = 0x00;
    C1MASK0H = 0x00;
    C1MASK0U = 0x00;
    C1MASK0T = 0x40;    // MIDE set: filter on EXIDE
    // FLTEN0 enabled; F0BP FIFO 1; 
    C1FLTCON0L = 0x81;
}

static void CAN1_TX_FIFO_Configuration(void) {

    // TXATIE enabled; TXQEIE disabled; TXQNIE disabled; 
    C1TXQCONL = 0x10;

    // FRESET enabled; UINC disabled; 
    C1TXQCONH = 0x04;

    // TXAT 3; TXPRI 1; 
    C1TXQCONU = 0x60;

    // PLSIZE 8; FSIZE 10; 
    C1TXQCONT = 0x09;
}

static void CAN1_BitRateConfiguration(void) {

    // SJW 2; 
    C1NBTCFGL = 0x02;

    // TSEG2 2; 
    C1NBTCFGH = 0x02;

    // TSEG1 3; 
    C1NBTCFGU = 0x03;

    // BRP 63; 
    C1NBTCFGT = 0x3F;

}

static void CAN1_ErrorNotificationInterruptEnable(void) {

    PIR0bits.CANIF = 0;

    // MODIF disabled; TBCIF disabled; 
    C1INTL = 0x00;

    // IVMIF disabled; WAKIF disabled; CERRIF disabled; SERRIF disabled; 
    C1INTH = 0x00;

    // TEFIE disabled; MODIE enabled; TBCIE disabled; RXIE disabled; TXIE disabled; 
    C1INTU = 0x08;

    // IVMIE enabled; WAKIE enabled; CERRIE enabled; SERRIE enabled; RXOVIE enabled; TXATIE enabled; 
    C1INTT = 0xFC;

    PIE0bits.CANIE = 1;
}

static CAN_OP_MODES CAN1_OperationModeGet(void) {

    return C1CONUbits.OPMOD;
}

static CAN_OP_MODE_STATUS CAN1_OperationModeSet(const CAN_OP_MODES requestMode) {

    CAN_OP_MODE_STATUS status = CAN_OP_MODE_REQUEST_SUCCESS;
    CAN_OP_MODES opMode = CAN1_OperationModeGet();

    if (CAN_CONFIGURATION_MODE == opMode
            || CAN_DISABLE_MODE == requestMode
            || CAN_CONFIGURATION_MODE == requestMode) {
        C1CONTbits.REQOP = requestMode;

        while (C1CONUbits.OPMOD != requestMode) {
            //This condition is avoiding the system error case endless loop
            if (1 == C1INTHbits.SERRIF) {
                status = CAN_OP_MODE_SYS_ERROR_OCCURED;
                break;
            }
        }
    } else {
        status = CAN_OP_MODE_REQUEST_FAIL;
    }

    return status;
}

static void CAN1_Initialize(void) {

    /* Enable the CAN module */
    C1CONHbits.ON = 1;

    if (CAN_OP_MODE_REQUEST_SUCCESS == CAN1_OperationModeSet(CAN_CONFIGURATION_MODE)) {

        /* Initialize the C1FIFOBA with the start address of the CAN FIFO message object area. */
        C1FIFOBA = CAN1_BUFFERS_BASE_ADDRESS;

        // CLKSEL0 disabled; PXEDIS enabled; ISOCRCEN enabled; DNCNT 0; 
        C1CONL = 0x60;

        // ON enabled; FRZ disabled; SIDL disabled; BRSDIS enabled; WFT T11 Filter; WAKFIL enabled; 
        C1CONH = 0x97;

        // TXQEN enabled; STEF disabled; SERR2LOM disabled; ESIGM disabled; RTXAT disabled; 
        C1CONU = 0x10;

        CAN1_BitRateConfiguration();
        CAN1_TX_FIFO_Configuration();
        CAN1_RX_FIFO_Configuration();
        CAN1_RX_FIFO_FilterMaskConfiguration();
        CAN1_RX_FIFO_ResetInfo();
        CAN1_ErrorNotificationInterruptEnable();
        CAN1_OperationModeSet(CAN_NORMAL_2_0_MODE);
    }
}

static uint8_t GetRxFifoDepth(uint8_t validChannel) {

    return 1U + (FIFO[validChannel].CONT & _C1FIFOCON1T_FSIZE_MASK);
}

static CAN_RX_FIFO_STATUS GetRxFifoStatus(uint8_t validChannel) {

    return FIFO[validChannel].STAL & (CAN_RX_MSG_AVAILABLE | CAN_RX_MSG_OVERFLOW);
}

static void ReadMessageFromFifo(uint8_t* rxFifoObj, canFrame_t *rxCanMsg) {

    bytes16_t v;
    v.valueL = rxFifoObj[0];
    v.valueH = rxFifoObj[1] & 0b00000111;
    rxCanMsg->stdID = v.value;

    rxCanMsg->dlc = rxFifoObj[4];
    uint8_t len = rxCanMsg->dlcBits.DLC;
    if (len > 8) {
        len = 8;
        rxCanMsg->dlcBits.DLC = len;
    }

    utilMemcpy(rxCanMsg->data, &rxFifoObj[8], len);
}

bool CAN1_Receive(canFrame_t* rxCanMsg)
{
    uint8_t index;
    bool status = false;
    
    for (index = 0; index < NUM_OF_RX_FIFO; index++)
    {
        CAN1_RX_FIFO_CHANNELS channel = rxFifos[index].channel;
        CAN_RX_FIFO_STATUS rxMsgStatus = GetRxFifoStatus(channel);

        if (CAN_RX_MSG_AVAILABLE == (rxMsgStatus & CAN_RX_MSG_AVAILABLE))
        {
            uint8_t* rxFifoObj = (uint8_t*) FIFO[channel].UA;
            
            if (rxFifoObj != NULL)
            {
                ReadMessageFromFifo(rxFifoObj, rxCanMsg);
                FIFO[channel].CONH |= _C1FIFOCON1H_UINC_MASK;
                
                rxFifos[index].fifoHead += 1;
                if (rxFifos[index].fifoHead >= GetRxFifoDepth(channel))
                {
                    rxFifos[index].fifoHead = 0;
                }

                if (CAN_RX_MSG_OVERFLOW == (rxMsgStatus & CAN_RX_MSG_OVERFLOW))
                {
                    FIFO[channel].STAL &= ~_C1FIFOSTA1L_RXOVIF_MASK;
                }

                status = true;
            }

            break;
        }
    }
    return status;
}

static uint8_t CAN1_ReceivedMessageCountGet(void) {

    uint8_t index, totalMsgObj = 0;

    for (index = 0; index < NUM_OF_RX_FIFO; index++) {

        CAN1_RX_FIFO_CHANNELS channel = rxFifos[index].channel;
        CAN_RX_FIFO_STATUS rxMsgStatus = GetRxFifoStatus(channel);

        if (CAN_RX_MSG_AVAILABLE == (rxMsgStatus & CAN_RX_MSG_AVAILABLE)) {
            uint8_t numOfMsg, fifoDepth = GetRxFifoDepth(channel);

            if (CAN_RX_MSG_OVERFLOW == (rxMsgStatus & CAN_RX_MSG_OVERFLOW)) {
                numOfMsg = fifoDepth;
            } else {
                uint8_t fifoTail = FIFO[channel].STAH & _C1FIFOSTA1H_FIFOCI_MASK;
                uint8_t fifoHead = rxFifos[index].fifoHead;

                if (fifoTail < fifoHead) {
                    numOfMsg = ((fifoTail + fifoDepth) - fifoHead); // wrap
                } else if (fifoTail > fifoHead) {
                    numOfMsg = fifoTail - fifoHead;
                } else {
                    numOfMsg = fifoDepth;
                }
            }

            totalMsgObj += numOfMsg;
        }
    }

    return totalMsgObj;
}

static bool isTxChannel(uint8_t channel) {

    return channel < 4u && (FIFO[channel].CONL & _C1FIFOCON1L_TXEN_MASK);
}

static CAN_TX_FIFO_STATUS GetTxFifoStatus(uint8_t validChannel) {

    return (FIFO[validChannel].STAL & _C1FIFOSTA1L_TFNRFNIF_MASK);
}

static void WriteMessageToFifo(uint8_t* txFifoObj, canFrame_t* txCanMsg) {

    txFifoObj[0] = ((bytes16_t) txCanMsg->stdID).valueL;
    txFifoObj[1] = ((bytes16_t) txCanMsg->stdID).valueH;

    txFifoObj[4] = txCanMsg->dlc;

    utilMemcpy(&txFifoObj[8], txCanMsg->data, txCanMsg->dlcBits.DLC);
}

static CAN_TX_MSG_REQUEST_STATUS ValidateTransmission(uint8_t validChannel) {

    CAN_TX_MSG_REQUEST_STATUS txMsgStatus = CAN_TX_MSG_REQUEST_SUCCESS;

    if (CAN_TX_FIFO_FULL == GetTxFifoStatus(validChannel)) {
        txMsgStatus |= CAN_TX_MSG_REQUEST_FIFO_FULL;
    }

    return txMsgStatus;
}

static CAN_TX_MSG_REQUEST_STATUS CAN1_Transmit(const CAN1_TX_FIFO_CHANNELS fifoChannel, canFrame_t* txCanMsg) {

    CAN_TX_MSG_REQUEST_STATUS status = CAN_TX_MSG_REQUEST_FIFO_FULL;

    if (isTxChannel(fifoChannel)) {
        status = ValidateTransmission(fifoChannel);
        if (CAN_TX_MSG_REQUEST_SUCCESS == status) {
            uint8_t *txFifoObj = (uint8_t *) FIFO[fifoChannel].UA;

            if (txFifoObj != NULL) {
                WriteMessageToFifo(txFifoObj, txCanMsg);
                FIFO[fifoChannel].CONH |= (_C1FIFOCON1H_TXREQ_MASK | _C1FIFOCON1H_UINC_MASK);
            }
        }
    }

    return status;
}

static CAN_TX_FIFO_STATUS CAN1_TransmitFIFOStatusGet(const CAN1_TX_FIFO_CHANNELS fifoChannel) {

    CAN_TX_FIFO_STATUS status = CAN_TX_FIFO_FULL;

    if (isTxChannel(fifoChannel)) {
        status = GetTxFifoStatus(fifoChannel);
    }

    return status;
}

static bool CAN1_IsBusOff(void) {
    return C1TRECUbits.TXBO;
}

static bool CAN1_IsRxErrorPassive(void) {
    return C1TRECUbits.RXBP;
}

static bool CAN1_IsRxErrorWarning(void) {
    return C1TRECUbits.RXWARN;
}

static bool CAN1_IsRxErrorActive(void) {
    return !CAN1_IsRxErrorPassive();
}

static bool CAN1_IsTxErrorPassive(void) {
    return C1TRECUbits.TXBP;
}

static bool CAN1_IsTxErrorWarning(void) {
    return C1TRECUbits.TXWARN;
}

static bool CAN1_IsTxErrorActive(void) {
    return !CAN1_IsTxErrorPassive();
}

static void CAN1_Sleep(void) {

    C1INTHbits.WAKIF = 0;
    C1INTTbits.WAKIE = 1;

    CAN1_OperationModeSet(CAN_DISABLE_MODE);
}

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
//
//void __interrupt(irq(CANRX), base(IVT_BASE_ADDRESS), low_priority) CAN1_RXI_ISR(void) {
//
//    if (1 == C1FIFOSTA1Lbits.TFNRFNIF) {
//        //CAN1_FIFO1NotEmptyHandler();
//        // flag readonly
//    }
//
//}
//
//void __interrupt(irq(CANTX), base(IVT_BASE_ADDRESS), low_priority) CAN1_TXI_ISR(void) {
//
//    if (1 == C1TXQSTALbits.TXQNIF) {
//        //CAN1_TXQNotFullHandler();
//        // flag readonly
//    }
//
//}




//******************************************************************************




// extern
#ifdef INCLUDE_CBUS_CAN_STATS
volatile stats_t stats = {.bytes =
    {[0 ... sizeof (stats.bytes) - 1] = 0}};
#endif

/**
 * Initialises the CAN1 peripheral.
 */
void initCan1() {

    CAN1_Initialize();
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
    CAN1_Transmit(0, &msg);
}

/**
 * Sends an RTR response.
 */
void can1SendRtrResponse() {

    canFrame_t msg;
    msg.stdID = curStdID.value;
    msg.dlc = 0;
    CAN1_Transmit(0, &msg);
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
    utilMemcpy(msg.data, cbusMsg, msg.dlcBits.DLC);
    CAN1_Transmit(0, &msg);
}

/**
 * Receives a message.
 * 
 * @param msgCheckFunc function to pre-process message and check for CBUS message
 * @return -1: no message; 0: not a CBUS message; 1: is a CBUS message
 * @post cbusMsg[] CBUS message
 */
int8_t can1Receive(bool(* msgCheckFunc)(uint16_t stdID, uint8_t dataLen, volatile uint8_t* data)) {

    // If no messages, we're done
    if (CAN1_ReceivedMessageCountGet() == 0) return -1;

    // Get message; done if error
    canFrame_t msg;
    if (!CAN1_Receive(&msg)) return -1;

    // Check for things we can't handle (should never happen if hardware config OK)
    if (msg.dlc & 0b11010000) return -1;

    // Check for CBUS message
    volatile uint8_t* data = (msg.dlcBits.RTR) ? NULL : msg.data;
    bool haveMsg = msgCheckFunc((uint16_t) msg.stdID, msg.dlcBits.DLC, data);

    return haveMsg ? 1 : 0;
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">

/**
 * Performs regular CAN1 operations (called every millisecond).
 */
void can1TimerIsr() {
}


// </editor-fold>


#endif  /* CAN1_BUFFERS_BASE_ADDRESS */
