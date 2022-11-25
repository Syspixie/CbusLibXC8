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


#define TX_PRIORITY_HIKE_MILLIS 750     // 0.75 seconds
#define TX_TIMEOUT_MILLIS 1000          // 1 second


// From can_types.h


typedef union {
    uint8_t msgfields;
    struct {
        uint8_t idType : 1; // 1 bit (Standard Frame or Extended Frame)
        uint8_t frameType : 1; // 1 bit (Data Frame or RTR Frame)
        uint8_t dlc : 4; // 4 bit (No of data bytes a message frame contains)
        uint8_t formatType : 1; // 1 bit (CAN 2.0 Format or CAN_FD Format)
        uint8_t brs : 1; // 1 bit (Bit Rate Switch)
    };
} CAN_MSG_FIELD;

typedef struct {
    uint32_t msgId; // 29 bit (SID: 11bit, EID:18bit)
    CAN_MSG_FIELD field; // CAN TX/RX Message Object Control
    volatile uint8_t *data; // Pointer to message data
} CAN_MSG_OBJ;

typedef enum {
    CAN_NON_BRS_MODE = 0,
    CAN_BRS_MODE = 1 //Supported only in CAN FD mode
} CAN_MSG_OBJ_BRS_MODE;

typedef enum {
    CAN_FRAME_STD = 0,
    CAN_FRAME_EXT = 1,
} CAN_MSG_OBJ_ID_TYPE;

typedef enum {
    CAN_FRAME_DATA = 0,
    CAN_FRAME_RTR = 1,
} CAN_MSG_OBJ_FRAME_TYPE;

typedef enum {
    CAN_2_0_FORMAT = 0,
    CAN_FD_FORMAT = 1 //Supported only in CAN FD mode
} CAN_MSG_OBJ_TYPE;

typedef enum {
    CAN_TX_MSG_REQUEST_SUCCESS = 0, // Transmit message object successfully placed into Transmit FIFO
    CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR = 1, // Transmit message object DLC size is more than Transmit FIFO configured DLC size
    CAN_TX_MSG_REQUEST_BRS_ERROR = 2, // Transmit FIFO is configured has Non BRS mode and CAN TX Message object has BRS enabled
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

typedef enum {
    FIFO1 = 1
} CAN1_RX_FIFO_CHANNELS;


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
#define BRS_POSN                        (6U)
#define FDF_POSN                        (7U)

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

struct CAN1_RX_FIFO {
    CAN1_RX_FIFO_CHANNELS channel;
    volatile uint8_t fifoHead;
};

//CAN RX FIFO Message object data field 
static volatile uint8_t rxMsgData[RX_FIFO_MSG_DATA];

static struct CAN1_RX_FIFO rxFifos[] ={
    {FIFO1, 0u}
};

static volatile struct CAN_FIFOREG * const FIFO = (struct CAN_FIFOREG *) &C1TXQCONL;
static const uint8_t DLC_BYTES[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};



static void CAN1_RX_FIFO_ResetInfo(void) {

    uint8_t index;

    for (index = 0; index < NUM_OF_RX_FIFO; index++) {
        rxFifos[index].fifoHead = 0;
    }
}

static void CAN1_RX_FIFO_Configuration(void) {

    // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE enabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE enabled; 
    C1FIFOCON1L = 0x09;

    // FRESET enabled; TXREQ disabled; UINC disabled; 
    C1FIFOCON1H = 0x04;

    // TXAT Unlimited number of retransmission attempts; TXPRI 1; 
    C1FIFOCON1U = 0x60;

    // PLSIZE 8; FSIZE 12; 
    C1FIFOCON1T = 0x0B;

    C1INTUbits.RXIE = 1;

    PIR4bits.CANRXIF = 0;
    PIE4bits.CANRXIE = 1;
}

static void CAN1_RX_FIFO_FilterMaskConfiguration(void) {

    // FLTEN0 enabled; F0BP FIFO 1; 
    C1FLTOBJ0L = 0x00;
    C1FLTOBJ0H = 0x00;
    C1FLTOBJ0U = 0x00;
    C1FLTOBJ0T = 0x00;
    C1MASK0L = 0xFC;
    C1MASK0H = 0x07;
    C1MASK0U = 0x00;
    C1MASK0T = 0x40;
    C1FLTCON0L = 0x81;

}

static void CAN1_TX_FIFO_Configuration(void) {

    // TXATIE enabled; TXQEIE disabled; TXQNIE enabled; 
    C1TXQCONL = 0x11;

    // FRESET enabled; UINC disabled; 
    C1TXQCONH = 0x04;

    // TXAT 3; TXPRI 1; 
    C1TXQCONU = 0x60;

    // PLSIZE 8; FSIZE 10; 
    C1TXQCONT = 0x09;

    C1INTUbits.TXIE = 1;

    PIR4bits.CANTXIF = 0;
    PIE4bits.CANTXIE = 1;
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

    // TEFIE disabled; MODIE enabled; TBCIE disabled; RXIE enabled; TXIE enabled; 
    C1INTU = 0x0B;

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
        C1FIFOBA = 0x3800;

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

static void ReadMessageFromFifo(uint8_t *rxFifoObj, CAN_MSG_OBJ *rxCanMsg) {

    uint32_t msgId;
    uint8_t status = rxFifoObj[4];
    const uint8_t payloadOffsetBytes =
            4U // ID
            + 1U // FDF, BRS, RTR, ...
            + 1U // FILHIT, ...
            + 2U; // Unimplemented

    rxCanMsg->field.dlc = status;
    rxCanMsg->field.idType = (status & (1UL << IDE_POSN)) ? CAN_FRAME_EXT : CAN_FRAME_STD;
    rxCanMsg->field.frameType = (status & (1UL << RTR_POSN)) ? CAN_FRAME_RTR : CAN_FRAME_DATA;
    rxCanMsg->field.brs = (status & (1UL << BRS_POSN)) ? CAN_BRS_MODE : CAN_NON_BRS_MODE;
    rxCanMsg->field.formatType = (status & (1UL << FDF_POSN)) ? CAN_FRAME_EXT : CAN_FRAME_STD;

    msgId = rxFifoObj[1] & SID_HIGH_MASK;
    msgId <<= SID_LOW_WIDTH;
    msgId |= rxFifoObj[0];
    if (CAN_FRAME_EXT == rxCanMsg->field.idType) {
        msgId <<= EID_HIGH_WIDTH;
        msgId |= (rxFifoObj[3] & EID_HIGH_MASK);
        msgId <<= EID_MID_WIDTH;
        msgId |= rxFifoObj[2];
        msgId <<= EID_LOW_WIDTH;
        msgId |= (rxFifoObj[1] & EID_LOW_MASK) >> EID_LOW_POSN;
    }
    rxCanMsg->msgId = msgId;

    utilMemcpy(rxMsgData, rxFifoObj + payloadOffsetBytes, DLCToPayloadBytes(rxCanMsg->field.dlc));
    rxCanMsg->data = rxMsgData;
}

static bool Receive(uint8_t index, CAN1_RX_FIFO_CHANNELS channel, CAN_MSG_OBJ *rxCanMsg) {

    bool status = false;
    CAN_RX_FIFO_STATUS rxMsgStatus = GetRxFifoStatus(channel);

    if (CAN_RX_MSG_AVAILABLE == (rxMsgStatus & CAN_RX_MSG_AVAILABLE)) {

        uint8_t *rxFifoObj = (uint8_t *) FIFO[channel].UA;

        if (rxFifoObj != NULL) {

            ReadMessageFromFifo(rxFifoObj, rxCanMsg);
            FIFO[channel].CONH |= _C1FIFOCON1H_UINC_MASK;

            rxFifos[index].fifoHead += 1;
            if (rxFifos[index].fifoHead >= GetRxFifoDepth(channel)) {
                rxFifos[index].fifoHead = 0;
            }

            if (CAN_RX_MSG_OVERFLOW == (rxMsgStatus & CAN_RX_MSG_OVERFLOW)) {
                FIFO[channel].STAL &= ~_C1FIFOSTA1L_RXOVIF_MASK;
            }

            status = true;
        }
    }
    return status;
}

static bool CAN1_Receive(CAN_MSG_OBJ *rxCanMsg) {

    uint8_t index;
    bool status = false;

    for (index = 0; index < NUM_OF_RX_FIFO; index++) {
        status = Receive(index, rxFifos[index].channel, rxCanMsg);

        if (status) {
            break;
        }
    }

    return status;
}

static bool CAN1_ReceiveFrom(const CAN1_RX_FIFO_CHANNELS channel, CAN_MSG_OBJ *rxCanMsg) {

    uint8_t index;
    bool status = false;

    for (index = 0; index < NUM_OF_RX_FIFO; index++) {
        if (channel == rxFifos[index].channel) {
            status = Receive(index, channel, rxCanMsg);
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

static void WriteMessageToFifo(uint8_t *txFifoObj, CAN_MSG_OBJ *txCanMsg) {

    uint32_t msgId = txCanMsg->msgId;
    uint8_t status;
    const uint8_t payloadOffsetBytes =
            4U // ID
            + 1U // FDF, BRS, RTR, ...
            + 1U // SEQ[6:0], ESI
            + 2U; // SEQ

    if (CAN_FRAME_EXT == txCanMsg->field.idType) {
        txFifoObj[1] = (msgId << EID_LOW_POSN) & EID_LOW_MASK;
        msgId >>= EID_LOW_WIDTH;
        txFifoObj[2] = (uint8_t) msgId;
        msgId >>= EID_MID_WIDTH;
        txFifoObj[3] = (msgId & EID_HIGH_MASK);
        msgId >>= EID_HIGH_WIDTH;
    } else {
        txFifoObj[1] = txFifoObj[2] = txFifoObj[3] = 0;
    }

    txFifoObj[0] = (uint8_t) msgId;
    msgId >>= SID_LOW_WIDTH;
    txFifoObj[1] |= (msgId & SID_HIGH_MASK);

    status = txCanMsg->field.dlc;
    status |= (txCanMsg->field.idType << IDE_POSN);
    status |= (txCanMsg->field.frameType << RTR_POSN);
    status |= (txCanMsg->field.brs << BRS_POSN);
    status |= (txCanMsg->field.formatType << FDF_POSN);
    txFifoObj[4] = status;

    if (CAN_FRAME_DATA == txCanMsg->field.frameType) {
        utilMemcpy(txFifoObj + payloadOffsetBytes, txCanMsg->data, DLCToPayloadBytes(txCanMsg->field.dlc));
    }
}

static CAN_TX_MSG_REQUEST_STATUS ValidateTransmission(uint8_t validChannel, CAN_MSG_OBJ *txCanMsg) {

    CAN_TX_MSG_REQUEST_STATUS txMsgStatus = CAN_TX_MSG_REQUEST_SUCCESS;
    CAN_MSG_FIELD field = txCanMsg->field;
    uint8_t plsize = 0;

    if (CAN_BRS_MODE == field.brs && (CAN_NORMAL_2_0_MODE == CAN1_OperationModeGet())) {
        txMsgStatus |= CAN_TX_MSG_REQUEST_BRS_ERROR;
    }

    if (field.dlc > DLC_8 && (CAN_2_0_FORMAT == field.formatType || CAN_NORMAL_2_0_MODE == CAN1_OperationModeGet())) {
        txMsgStatus |= CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR;
    }

    if (DLCToPayloadBytes(field.dlc) > PLSIZEToPayloadBytes(plsize)) {
        txMsgStatus |= CAN_TX_MSG_REQUEST_DLC_EXCEED_ERROR;
    }

    if (CAN_TX_FIFO_FULL == GetTxFifoStatus(validChannel)) {
        txMsgStatus |= CAN_TX_MSG_REQUEST_FIFO_FULL;
    }

    return txMsgStatus;
}

static CAN_TX_MSG_REQUEST_STATUS CAN1_Transmit(const CAN1_TX_FIFO_CHANNELS fifoChannel, CAN_MSG_OBJ *txCanMsg) {

    CAN_TX_MSG_REQUEST_STATUS status = CAN_TX_MSG_REQUEST_FIFO_FULL;

    if (isTxChannel(fifoChannel)) {
        status = ValidateTransmission(fifoChannel, txCanMsg);
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

void __interrupt(irq(CANRX), base(IVT_BASE_ADDRESS), low_priority) CAN1_RXI_ISR(void) {

    if (1 == C1FIFOSTA1Lbits.TFNRFNIF) {
        //CAN1_FIFO1NotEmptyHandler();
        // flag readonly
    }

}

void __interrupt(irq(CANTX), base(IVT_BASE_ADDRESS), low_priority) CAN1_TXI_ISR(void) {

    if (1 == C1TXQSTALbits.TXQNIF) {
        //CAN1_TXQNotFullHandler();
        // flag readonly
    }

}




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
 * Sends an RTR request.
 * 
 * @pre encodedCanBusID populated
 */
void can1SendRtrRequest() {
}

/**
 * Sends an RTR response.
 * 
 * @pre encodedCanBusID populated
 */
void can1SendRtrResponse() {
}

/**
 * Sends a CBUS message.
 * 
 * @pre encodedCanBusID populated
 * @pre message in cbusMsg[]
 */
void can1Transmit() {
}

/**
 * Receives a message.
 * 
 * @param msgCheckFunc function to pre-process message and check for CBUS message
 * @return -1: no message; 0: not a CBUS message; 1: is a CBUS message
 * @post cbusMsg[] CBUS message
 */
int8_t can1Receive(bool(* msgCheckFunc)(uint8_t id, uint8_t dataLen, volatile uint8_t* data)) {

    return 0;
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">

/**
 * Performs regular CAN1 operations (called every millisecond).
 */
void can1TimerIsr() {

}


// </editor-fold>


#endif  /* CAN1_BUFFERS_BASE_ADDRESS */
