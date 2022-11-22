/**
 * @file CbusLibXC8 ecan.c
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
 * @note Uses hardware ECAN module.
 *
 * The ECAN peripheral is configured in 'mode 2', with 3 TX buffers and 8 RX
 * buffers in a hardware FIFO.  Only standard ID messages are received and
 * transmitted; extended ID messages are filtered out in hardware.  Bus speed
 * is set at 125kbps.
 * 
 * 'Normal' message transmission is done through TXB0.  TXB1 and TXB2 are used
 * for sending, and responding to, RTR messages as part of self-enumeration.
 * Starting TXB1 and TXB2 transmissions are the only times buffers are accessed
 * outside of an ISR, and are done with interrupts disabled.
 * 
 * The software FIFOs use unconstrained 'pointers' to the newest and oldest
 * entries - the modulus of the pointer value is used as the FIFO index.  This
 * is very efficient, and works fine as long as the FIFO queue lengths are a
 * power of 2 (otherwise there is a jump in the index value when the pointer
 * overflows).  For the 8-bit pointers used here, FIFO queue lengths of 4, 8,
 * 16, 32, 64 and 128 are valid.
 * 
 * The FIFO queues are also thread safe as long as the independent threads only
 * modify one pointer.  In our case for example, rxFifoNewest is only modified
 * in ISR (low priority), and rxFifoOldest is only modified in normal mode,
 * which can be done without disabling interrupts.  The same applies to
 * txFifoOldest and txFifoNewest with the thread roles reversed.
 *  
 * [ See Microchip application note AN930 for ECAN code example ]
 */

#if defined(ECAN_BUFFERS_BASE_ADDRESS)


#include "ecan.h"
#include "hardware.h"
#include "util.h"
#include "millis.h"
#include "stats.h"
#include "module.h"


#define RX_FIFO_WM_TARGET_LENGTH (RX_FIFO_LENGTH - 3)
#define RX_FIFO_CUR_LENGTH ((uint8_t) (rxFifoNewest - rxFifoOldest))
#define TX_FIFO_CUR_LENGTH ((uint8_t) (txFifoNewest - txFifoOldest))

#define TX_PRIORITY_HIKE_MILLIS 750     // 0.75 seconds
#define TX_TIMEOUT_MILLIS 1000          // 1 second


// ECANCON register EWIN<4:0> bits determine which RX/TX buffer is mapped into
// the Access Bank address (ECAN_BUFFERS_BASE_ADDRESS)
typedef enum {
    ewinTX = 0b00011,
    ewinTXB0 = 0b00011,
    ewinTXB1 = 0b00100,
    ewinTXB2 = 0b00101,
    ewinRxFifo = 0b10000,
    ewinRXB0 = 0b10000,
    ewinRXB1 = 0b10001,
    ewinProgB0 = 0b10010,
    ewinProgB1 = 0b10011,
    ewinProgB2 = 0b10100,
    ewinProgB3 = 0b10101,
    ewinProgB4 = 0b10110,
    ewinProgB5 = 0b10111
} ewin_t;

// RX/TX buffer location names mapped to ECAN_BUFFERS_BASE_ADDRESS
volatile struct {
    union {
        uint8_t CON;
        RXB0CONbits_t RXCONbits;
        TXB0CONbits_t TXCONbits;
    };
    uint8_t SIDH;
    uint8_t SIDL;
    uint8_t EIDH;
    uint8_t EIDL;

    union {
        uint8_t DLC;
        B0DLCbits_t DLCbits;
    };
    uint8_t data[8];
} ewin __at(ECAN_BUFFERS_BASE_ADDRESS);


// extern
#ifdef INCLUDE_CBUS_CAN_STATS
volatile stats_t stats = {.bytes =
    {[0 ... sizeof (stats.bytes) - 1] = 0}};
#endif

// Receive software FIFO
volatile canFrame_t rxFifo[RX_FIFO_LENGTH];
volatile uint8_t rxFifoNewest = 0;
volatile uint8_t rxFifoOldest = 0;

// Transmit software FIFO
volatile canFrame_t txFifo[TX_FIFO_LENGTH]; // Transmit FIFO
volatile uint8_t txFifoNewest = 0;
volatile uint8_t txFifoOldest = 0;

// Transmit buffer start times for determining timeout
uint16_t txb0StartedMillis;
uint16_t txb1StartedMillis;
uint16_t txb2StartedMillis;


/**
 * Initialises the ECAN peripheral.
 */
void initEcan() {

    CANCON = 0x80;
    while ((CANSTAT & 0xE0) != 0x80); // wait until ECAN is in config mode

    ECANCON = 0b10100000; // CAN mode 2; FIFOWM when one receive buffer remains
    BSEL0 = 0b00000000; // All programmable buffers receive

#if defined(CPU_FAMILY_PIC18_K80)
    CIOCON = 0b00100000; // TX drives Vdd when recessive
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    CIOCON = 0b00000000;
#endif

    // Receive masks
    RXM0EIDH = 0;
    RXM0EIDL = 0;
    RXM0SIDH = 0;
    RXM0SIDL = 0b00001000; // Messages selected by the EXIDEN bit in RXFnSIDL will be accepted
    RXM1EIDH = 0;
    RXM1EIDL = 0;
    RXM1SIDH = 0;
    RXM1SIDL = 0b00001000; // Messages selected by the EXIDEN bit in RXFnSIDL will be accepted

    // Receive filters
    RXF0EIDH = 0;
    RXF0EIDL = 0;
    RXF0SIDH = 0;
    RXF0SIDL = 0b10000000; // Filter will only accept standard ID messages
    RXF1EIDH = 0;
    RXF1EIDL = 0;
    RXF1SIDH = 0;
    RXF1SIDL = 0;
    RXF2EIDH = 0;
    RXF2EIDL = 0;
    RXF2SIDH = 0;
    RXF2SIDL = 0;
    RXF3EIDH = 0;
    RXF3EIDL = 0;
    RXF3SIDH = 0;
    RXF3SIDL = 0;
    RXF4EIDH = 0;
    RXF4EIDL = 0;
    RXF4SIDH = 0;
    RXF4SIDL = 0;
    RXF5EIDH = 0;
    RXF5EIDL = 0;
    RXF5SIDH = 0;
    RXF5SIDL = 0;

    // Link all filters to RXB0
    RXFBCON0 = 0;
    RXFBCON1 = 0;
    RXFBCON2 = 0;
    RXFBCON3 = 0;
    RXFBCON4 = 0;
    RXFBCON5 = 0;
    RXFBCON6 = 0;
    RXFBCON7 = 0;

    // Link all filters to mask 0
    MSEL0 = 0;
    MSEL1 = 0;
    MSEL2 = 0;
    MSEL3 = 0;

    BRGCON1 = 0b00001111; // Sync 1xTQ; Baud rate 125kbps
    BRGCON2 = 0b10011110; // Segment 1 4xTQ; propagation 7xTQ
    BRGCON3 = 0b00000011; // Segment 2 4xTQ

    CANCON = 0x00;
    while ((CANSTAT & 0xE0) != 0x00); // wait until ECAN is in Normal mode

    IPR5bits.IRXIP = 0;
    PIR5bits.IRXIF = 0;
    PIE5bits.IRXIE = 1; // Bus error received interrupt

    IPR5bits.ERRIP = 0;
    PIR5bits.ERRIF = 0;
    PIE5bits.ERRIE = 1; // Bus error interrupt

    TXB0CONbits.TXBIF = 0;
    TXBIEbits.TXB0IE = 1; // TXB0 interrupt

    TXB1CONbits.TXBIF = 0;
    TXBIEbits.TXB1IE = 1; // TXB1 interrupt

    TXB2CONbits.TXBIF = 0;
    TXBIEbits.TXB2IE = 1; // TXB2 interrupt

    IPR5bits.TXBnIP = 0;
    PIR5bits.TXBnIF = 0;
    PIE5bits.TXBnIE = 1; // Mode 2 transmit interrupt

    IPR5bits.RXBnIP = 0;
    PIR5bits.RXBnIF = 0;
    PIE5bits.RXBnIE = 1; // Mode 2 receive interrupt

    IPR5bits.FIFOWMIP = 0;
    PIR5bits.FIFOWMIF = 0;
    PIE5bits.FIFOWMIE = 1; // High water interrupt (one FIFO buffer remaining)

    // Clear RX buffer control registers
    for (uint8_t i = 0; i <= 7; ++i) {
        ECANCONbits.EWIN = ewinRxFifo | i;
        ewin.CON = 0;
    }

    // Configure TX buffer control registers
    ECANCONbits.EWIN = ewinTXB0;
    ewin.CON = 0b00000000; // Priority 0 (lowest)
    ECANCONbits.EWIN = ewinTXB1;
    ewin.CON = 0b00000010; // Priority 2
    ECANCONbits.EWIN = ewinTXB2;
    ewin.CON = 0b00000011; // Priority 3 (highest)
}

/**
 * Sends an RTR request.
 * 
 * @pre encodedCanBusID populated
 */
void ecanSendRtrRequest() {

    // Send RTR request, which will trigger a response from all currently
    // active nodes.
    INTERRUPTbits_GIEL = 0;
    ECANCONbits.EWIN = ewinTXB1;
    ewin.SIDH = encodedCanBusID.valueH;
    ewin.SIDL = encodedCanBusID.valueL;
    ewin.DLC = 0b01000000;
    ewin.TXCONbits.TXREQ = 1;
    INTERRUPTbits_GIEL = 1;

    // Note time for timeout checking
    txb1StartedMillis = getMillisShort();
}

/**
 * Sends an RTR response.
 * 
 * @pre encodedCanBusID populated
 */
void ecanSendRtrResponse() {

    // Send response which includes our CAN ID
    INTERRUPTbits_GIEL = 0;
    ECANCONbits.EWIN = ewinTXB2;
    ewin.SIDH = encodedCanBusID.valueH;
    ewin.SIDL = encodedCanBusID.valueL;
    ewin.DLC = 0;
    ewin.TXCONbits.TXREQ = 1;
    INTERRUPTbits_GIEL = 1;

    // Note time for timeout checking
    txb2StartedMillis = getMillisShort();
}

/**
 * Sends a CBUS message.
 * 
 * @pre encodedCanBusID populated
 * @pre message in cbusMsg[]
 */
void ecanTransmit() {

    // If room in the FIFO...
    if (TX_FIFO_CUR_LENGTH < TX_FIFO_LENGTH) {

        // FIFO index
        uint8_t idx = txFifoNewest % TX_FIFO_LENGTH;

        // Copy data
        uint8_t dataLen = (cbusMsg[0] >> 5) + 1;
        txFifo[idx].dlc = dataLen;
        utilMemcpy(txFifo[idx].data, cbusMsg, dataLen);

        // Update FIFO
        ++txFifoNewest;
#ifdef INCLUDE_CBUS_CAN_STATS
        ++stats.txFifoUsed;
        if (stats.txFifoUsed > stats.txFifoMaxUsed) stats.txFifoMaxUsed = stats.txFifoUsed;
#endif

        // FIFO full
#ifdef INCLUDE_CBUS_CAN_STATS
    } else {
        ++stats.txSwFifoFull;
#endif
    }
}

/**
 * Receives a message.
 * 
 * @param msgCheckFunc function to pre-process message and check for CBUS message
 * @return -1: no message; 0: not a CBUS message; 1: is a CBUS message
 * @post cbusMsg[] CBUS message
 */
int8_t ecanReceive(bool (* msgCheckFunc)(uint8_t id, uint8_t dlc, volatile uint8_t* data)) {

    // If no messages, we're done
    if (RX_FIFO_CUR_LENGTH == 0) return -1;

    // FIFO index
    uint8_t idx = rxFifoOldest % RX_FIFO_LENGTH;

    // Check for CBUS message
    bool haveMsg = msgCheckFunc(rxFifo[idx].id, rxFifo[idx].dlc, rxFifo[idx].data);

    // Update FIFO
    ++rxFifoOldest;
#ifdef INCLUDE_CBUS_CAN_STATS
    --stats.rxFifoUsed;
#endif

    return haveMsg ? 1 : 0;
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Checks for transmit buffer errors, and abort the transmit if found.
 */
static void txCheckErrorsIsr() {

    // Loop through TX buffers
    for (uint8_t i = 0; i < 3; ++i) {
        ECANCONbits.EWIN = ewinTX + i;

        // Abort transmit on error
        if (ewin.TXCONbits.TXREQ && ewin.TXCONbits.TXERR) {
#ifdef INCLUDE_CBUS_CAN_STATS
            ++stats.txBusErr;
#endif
            ewin.TXCONbits.TXREQ = 0; // Abort TX
        }
    }
}

/**
 * Transmits the next entry in the FIFO queue.
 * 
 * @pre TXB0 mapped to the Access Bank.
 */
static void txb0NextIsr() {

    // FIFO index
    uint8_t idx = txFifoOldest % TX_FIFO_LENGTH;

    // Copy encoded CAN ID into buffer
    ewin.SIDH = encodedCanBusID.valueH;
    ewin.SIDL = encodedCanBusID.valueL;

    // Copy data into buffer
    ewin.DLC = txFifo[idx].dlc;
    utilMemcpy(ewin.data, txFifo[idx].data, ewin.DLCbits.DLC);

    // Update FIFO
    ++txFifoOldest;
#ifdef INCLUDE_CBUS_CAN_STATS
    --stats.txFifoUsed;
#endif

    // Start transmit
    ewin.TXCONbits.TXREQ = 1;
    txb0StartedMillis = millisTicks.words[0];
}

/**
 * Receives the next entry from the hardware FIFO into the software FIFO queue.
 * 
 * @pre The appropriate RX buffer mapped to the Access Bank.
 */
static void rxNextIsr() {

    // FIFO index
    uint8_t idx = rxFifoNewest % RX_FIFO_LENGTH;

    // Decode standard CAN ID from buffer
    bytes16_t v;
    v.valueL = ewin.SIDL;
    v.valueH = ewin.SIDH;
    rxFifo[idx].id = (v.value >> 5) & 0b01111111;

    // Copy data from buffer
    rxFifo[idx].dlc = ewin.DLC;
    utilMemcpy(rxFifo[idx].data, ewin.data, ewin.DLCbits.DLC);

    // Update FIFO
    ++rxFifoNewest;
#ifdef INCLUDE_CBUS_CAN_STATS
    ++stats.rxFifoUsed;
    if (stats.rxFifoUsed > stats.rxFifoMaxUsed) stats.rxFifoMaxUsed = stats.rxFifoUsed;
#endif

    // Finish receive
    ewin.RXCONbits.RXFUL = 0;
}

/**
 * Checks for transmit buffer timeout.
 * 
 * @pre A TX buffer mapped to the Access Bank.
 */
static void txCheckTimeoutIsr(uint16_t started) {

    uint16_t tSince = millisTicks.words[0] - started;

    // If transmit has been running at low priority (as determined by the top
    // CAN ID bits) for some time, abort it, change to high priority, then
    // restart it.
    if ((ewin.SIDH & 0b11000000) && tSince >= TX_PRIORITY_HIKE_MILLIS) {
        ewin.TXCONbits.TXREQ = 0; // Abort
        ewin.SIDH &= ~0b11000000; // Set highest priority
        ewin.TXCONbits.TXREQ = 1; // Retry
#ifdef INCLUDE_CBUS_CAN_STATS
        ++stats.txPrioHike;
#endif

        // If transmit has been running for too long, abort it.
    } else if (tSince >= TX_TIMEOUT_MILLIS) {
        ewin.TXCONbits.TXREQ = 0; // Abort
#ifdef INCLUDE_CBUS_CAN_STATS
        ++stats.txTmoErr;
#endif
    }
}

/**
 * Called on ECAN receive message interrupt.
 */
#if defined(IVT_BASE_ADDRESS)
void __interrupt(irq(RXB1IF), base(IVT_BASE_ADDRESS), low_priority) ECAN_RXBnI_ISR(void) {
#else
void ECAN_RXBnI_ISR(void) {
#endif

    // Loop for all used RX FIFO buffers, and room in the software FIFO
    while (COMSTATbits.NOT_FIFOEMPTY && RX_FIFO_CUR_LENGTH < RX_FIFO_LENGTH) {
        ECANCONbits.EWIN = ewinRxFifo | (CANCON & 0x07);

        // Get the received data
        rxNextIsr();
#ifdef INCLUDE_CBUS_CAN_STATS
        ++stats.rxMsgCount;
#endif
    }

    // Clear interrupt flag
    PIR5bits.RXBnIF = 0;
}

/**
 * Called on ECAN transmit message interrupt.
 */
#if defined(IVT_BASE_ADDRESS)
void __interrupt(irq(TXB2IF), base(IVT_BASE_ADDRESS), low_priority) ECAN_TXBnI_ISR(void) {
#else
void ECAN_TXBnI_ISR(void) {
#endif

    // Loop through TX buffers looking for the source of the interrupt
    for (uint8_t i = 0; i < 3; ++i) {
        ECANCONbits.EWIN = ewinTX + i;
        if (ewin.TXCONbits.TXBIF) {

            // Transmit finished
#ifdef INCLUDE_CBUS_CAN_STATS
            ++stats.txMsgCount;
#endif
            ewin.TXCONbits.TXBIF = 0;

            // If TXB0, and FIFO not empty, start the next transmit
            if (i == 0 && TX_FIFO_CUR_LENGTH > 0) txb0NextIsr();
        }
    }

    // Clear interrupt flag
    PIR5bits.TXBnIF = 0;
}

/**
 * Called on ECAN FIFO high water interrupt.
 */
#if defined(IVT_BASE_ADDRESS)
void __interrupt(irq(RXB0IF), base(IVT_BASE_ADDRESS), low_priority) ECAN_FIFOWMI_ISR(void) {
#else
void ECAN_FIFOWMI_ISR(void) {
#endif

    // It was found that allowing the hardware FIFO to fill up, and not have
    // any space in the software FIFO, the system would 'lock-up' until
    // restarted.  Clearing a bit of space in the software FIFO stops this
    // from happening; reducing the FIFO queue so that there are at least
    // 3 free entries appears to work well.  Doing this by changing
    // rxFifoNewest might appear illogical, but it retains the thread safe
    // nature of the FIFO.
    while (RX_FIFO_CUR_LENGTH > RX_FIFO_WM_TARGET_LENGTH) {
        --rxFifoNewest;
#ifdef INCLUDE_CBUS_CAN_STATS
        --stats.rxFifoUsed;
        --stats.rxMsgCount;
        ++stats.rxMsgDiscarded;
#endif
    }

    // Clear interrupt flag
    PIR5bits.FIFOWMIF = 0;
}

/**
 * Called on ECAN error interrupt.
 */
#if defined(IVT_BASE_ADDRESS)
void __interrupt(irq(ERRIF), base(IVT_BASE_ADDRESS), low_priority) ECAN_ERRI_ISR(void) {
#else
void ECAN_ERRI_ISR(void) {
#endif

    if (COMSTATbits.RXBNOVFL) {
#ifdef INCLUDE_CBUS_CAN_STATS
        ++stats.rxHwFifoFull;
#endif
        COMSTATbits.RXBNOVFL = 0;
    }

    // Clear interrupt flag
    PIR5bits.ERRIF = 0;
}

/**
 * Called on ECAN invalid message interrupt.
 */
#if defined(IVT_BASE_ADDRESS)
void __interrupt(irq(IRXIF), base(IVT_BASE_ADDRESS), low_priority) ECAN_IRXI_ISR(void) {
#else
void ECAN_IRXI_ISR(void) {
#endif

#ifdef INCLUDE_CBUS_CAN_STATS
    ++stats.busInvMsg;
#endif
    txCheckErrorsIsr();

    // Clear interrupt flag
    PIR5bits.IRXIF = 0;
}

#if defined(CPU_FAMILY_PIC18_K80)
/**
 * Service routine for ECAN interrupts.
 */
void ecanIsr() {

    if (PIR5bits.RXBnIF) ECAN_RXBnI_ISR();
    if (PIR5bits.TXBnIF) ECAN_TXBnI_ISR();
    if (PIR5bits.FIFOWMIF) ECAN_FIFOWMI_ISR();
    if (PIR5bits.ERRIF) ECAN_ERRI_ISR();
    if (PIR5bits.IRXIF) ECAN_IRXI_ISR();
}
#endif

/**
 * Performs regular ECAN operations (called every millisecond).
 */
void ecanTimerIsr() {

    // Loop through TX buffers
    for (uint8_t i = 0; i < 3; ++i) {
        ECANCONbits.EWIN = ewinTX + i;

        // Check for timeout
        if (ewin.TXCONbits.TXREQ) txCheckTimeoutIsr(txb0StartedMillis);

        // If TXB0 is idle, and FIFO not empty, start the next transmit
        if (i == 0 && !ewin.TXCONbits.TXREQ && TX_FIFO_CUR_LENGTH > 0) txb0NextIsr();
    }
}


// </editor-fold>


#endif  /* ECAN_BUFFERS_BASE_ADDRESS */
