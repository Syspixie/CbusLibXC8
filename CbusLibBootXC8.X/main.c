/**
 * @file CbusLibBootXC8 main.c
 * @copyright (C) 2022 Konrad Orlowski     <syspixie@gmail.com>
 * 
 *  CbusLibBootXC8 is licensed under the:
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
 * @author Konrad Orlowski
 * @date October 2022
 * 
 * Linking PIC18 Bootloaders & Applications:
 * https://www.microchip.com/stellent/groups/SiteComm_sg/documents/DeviceDoc/en558478.pdf
 */


#include "global.h"
#include "hardware.h"


// Uncomment if verification of configuration writes is required; verification
// may fail if configuration bits written are not all read/write
//#define VERIFY_CONFIG_WRITE


#define APPLICATION_BASE_ADDRESS 0x000800
#define EEPROM_BOOT_FLAG EEPROM_TOP

#if defined(CPU_FAMILY_PIC18_K80)
#define CONFIG_UPPER_BYTE 0x30
#define EEPROM_UPPER_BYTE 0xF0
#define NVM_READ_EEPROM 0b00000000
#define NVM_WRITE_EEPROM 0b00000100
#define NVM_WRITE_CONFIG 0b01000100
#define NVM_WRITE_FLASH_PAGE 0b10000100
#define NVM_ERASE_FLASH_PAGE 0b10010100
#define NVMADRL EEADR
#define NVMADRH EEADRH
#define NVMDATL EEDATA
#endif
#if defined(CPU_FAMILY_PIC18_K83)
#define CONFIG_UPPER_BYTE 0x30
#define EEPROM_UPPER_BYTE 0x31
#define NVM_READ_EEPROM 0b00000000
#define NVM_WRITE_EEPROM 0b00000100
#define NVM_WRITE_CONFIG 0b01000100
#define NVM_WRITE_FLASH_PAGE 0b10000100
#define NVM_ERASE_FLASH_PAGE 0b10010100
#define NVMDATL NVMDAT
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
#define CONFIG_UPPER_BYTE 0x30
#define EEPROM_UPPER_BYTE 0x38
#define NVM_READ_EEPROM 0b000
#define NVM_READ_EEPROM_POSTINC 0b001
#define NVM_WRITE_EEPROM 0b011
#define NVM_WRITE_EEPROM_POSTINC 0b100
#define NVM_READ_CONFIG 0b000
#define NVM_READ_CONFIG_POSTINC 0b001
#define NVM_WRITE_CONFIG 0b011
#define NVM_WRITE_CONFIG_POSTINC 0b100
#define NVM_READ_FLASH 0b000
#define NVM_READ_FLASH_POSTINC 0b001
#define NVM_READ_FLASH_PAGE 0b010
#define NVM_WRITE_FLASH 0b011
#define NVM_WRITE_FLASH_POSTINC 0b100
#define NVM_WRITE_FLASH_PAGE 0b101
#define NVM_ERASE_FLASH_PAGE 0b110
#endif


// Status flags
typedef union {
    uint8_t value;
    struct {
        unsigned verifyError : 1;       // Self-verification error
    };
} status_t;

// Control flags
typedef union {
    uint8_t value;
    struct {
        unsigned writeUnlock : 1;       // Allow write and erase operations to memory
        unsigned eraseOnly : 1;         // Erase Program Memory (must be on block boundary)
        unsigned autoErase : 1;         // Automatically erase Program Memory while writing data
        unsigned autoIncrement : 1;     // Automatically increment the pointer after writing
    };
} ctl_t;

// Commands
typedef enum {
    cmdReset = 1,
    cmdClearError = 2,
    cmdCheckError = 3,
    cmdBootTest = 4
} cmd_t;

// CAN bus received buffer definitions
typedef union {
    struct {
        bytes24_t memAddr;          // Next memory location to be programmed
        uint8_t spare;
        ctl_t ctl;                  // Control flags
        cmd_t cmd;                  // Command
        bytes16_t cmdData;          // Command data
    };
    uint8_t bytes[8];
} buff_t;

// CAN bus transmit responses
typedef enum {
    canSendNack = 0,                // NACK - operation failed
    canSendAck = 1,                 // ACK - operation succeeded
    canSendConfirmBoot = 2          // Confirm that bootloader is running
} canSend_t;


// Set the EEPROM_BOOT_FLAG to 0 (no bootloader)
// __EEPROM_DATA takes 8 bytes, so set a new more defaults as well
asm("PSECT eeprom_data,class=EEDATA");
asm("ORG " ___mkstr(EEPROM_BOOT_FLAG - 7));
__EEPROM_DATA(0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00);


#if defined(ECAN_BUFFERS_BASE_ADDRESS)
// ECAN_BUFFERS_BASE_ADDRESS is the address of RXB0, which is handy...
volatile buff_t rxBuff __at(ECAN_BUFFERS_BASE_ADDRESS + 6);  // RXB0 D0-D7
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
// CAN1_BUFFERS_BASE_ADDRESS is the address of TXQ; FIFO1 is next one up...
volatile uint8_t txFifoObj[16] __at(CAN1_BUFFERS_BASE_ADDRESS);
volatile uint8_t rxFifoObj[16] __at(CAN1_BUFFERS_BASE_ADDRESS + 16);
volatile buff_t rxBuff __at(CAN1_BUFFERS_BASE_ADDRESS + 24);
#endif

buff_t buff;                        // Copy of CAN data D0-D7
uint8_t buffLen;                    // # bytes sent

bytes16_t checksum;                 // Programmed data checksum
status_t status;                    // Status flags


/**
 * Performs a memory read operation.
 * 
 * @pre Appropriate address registers set.
 * @param nvmOp Operation to be performed.
 * @post Data in latch or holding registers.
 */
static void readMemory(uint8_t nvmOp) {

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up read
    EECON1 = nvmOp;

    // Initiate read
    EECON1bits.RD = 1;

    // Wait for read to complete
    NOP();
    NOP();
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up read
    NVMCON1 = nvmOp;

    // Initiate read
    NVMCON1bits.RD = 1;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up read
    NVMCON1bits.NVMCMD = nvmOp;

    // Initiate read
    NVMCON0bits.GO = 1;

    // Wait for read to complete
    while (NVMCON0bits.GO);
#endif
}

/**
 * Performs a memory write or erase operation.
 * 
 * @pre Appropriate address registers set.
 * @pre Data in latch or holding registers.
 * @param nvmOp Operation to be performed.
 * 
 * @note The routine waits for the completion of the operation, which can be
 * up to 11 milliseconds.
 */
static void writeMemory(uint8_t nvmOp) {

    // Only write if unlocked
    if (!buff.ctl.writeUnlock) return;

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up write
    EECON1 = nvmOp;

    // Perform write
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;

    // Wait for write to complete
    while (EECON1bits.WR);
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up write
    NVMCON1 = nvmOp;

    // Perform write
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;

    // Wait for write to complete
    while (NVMCON1bits.WR);
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up write
    NVMCON1bits.NVMCMD = nvmOp;

    // Perform write
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;

    // Wait for write to complete
    while (NVMCON0bits.GO);
#endif
}

/**
 * Erases a block of FLASH memory.
 * 
 * @pre Address in buff.memAddr; only erases when address on a block boundary.
 * 
 * @note Changes all the bytes to 0xFF, and is the only way of changing
 * FLASH bits from '0' to '1'.
 */
static void eraseFlash() {

    // Only erase on block boundary
    if (buff.memAddr.value & (FLASH_PAGE_SIZE - 1)) return;

    // Set the erase address
#if defined(CPU_FAMILY_PIC18_K80) | defined(CPU_FAMILY_PIC18_K83)
    TBLPTR = buff.memAddr.value;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMADR = buff.memAddr.value;
#endif

    // Erase FLASH
    writeMemory(NVM_ERASE_FLASH_PAGE);
}

/**
 * Writes 8 bytes of FLASH memory.
 * 
 * @pre Data in rxBuff.
 * @pre Data length in buffLen; only writes if 8 bytes.
 * @pre Address in buff.memAddr; only writes when address on 8-byte boundary.
 * 
 * @note This operation always writes a block from the holding registers to
 * FLASH.  Because the holding registers are set to 0xFF on reset and after
 * every write, and FLASH can only change '1's to '0's, unused holding registers
 * are effectively ignored and those FLASH locations are unchanged.
 * 
 * @note Each data byte is added to the checksum value.
 * @note Bytes written are read and verified.
 */
static void writeFlash() {

    // Only write if 8 bytes received, and writing on 8 byte boundary
    if (buffLen != 8) return;
    if (buff.memAddr.valueL & 0b00000111) return;

#if defined(CPU_FAMILY_PIC18_K80) | defined(CPU_FAMILY_PIC18_K83)
    // Set the write address
    TBLPTR = buff.memAddr.value;

    for (uint8_t i = 0; i < 8; ++i) {

        // Get byte, and add its value to the checksum
        uint8_t b = rxBuff.bytes[i];
        checksum.value += b;

        // Write byte to holding register
        TABLAT = b;
        asm("TBLWTPOSTINC");
    }

    // Reset the write address ready for verify read (do it now because
    // previous operation will have moved the address pointer outside the block)
    TBLPTR -= 8;

    // Perform write from holding registers to FLASH
    writeMemory(NVM_WRITE_FLASH_PAGE);
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set the write address
    NVMADR = buff.memAddr.value;

    // Working with words, not bytes: increment by 2
    for (uint8_t i = 0; i < 8; i += 2) {

        // Get byte, and add its value to the checksum
        uint8_t b = rxBuff.bytes[i];
        checksum.value += b;
        NVMDATL = b;

        // Ditto next byte
        b = rxBuff.bytes[i + 1];
        checksum.value += b;
        NVMDATH = b;

        // Write word to FLASH
        writeMemory(NVM_WRITE_FLASH_POSTINC);
    }

    // Get ready for verify read
    TBLPTR = buff.memAddr.value;
#endif

    for (uint8_t i = 0; i < 8; ++i) {

        // Read and verify byte
        asm("TBLRDPOSTINC");
        if (TABLAT != rxBuff.bytes[i]) status.verifyError = 1;
    }
}

/**
 * Writes up to 8 bytes of configuration memory.
 * 
 * @pre Data in rxBuff.
 * @pre Data length in buffLen.
 * @pre Address in buff.memAddr; upper byte must match CONFIG_UPPER_BYTE.
 * 
 * @note Each data byte is added to the checksum value.
 * @note Bytes written are read and optionally verified.
 */
static void writeConfig() {

#if defined(CPU_FAMILY_PIC18_K80) | defined(CPU_FAMILY_PIC18_K83)
    // Set the write address
    TBLPTR = buff.memAddr.value;

    for (uint8_t i = 0; i < buffLen; ++i) {

        // Get byte, and add its value to the checksum
        uint8_t b = rxBuff.bytes[i];
        checksum.value += b;

        // Write byte to holding register
        TABLAT = b;
        asm("TBLWT");

        // Perform write from holding registers to configuration
        writeMemory(NVM_WRITE_CONFIG);

        // Read and verify byte (read must always be done to increment address;
        // verify is optional because not all configuration bits are read/write)
        asm("TBLRDPOSTINC");
#ifdef VERIFY_CONFIG_WRITE
        if (TABLAT != b) status.verifyError = 1;
#endif //VERIFY_CONFIG_WRITE
    }
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set the write address
    NVMADR = buff.memAddr.value;

    for (uint8_t i = 0; i < buffLen; ++i) {

        // Get byte, and add its value to the checksum
        uint8_t b = rxBuff.bytes[i];
        checksum.value += b;
        NVMDATL = b;

        // Write byte to configuration
        writeMemory(NVM_WRITE_CONFIG);

        // Read and verify byte (read must always be done to increment address;
        // verify is optional because not all configuration bits are read/write)
        readMemory(NVM_READ_CONFIG_POSTINC);
#ifdef VERIFY_CONFIG_WRITE
        if (TABLAT != b) status.verifyError = 1;
#endif //VERIFY_CONFIG_WRITE
    }
#endif
}

/**
 * Writes up to 8 bytes of EEPROM memory.
 * 
 * @pre Data in rxBuff.
 * @pre Data length in buffLen.
 * @pre Address in buff.memAddr; upper byte must match EEPROM_UPPER_BYTE.
 * 
 * @note Each data byte is added to the checksum value.
 * @note Bytes written are read and verified.
 */
static void writeEeprom() {

    // Set the write address
    NVMADRL = buff.memAddr.valueL;
    NVMADRH = buff.memAddr.valueH;
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMADRU = buff.memAddr.valueU;
#endif

    for (uint8_t i = 0; i < buffLen; ++i) {

        // Get byte, and add its value to the checksum
        uint8_t b = rxBuff.bytes[i];
        checksum.value += b;
        NVMDATL = b;

        // Write byte to EEPROM
        writeMemory(NVM_WRITE_EEPROM);

        // Read and verify byte
        readMemory(NVM_READ_EEPROM);
        if (NVMDATL != b) status.verifyError = 1;

        // Move to next address
        if (++NVMADRL == 0) ++NVMADRH;
    }
}

/**
 * Transmits a single-byte response on the CAN bus.
 * 
 * @pre TXB0 set up with CAN ID.
 * @param data Byte to be transmitted.
 */
static void sendResponse(canSend_t data) {

#if defined(ECAN_BUFFERS_BASE_ADDRESS)
    // Wait for any previous transmission to complete
    while (TXB0CONbits.TXREQ);

    // Set up data to send
    TXB0DLC = 1;
    TXB0D0 = data;

    // Start transmission
    TXB0CONbits.TXREQ = 1;
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
    // Wait for any previous transmission to complete
    while (C1TXQCONHbits.TXREQ);

    // Set up data to send
    txFifoObj[4] = 0b00010001;  // RTR=0; EID=1; DLC=1
    txFifoObj[8] = data;

    // Start transmission
    C1TXQCONHbits.UINC = 1;
    C1TXQCONHbits.TXREQ = 1;
#endif
}

/**
 * Clears the bootloader flag then perform a software device reset.
 */
static void startApplication() {

    LATB = 0x00;            // LEDs off

    // Set up address of bootloader flag in EEPROM
    NVMADRL = EEPROM_BOOT_FLAG & 0xFF;
    NVMADRH = EEPROM_BOOT_FLAG >> 8;
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMADRU = EEPROM_UPPER_BYTE;
#endif

    // Clear bootloader flag
    NVMDATL = 0x00;
    writeMemory(NVM_WRITE_EEPROM);

    RESET();    // MCU reset
}

/**
 * Bootloader entry point.
 */
void main() __at(0x0020) {

    // Set up address of bootloader flag
    NVMADRL = EEPROM_BOOT_FLAG & 0xFF;
    NVMADRH = EEPROM_BOOT_FLAG >> 8;
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMADRU = EEPROM_UPPER_BYTE;
#endif

    // If bootloader flag is not set jump to application start
    readMemory(NVM_READ_EEPROM);
    if (NVMDATL != 0xFF) {
        asm("GOTO " ___mkstr(APPLICATION_BASE_ADDRESS));
    }

//********* Initialise pins

#if defined(CPU_FAMILY_PIC18_K83)
    RB2PPS = 0x33;      //RB2->ECAN:CANTX0;    
    CANRXPPS = 0x0B;    //RB3->ECAN:CANRX;    
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    RB2PPS = 0x46;      //RB2->CAN1:CANTX;
    CANRXPPS = 0x0B;    //RB3->CAN1:CANRX;
#endif

    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;

#if defined(CPU_FAMILY_PIC18_K80)
    TRISA = 0b11101111;     // A7:OSC1 A6:OSC2 A4:VCAP A2:BtnProg
    TRISB = 0b00111011;     // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX
    TRISC = 0b11111111;

    ANCON0 = 0x00;
    ANCON1 = 0x00;

    WPUB = 0b00110011;      // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX
    INTCON2bits.nRBPU = 0;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    TRISA = 0b11111111;     // A7:OSC1 A6:OSC2 A2:BtnProg
    TRISB = 0b00111011;     // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX0
    TRISC = 0b11111111;

    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    WPUA = 0b00111011;      // A7:OSC1 A6:OSC2 A2:BtnProg
    WPUB = 0b00110011;      // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX
    WPUC = 0b11111111;
    WPUE = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    TRISA = 0b11111111;     // A7:OSC1 A6:OSC2 A2:BtnProg
    TRISB = 0b00111011;     // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX0
    TRISC = 0b11111111;
    TRISE = 0b00001000;

    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    WPUA = 0b00111011;      // A7:OSC1 A6:OSC2 A2:BtnProg
    WPUB = 0b00110011;      // B7:LedSlim B6:LedFlim B3:CANRX B2:CANTX
    WPUC = 0b11111111;
    WPUE = 0b00000000;
#endif

    LATB = 0b11000000;      // RB7 & RB6 set high (LEDs on)

//********* Initialise CAN bus interface

#if defined(ECAN_BUFFERS_BASE_ADDRESS)
    CANCON = 0x80;
    while ((CANSTAT & 0xE0) != 0x80); // wait until ECAN is in config mode

#if defined(CPU_FAMILY_PIC18_K80)
    CIOCON = 0b00100000;    // TX drives Vdd when recessive
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    CIOCON = 0b00000000;
#endif

    // Receive masks
    RXM0EIDH = 0b11111111;
    RXM0EIDL = 0b11111000;
    RXM0SIDH = 0b11111111;
    RXM0SIDL = 0b11101011;

    // Receive filters
    RXF0EIDH = 0b00000000;
    RXF0EIDL = 0b00000111;
    RXF0SIDH = 0b00000000;
    RXF0SIDL = 0b00001000;

    BRGCON1 = 0b00001111;   // Sync 1xTQ; Baud rate 125kbps
    BRGCON2 = 0b10011110;   // Segment 1 4xTQ; propagation 7xTQ
    BRGCON3 = 0b00000011;   // Segment 2 4xTQ

    CANCON = 0x00;
    while ((CANSTAT & 0xE0) != 0x00); // wait until ECAN is in Normal mode

    // Transmit buffer ID
    TXB0SIDL = 0b00001000;
    TXB0SIDH = 0b10000000;
    TXB0EIDL = 0b00000100;
    TXB0EIDH = 0b00000000;
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
    /* Enable the CAN module */
    C1CONHbits.ON = 1;

    C1CONTbits.REQOP = 0b100;
    while (C1CONUbits.OPMOD != 0b100);  // Wait for Configuration mode

    /* Initialize the C1FIFOBA with the start address of the CAN FIFO message object area. */
    C1FIFOBA = CAN1_BUFFERS_BASE_ADDRESS;

    C1CONL = 0x60;      // CLKSEL0 disabled; PXEDIS enabled; ISOCRCEN enabled; DNCNT 0;
    C1CONH = 0x97;      // ON enabled; FRZ disabled; SIDL disabled; BRSDIS enabled; WFT T11 Filter; WAKFIL enabled;
    C1CONU = 0x10;      // TXQEN enabled; STEF disabled; SERR2LOM disabled; ESIGM disabled; RTXAT disabled;

    C1NBTCFGL = 0x02;   // SJW 2;
    C1NBTCFGH = 0x02;   // TSEG2 2;
    C1NBTCFGU = 0x03;   // TSEG1 3;
    C1NBTCFGT = 0x3F;   // BRP 63;

    C1TXQCONL = 0x00;   // TXATIE disabled; TXQEIE disabled; TXQNIE disabled;
    C1TXQCONH = 0x04;   // FRESET enabled; UINC disabled;
    C1TXQCONU = 0x60;   // TXAT 3; TXPRI 1;
    C1TXQCONT = 0x00;   // PLSIZE 8; FSIZE 1;

    C1FIFOCON1L = 0x00; // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE disabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE disabled;
    C1FIFOCON1H = 0x04; // FRESET enabled; TXREQ disabled; UINC disabled;
    C1FIFOCON1U = 0x20; // TXAT Three retransmission attempts; TXPRI 1;
    C1FIFOCON1T = 0x00; // PLSIZE 8; FSIZE 1;

    C1FLTOBJ0L = 0b00000000;
    C1FLTOBJ0H = 0b00111000;
    C1FLTOBJ0U = 0b00000000;
    C1FLTOBJ0T = 0b01000000;    // EXIDE set: allow extended ID only
    C1MASK0L = 0b11111111;
    C1MASK0H = 0b11000111;
    C1MASK0U = 0b11111111;
    C1MASK0T = 0b01011111;      // MIDE set: filter on EXIDE
    C1FLTCON0L = 0x81;  // FLTEN0 enabled; F0BP FIFO 1; 

    C1INTL = 0x00;      // MODIF disabled; TBCIF disabled;
    C1INTH = 0x00;      // IVMIF disabled; WAKIF disabled; CERRIF disabled; SERRIF disabled;
    C1INTU = 0x08;      // TEFIE disabled; MODIE enabled; TBCIE disabled; RXIE disabled; TXIE disabled;
    C1INTT = 0xFC;      // IVMIE enabled; WAKIE enabled; CERRIE enabled; SERRIE enabled; RXOVIE enabled; TXATIE enabled;

    C1CONTbits.REQOP = 0b110;
    while (C1CONUbits.OPMOD != 0b110);  // Wait for Normal 2.0 mode

    // Transmit FIFO ID
    txFifoObj[0] = 0b00000000;
    txFifoObj[1] = 0b00100100;
    txFifoObj[2] = 0b00000000;
    txFifoObj[3] = 0b00000000;
#endif

//********* Initialise local variables

    checksum.value = 0;
    status.value = 0;
    buff.ctl.value = 0;

//********* Main loop

    while (true) {

        bool validMsg = true;

#if defined(ECAN_BUFFERS_BASE_ADDRESS)
        // Wait for message
        while (!RXB0CONbits.RXFUL);

        // Check RXRTR || !EXID
        if (RXB0DLCbits.RXRTR || !RXB0SIDLbits.EXID) validMsg = false;

        // Data count
        buffLen = RXB0DLCbits.DLC;
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
        // Wait for message
        while (!C1FIFOSTA1Lbits.TFNRFNIF);

        // Check FDF || BRS || RTR || !EID
        if ((rxFifoObj[4] ^ 0b00010000) & 0b11110000) validMsg = false;

        // Data count
        buffLen = rxFifoObj[4] & 0b00001111;
#endif
        if (buffLen > 8 || buffLen == 0) validMsg = false;

        if (!validMsg) {
            // Ignore message

        // Lowest bit of CAN ID determines whether PUT data or command sent
#if defined(ECAN_BUFFERS_BASE_ADDRESS)
        } else if (RXB0EIDLbits.EID0) {
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
        } else if (rxFifoObj[1] & 0b00001000) {
#endif

            //********* Received PUT data

            // Upper address byte determines memory type
            uint8_t m = buff.memAddr.valueU;
            if (m < CONFIG_UPPER_BYTE) {

                // Erase and write to FLASH
                if (buff.ctl.autoErase) eraseFlash();
                if (!buff.ctl.eraseOnly) writeFlash();

            } else if (m == CONFIG_UPPER_BYTE) {

                // Write to configuration values
                writeConfig();

            } else if (m == EEPROM_UPPER_BYTE) {

                // Write to EEPROM
                writeEeprom();

            }

            // Perform address auto increment
            if (buff.ctl.autoIncrement) buff.memAddr.value += buffLen;

        } else if (buffLen >= 6) {

            //********* Received command

            // Take a copy of the received data
            buff = rxBuff;

            // Execute command (unrecognised values treated as NOOP)
            switch (buff.cmd) {

                // Stop bootloader and start application
                case cmdReset: {
                    startApplication();
                } break;

                // Clear checksum value and verify error flag
                case cmdClearError: {
                    checksum.value = 0;
                    status.value = 0;
                } break;

                // Check checksum value and verify error flag
                case cmdCheckError: {
                    if (status.verifyError
                            || (checksum.value + buff.cmdData.value) != 0) {
                        sendResponse(canSendNack);
                    } else {
                        sendResponse(canSendAck);
                    }
                } break;

                // Send 'hello' message - confirms bootloader is running
                case cmdBootTest: {
                    sendResponse(canSendConfirmBoot);
                } break;
            }
        }

#if defined(ECAN_BUFFERS_BASE_ADDRESS)
        // Buffer ready to receive
        RXB0CONbits.RXFUL = 0;
#endif
#if defined(CAN1_BUFFERS_BASE_ADDRESS)
        // Buffer ready to receive
        C1FIFOCON1Hbits.UINC = 1;
        C1FIFOSTA1Lbits.RXOVIF = 0;
#endif
    }
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


#if defined(IVT_BASE_ADDRESS)
/**
 * Relocation of the IVT (interrupt vector table) is left up to the application.
 */
#else
/**
 * Relocation of the application's base address from 0x00000 to 0x000800 (making
 * room for the bootloader) means that its high and low priority interrupt
 * handler routines are located at 0x000808 and 0x000818 respectively; but the
 * hardware still uses the default 0x000008 and 0x000018 locations.  The
 * bootloader does not use interrupts, so just provides jumps from the default
 * to the relocated handler addresses.
 */

// Jump to relocated high priority interrupt location in the application
asm("PSECT intcode");
asm("GOTO " ___mkstr(APPLICATION_BASE_ADDRESS + 0x08));

// Jump to relocated low priority interrupt location in the application
asm("PSECT intcodelo");
asm("GOTO " ___mkstr(APPLICATION_BASE_ADDRESS + 0x18));
#endif


// </editor-fold>
