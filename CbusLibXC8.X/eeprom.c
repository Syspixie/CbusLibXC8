/**
 * @file CbusLibXC8 eeprom.c
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
 * Code to read and write EEPROM memory.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 */


#include "eeprom.h"
#include "module.h"


#if defined(CPU_FAMILY_PIC18_K80)
#define NVMADRL EEADR
#define NVMADRH EEADRH
#endif


/**
 * Reads from the current EEPROM location.
 * 
 * @pre NVMADRL & NVMADRH set.
 * @return Byte read.
 */
static uint8_t readEE() {

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up read
    EECON1 = 0b00000000;

    // Initiate read
    EECON1bits.RD = 1;

    // Wait for read to complete
    NOP();
    NOP();
    
    return EEDATA;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up read
    NVMCON1 = 0b00000000;

    // Initiate read
    NVMCON1bits.RD = 1;
    
    return NVMDAT;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up read
    NVMADRU = 0x38;
    NVMCON1bits.NVMCMD = 0b000;

    // Initiate read
    NVMCON0bits.GO = 1;
    
    return NVMDATL;
#endif
}

/**
 * Writes to the current EEPROM location.
 * 
 * Uses stallMemoryWrite callback as the code goes into a blocking loop
 * (interrupts enabled) until the write is complete (typically 4ms).
 * 
 * @pre NVMADRL & NVMADRH set.
 * @param data Byte to be written.
 */
static void writeEE(uint8_t data) {

    while (stallMemoryWrite());

#if defined(CPU_FAMILY_PIC18_K80)
    EEDATA = data;

    // Set up write
    EECON1 = 0b00000100;

    // Initiate write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Wait for write to complete
    while (EECON1bits.WR);      // Blocking loop

    // Avoid accidental writes
    EECON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    NVMDAT = data;

    // Set up write
    NVMCON1 = 0b00000100;

    // Initiate write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Wait for write to complete
    while (NVMCON1bits.WR);     // Blocking loop

    // Avoid accidental writes
    NVMCON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMDATL = data;

    // Set up write
    NVMADRU = 0x38;
    NVMCON1bits.NVMCMD = 0b011;

    // Initiate write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    INTERRUPT_EnableHigh();

    // Wait for write to complete
    while (NVMCON0bits.GO);     // Blocking loop

    // Avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;
#endif
}

/**
 * Initialises the EEPROM interface.
 */
void initEeprom() {
    
    // Delay required otherwise first reads & writes get screwed up.
    // Can't use millis timer because interrupts are not enabled yet.
    __delay_ms(20);
}

/**
 * Reads a byte from EEPROM.
 * 
 * @param addr EEPROM address to be read.
 * @return Byte read.
 */
uint8_t readEeprom8(uint16_t addr) {

    NVMADRL = ((bytes16_t) addr).valueL;
    NVMADRH = ((bytes16_t) addr).valueH;

    return readEE();
}

/**
 * Reads a two-byte word from EEPROM.
 * 
 * @param addr EEPROM address to be read.
 * @return Word read.
 */
uint16_t readEeprom16(uint16_t addr) {

    NVMADRL = ((bytes16_t) addr).valueL;
    NVMADRH = ((bytes16_t) addr).valueH;

    bytes16_t v;
    v.bytes[0] = readEE();
    if (++NVMADRL == 0) ++NVMADRH;
    v.bytes[1] = readEE();
    return v.value;
}

/**
 * Writes a byte to EEPROM.
 * 
 * @param addr EEPROM address to be written.
 * @param data Byte to write.
 */
void writeEeprom8(uint16_t addr, uint8_t data) {

    NVMADRL = ((bytes16_t) addr).valueL;
    NVMADRH = ((bytes16_t) addr).valueH;

    writeEE(data);
}

/**
 * Writes a two-byte word to EEPROM.
 * 
 * @param addr EEPROM address to be written.
 * @param data Word to write.
 */
void writeEeprom16(uint16_t addr, uint16_t data) {

    NVMADRL = ((bytes16_t) addr).valueL;
    NVMADRH = ((bytes16_t) addr).valueH;

    writeEE(((bytes16_t) data).bytes[0]);
    if (++NVMADRL == 0) ++NVMADRH;
    writeEE(((bytes16_t) data).bytes[1]);
}
