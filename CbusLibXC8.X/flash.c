/**
 * @file CbusLibXC8 flash.c
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
 * Reads FLASH memory directly, or reads/writes FLASH using a RAM cache.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 * 
 * @note To avoid using 24-bit address pointers on an MCU which has only 64K
 * memory, the address data type is flashAddr_t, which is set to uint16_t or
 * uint24_t as appropriate (see flash.h and hardware.h).
 */


#include "flash.h"
#include "module.h"


typedef union {
    struct {
        unsigned isLoaded : 1;
        unsigned isModified : 1;
        unsigned requiresErase : 1;
    };
    uint8_t flags;
} cacheStatus_t;


uint8_t cacheData[FLASH_BLOCK_SIZE];        // Cache
flashAddr_t cachedBlock;                    // Currently cached memory block (on FLASH_BLOCK_SIZE boundary)
flashBlock_t cacheOffset;                   // Current byte offset within cachedBlock
cacheStatus_t cacheStatus;                  // Cache status flags


/**
 * Initialises the FLASH interface.
 */
void initFlash(void) {

    cacheStatus.flags = 0;
}


/* Non-cached reads ********************************************************* */


/**
 * Reads a byte from FLASH.
 * 
 * @param addr FLASH address to be read.
 * @return Byte read.
 */
uint8_t readFlash8(flashAddr_t addr) {

    TBLPTR = addr;
    asm("TBLRD");
    return TABLAT;
}

/**
 * Reads a two-byte word from FLASH.
 * 
 * @param addr FLASH address to be read.
 * @return Word read.
 */
uint16_t readFlash16(flashAddr_t addr) {

    TBLPTR = addr;
    bytes16_t v;
    asm("TBLRDPOSTINC");
    v.bytes[0] = TABLAT;
    asm("TBLRD");
    v.bytes[1] = TABLAT;
    return v.value;
}

/**
 * Reads a buffer from FLASH.
 * 
 * @param addr FLASH address to be read.
 * @param dst Pointer to buffer.
 * @param n Number of bytes to be read.
 */
void readFlash(flashAddr_t addr, void* dst, size_t n) {

    TBLPTR = addr;
    uint8_t* p = dst;
    while (n > 1) {             // Read all but the last byte
        asm("TBLRDPOSTINC");
        *p++ = TABLAT;
        --n;
    }
    if (n > 0) {
        asm("TBLRD");           // Last byte
        *p = TABLAT;
    }
}


/* Cached reads/writes ****************************************************** */


/**
 * Erases the current FLASH block.
 * 
 * Uses stallMemoryWrite callback as all code execution (including interrupts)
 * is stopped until the erase is complete (typically 2ms).
 * 
 * @pre cachedBlock set.
 * @pre cacheStatus set.
 * @post cacheStatus updated.
 */
static void eraseCache() {

    while (stallMemoryWrite());

    TBLPTR = cachedBlock;

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up erase
    EECON1 = 0b10010100;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTERRUPTbits_GIEH = 1;

    // Avoid accidental writes
    EECON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up erase
    NVMCON1 = 0b10010100;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    INTERRUPTbits_GIEH = 1;

    // Avoid accidental writes
    NVMCON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up erase
    NVMCON1bits.NVMCMD = 0b110;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    INTERRUPTbits_GIEH = 1;

    // Wait for erase to complete
    while (NVMCON0bits.GO);

    // Avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;
#endif

    // Update cache status
    cacheStatus.requiresErase = 0;
}

/**
 * Write all cache data back to the current FLASH block.
 * 
 * If any modification of the cache has involved a bit going from 0 to 1, then
 * the FLASH block is erased before writing (the write process can only change a
 * bit from 1 to 0). Uses stallMemoryWrite callback as all code execution
 * (including interrupts) is stopped until the write is complete (typically
 * 2ms).  There will be two such stoppages if an erase is also required.
 * 
 * @pre cachedBlock set.
 * @pre cacheStatus set.
 * @post cacheStatus updated.
 */
static void flushCache() {

    // Perform erase if required
    if (cacheStatus.requiresErase) eraseCache();

    while (stallMemoryWrite());

    TBLPTR = cachedBlock;

    // Populate the internal write buffer with the contents of the cache
    uint8_t* p = cacheData;
    flashBlock_t n = FLASH_BLOCK_SIZE;
    while (n > 1) {             // Write all but the last byte
        TABLAT = *p++;
        asm("TBLWTPOSTINC");
        --n;
    }
    TABLAT = *p;                // Last byte
    asm("TBLWT");

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up write
    EECON1 = 0b10000100;

    // Perform write with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTERRUPTbits_GIEH = 1;

    // Avoid accidental writes
    EECON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up write
    NVMCON1 = 0b10000100;

    // Perform write with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    INTERRUPTbits_GIEH = 1;

    // Avoid accidental writes
    NVMCON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up write
    NVMCON1bits.NVMCMD = 0b101;

    // Perform write with all interrupts temporarily disabled
    INTERRUPTbits_GIEH = 0;
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    INTERRUPTbits_GIEH = 1;

    // Wait for write to complete
    while (NVMCON0bits.GO);

    // Avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;
#endif

    // Update cache status
    cacheStatus.isModified = 0;
}

/**
 * Ensures that the correct cache block is loaded into RAM from FLASH for the
 * given address.
 * 
 * If an existing cache block is loaded, it is flushed back to FLASH.
 * 
 * @pre cacheStatus set.
 * @param FLASH address to be loaded.
 * @post cacheBlock set.
 * @post cacheOffset set.
 * @post cacheStatus updated.
 */
static void loadBlock(flashAddr_t addr) {

    // Find the base (boundary) value of the given address
    flashAddr_t requiredBlock = addr & (flashAddr_t) ~(FLASH_BLOCK_SIZE - 1);

    // If a different cache block is loaded, flush it
    if (cacheStatus.isLoaded && cachedBlock != requiredBlock) {
        if (cacheStatus.isModified) flushCache();
        cacheStatus.flags = 0;
    }

    // If the required cache block is not loaded, load it
    if (!cacheStatus.isLoaded) {
        cachedBlock = requiredBlock;
        readFlash(cachedBlock, cacheData, FLASH_BLOCK_SIZE);
        cacheStatus.isLoaded = 1;
    }

    // Set the cache block offset (the next byte to be read/written)
    cacheOffset = (flashBlock_t) (addr & (FLASH_BLOCK_SIZE - 1));
}

/**
 * Loads the next cache block from FLASH.
 * 
 * @pre cacheBlock set.
 * @pre cacheStatus set.
 * @post cacheBlock updated.
 * @post cacheOffset set.
 * @post cacheStatus updated.
 */
static void loadNextBlock() {

    // Flush the current cache block
    if (cacheStatus.isModified) flushCache();
    cacheStatus.flags = 0;

    // Read the next cache block from FLASH
    cachedBlock += FLASH_BLOCK_SIZE;
    readFlash(cachedBlock, cacheData, FLASH_BLOCK_SIZE);
    cacheStatus.isLoaded = 1;

    // Set the cache block offset (the next byte to be read/written)
    cacheOffset = 0;
}

/**
 * Reads a byte from the current cache location.
 * 
 * @pre cacheBlock set.
 * @pre cacheOffset set.
 * @pre cacheStatus set.
 * @post cacheBlock possibly updated.
 * @post cacheOffset updated.
 * @post cacheStatus updated.
 */
static uint8_t readCache() {

    // If the last operation made cascheOffset cross the cache block boundary,
    // load the next block
    if (cacheOffset >= FLASH_BLOCK_SIZE) loadNextBlock();

    // Read a byte, and increment cacheOffset
    return cacheData[cacheOffset++];
}

/**
 * Writes a byte to the current cache location.
 * 
 * @pre cacheBlock set.
 * @pre cacheOffset set.
 * @pre cacheStatus set.
 * @post cacheBlock possibly updated.
 * @post cacheOffset updated.
 * @post cacheStatus updated.
 */
static void writeCache(uint8_t data) {
    
    // If the last operation made cascheOffset cross the cache block boundary,
    // load the next block
    if (cacheOffset >= FLASH_BLOCK_SIZE) loadNextBlock();

    // Write a byte if its value has changed; if any 0s become 1s, mark block
    // for erase before write to FLASH
    uint8_t curData = cacheData[cacheOffset];
    if (data != curData) {
        if (data & ~curData) cacheStatus.requiresErase = 1;
        cacheData[cacheOffset] = data;
        cacheStatus.isModified = 1;
    }

    // Increment cacheOffset
    cacheOffset++;
}

/**
 * Reads a byte from FLASH via the cache.
 * 
 * @param addr FLASH address to be read.
 * @return Byte read.
 */
uint8_t readFlashCached8(flashAddr_t addr) {

    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Read byte
    return readCache();
}

/**
 * Reads a two-byte word from FLASH via the cache.
 * 
 * @param addr FLASH address to be read.
 * @return Word read.
 */
uint16_t readFlashCached16(flashAddr_t addr) {

    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Read word
    bytes16_t v;
    v.bytes[0] = readCache();
    v.bytes[1] = readCache();
    return v.value;
}

/**
 * Reads a buffer from FLASH via the cache.
 * 
 * @param addr FLASH address to be read.
 * @param dst Pointer to buffer.
 * @param n Number of bytes to be read.
 */
void readFlashCached(flashAddr_t addr, void* dst, size_t n) {

    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Read buffer
    uint8_t* p = dst;
    while (n > 0) {
        *p++ = readCache();
        --n;
    }
}

/**
 * Writes a byte to FLASH via the cache.
 * 
 * @param addr FLASH address to be written.
 * @param data Byte to write.
 */
void writeFlashCached8(flashAddr_t addr, uint8_t data) {

    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Write byte
    writeCache(data);
}

/**
 * Writes a two-byte word to FLASH via the cache.
 * 
 * @param addr FLASH address to be written.
 * @param data Word to write.
 */
void writeFlashCached16(flashAddr_t addr, uint16_t data) {

    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Write word
    writeCache(((bytes16_t) data).bytes[0]);
    writeCache(((bytes16_t) data).bytes[1]);
}

/*
 * Writes a buffer to FLASH via the cache.
 * 
 * @param addr FLASH address to be written.
 * @param src Pointer to buffer.
 * @param n Number of bytes to be written.
 */
void writeFlashCached(flashAddr_t addr, void* src, size_t n) {
    
    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Write buffer
    uint8_t* p = src;
    while (n > 0) {
        writeCache(*p++);
        --n;
    }
}

/**
 * Writes a value to a range of bytes in FLASH via the cache.
 * 
 * @param addr FLASH address to be written.
 * @param data Value to be written to all bytes.
 * @param n Number of bytes to be written.
 */
void fillFlashCached(flashAddr_t addr, uint8_t data, size_t n) {
    
    // Make sure correct block is loaded into cache
    loadBlock(addr);

    // Write value to each byte
    while (n > 0) {
        writeCache(data);
        --n;
    }
}

/**
 * Called to force a write-back of the cache to FLASH.
 */
void flushFlashCache() {

    // Write cache back to FLASH if modified
    if (cacheStatus.isModified) flushCache();
}
