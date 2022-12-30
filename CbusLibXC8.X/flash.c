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


#if defined(FLASH_PAGE_BUFFER_ADDRESS)
// Use the designated page buffer as the cache
uint8_t pageBuffer[FLASH_PAGE_SIZE] __at(FLASH_PAGE_BUFFER_ADDRESS);
#else
// Cache
uint8_t pageBuffer[FLASH_PAGE_SIZE];
#endif

flashAddr_t currentPage;        // Currently cached memory page (on FLASH_PAGE_SIZE boundary)
flashPageOffset_t pageOffset;   // Current byte offset within currentPage
cacheStatus_t cacheStatus;      // Cache status flags


/**
 * Initialises the FLASH interface.
 */
void initFlash() {

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
 * Reads the current FLASH page into the page buffer.
 * 
 * @pre currentPage set.
 */
static void readPage() {

#if defined(CPU_FAMILY_PIC18_K80)
    TBLPTR = currentPage;
    uint8_t* p = pageBuffer;
    flashPageOffset_t n = FLASH_PAGE_SIZE;

    // Populate the page buffer with the contents of the FLASH page
    while (n > 1) {             // Read all but the last byte
        asm("TBLRDPOSTINC");
        *p++ = TABLAT;
        --n;
    }
    if (n > 0) {
        asm("TBLRD");           // Last byte
        *p = TABLAT;
    }
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    TBLPTR = currentPage;
    uint8_t* p = pageBuffer;
    flashPageOffset_t n = FLASH_PAGE_SIZE;

    // Populate the page buffer with the contents of the FLASH page
    while (n > 1) {             // Read all but the last byte
        asm("TBLRDPOSTINC");
        *p++ = TABLAT;
        --n;
    }
    if (n > 0) {
        asm("TBLRD");           // Last byte
        *p = TABLAT;
    }
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    NVMADR = currentPage;
    NVMCON1bits.NVMCMD = 0b010;

    // Perform read
    NVMCON0bits.GO = 1;

    // Wait for read to complete
    while (NVMCON0bits.GO);
#endif
}

/**
 * Writes the page buffer to the current FLASH page.
 * 
 * @pre currentPage set.
 */
static void writePage() {

#if defined(CPU_FAMILY_PIC18_K80)
    TBLPTR = currentPage;
    uint8_t* p = pageBuffer;
    flashPageOffset_t n = FLASH_PAGE_SIZE;

    // Populate the holding registers with the contents of the page buffer
    while (n > 1) {             // Write all but the last byte
        TABLAT = *p++;
        asm("TBLWTPOSTINC");
        --n;
    }
    TABLAT = *p;                // Last byte
    asm("TBLWT");

    // Set up write
    EECON1 = 0b10000100;

    // Perform write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Avoid accidental writes
    EECON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    TBLPTR = currentPage;
    uint8_t* p = pageBuffer;
    flashPageOffset_t n = FLASH_PAGE_SIZE;

    // Populate the holding registers with the contents of the page buffer
    while (n > 1) {             // Write all but the last byte
        TABLAT = *p++;
        asm("TBLWTPOSTINC");
        --n;
    }
    TABLAT = *p;                // Last byte
    asm("TBLWT");

    // Set up write
    NVMCON1 = 0b10000100;

    // Perform write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Avoid accidental writes
    NVMCON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up write
    NVMADR = currentPage;
    NVMCON1bits.NVMCMD = 0b101;

    // Perform write with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    INTERRUPT_EnableHigh();

    // Wait for write to complete
    while (NVMCON0bits.GO);

    // Avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;
#endif
}

/**
 * Erases the current FLASH page.
 * 
 * @pre currentPage set.
 */
static void erasePage() {

#if defined(CPU_FAMILY_PIC18_K80)
    // Set up erase
    TBLPTR = currentPage;
    EECON1 = 0b10010100;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Avoid accidental writes
    EECON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    // Set up erase
    TBLPTR = currentPage;
    NVMCON1 = 0b10010100;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    INTERRUPT_EnableHigh();

    // Avoid accidental writes
    NVMCON1 = 0b00000000;
#endif
#if defined(CPU_FAMILY_PIC18_Q83Q84)
    // Set up erase
    NVMADR = currentPage;
    NVMCON1bits.NVMCMD = 0b110;

    // Perform erase with all interrupts temporarily disabled
    INTERRUPT_DisableHigh();
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    INTERRUPT_EnableHigh();

    // Wait for erase to complete
    while (NVMCON0bits.GO);

    // Avoid accidental writes
    NVMCON1bits.NVMCMD = 0b000;
#endif
}

/**
 * Write all cache data back to the current FLASH page.
 * 
 * If any modification of the cache has involved a bit going from 0 to 1, then
 * the FLASH page is erased before writing (the write process can only change a
 * bit from 1 to 0). Uses stallMemoryWrite callback as all code execution
 * (including interrupts) is stopped until the write is complete (typically
 * 2ms).  There will be two such stoppages if an erase is also required.
 * 
 * @pre currentPage set.
 * @pre cacheStatus set.
 * @post cacheStatus updated.
 */
static void flushCache() {

    // Perform erase if required
    if (cacheStatus.requiresErase) {
        while (stallMemoryWrite());
        erasePage();
        cacheStatus.requiresErase = 0;
    }

    // Perform write
    while (stallMemoryWrite());
    writePage();
    cacheStatus.isModified = 0;
}

/**
 * Ensures that the correct cache page is loaded into the page buffer from
 * FLASH for the given address.
 * 
 * If an existing cache page is loaded, it is flushed back to FLASH.
 * 
 * @pre cacheStatus set.
 * @param FLASH address to be loaded.
 * @post currentPage and pageOffset set.
 * @post cacheStatus updated.
 */
static void loadPage(flashAddr_t addr) {

    // Find the base (boundary) value of the given address
    flashAddr_t requiredPage = addr & (flashAddr_t) ~(FLASH_PAGE_SIZE - 1);

    // If a different cache page is loaded, flush it
    if (cacheStatus.isLoaded && currentPage != requiredPage) {
        if (cacheStatus.isModified) flushCache();
        cacheStatus.flags = 0;
    }

    // If the required cache page is not loaded, load it
    if (!cacheStatus.isLoaded) {
        currentPage = requiredPage;
        readPage();
        cacheStatus.isLoaded = 1;
    }

    // Set the cache page offset (the next byte to be read/written)
    pageOffset = (flashPageOffset_t) (addr & (FLASH_PAGE_SIZE - 1));
}

/**
 * Loads the next cache page from FLASH.
 * 
 * @pre currentPage set.
 * @pre cacheStatus set.
 * @post currentPage and pageOffset updated.
 * @post cacheStatus updated.
 */
static void loadNextPage() {

    // Flush the current cache page
    if (cacheStatus.isModified) flushCache();
    cacheStatus.flags = 0;

    // Read the next cache page from FLASH
    currentPage += FLASH_PAGE_SIZE;
    readPage();
    cacheStatus.isLoaded = 1;

    // Set the cache page offset (the next byte to be read/written)
    pageOffset = 0;
}

/**
 * Reads a byte from the current cache location.
 * 
 * @pre currentPage and pageOffset set.
 * @pre cacheStatus set.
 * @post currentPage and pageOffset updated.
 * @post cacheStatus updated.
 */
static uint8_t readCache() {

    // If the last operation made cascheOffset cross the cache page boundary,
    // load the next page
    if (pageOffset >= FLASH_PAGE_SIZE) loadNextPage();

    // Read a byte, and increment pageOffset
    return pageBuffer[pageOffset++];
}

/**
 * Writes a byte to the current cache location.
 * 
 * @pre currentPage and pageOffset set.
 * @pre cacheStatus set.
 * @post currentPage and pageOffset updated.
 * @post cacheStatus updated.
 */
static void writeCache(uint8_t data) {
    
    // If the last operation made cascheOffset cross the cache page boundary,
    // load the next page
    if (pageOffset >= FLASH_PAGE_SIZE) loadNextPage();

    // Write a byte if its value has changed; if any 0s become 1s, mark page
    // for erase before write to FLASH
    uint8_t curData = pageBuffer[pageOffset];
    if (data != curData) {
        if (data & ~curData) cacheStatus.requiresErase = 1;
        pageBuffer[pageOffset] = data;
        cacheStatus.isModified = 1;
    }

    // Increment pageOffset
    pageOffset++;
}

/**
 * Reads a byte from FLASH via the cache.
 * 
 * @param addr FLASH address to be read.
 * @return Byte read.
 */
uint8_t readFlashCached8(flashAddr_t addr) {

    // Make sure correct page is loaded into cache
    loadPage(addr);

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

    // Make sure correct page is loaded into cache
    loadPage(addr);

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

    // Make sure correct page is loaded into cache
    loadPage(addr);

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

    // Make sure correct page is loaded into cache
    loadPage(addr);

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

    // Make sure correct page is loaded into cache
    loadPage(addr);

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
    
    // Make sure correct page is loaded into cache
    loadPage(addr);

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
    
    // Make sure correct page is loaded into cache
    loadPage(addr);

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
