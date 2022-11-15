/**
 * @file CbusLibXC8 millis.c
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
 * Code to provide a general-purpose one millisecond timer/counter.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 * 
 * @note Uses hardware TIMER0 module.
 * 
 * Provides 'normal' 32-bit timer/counter, with an overrun period of 49.7 days.
 * By using only the lower word of the same variable, we have a 16-bit
 * timer/counter with an overrun period of 65 seconds - perfectly adequate for
 * timing events in the order of a few hundred milliseconds.
 * 
 * The millisTicks counter should not, ordinarily, be accessed directly, but by
 * the getMillis and getMillisShort functions, which ensure that the value
 * returned has not been incremented during the multi-byte read operation
 * (without disabling interrupts).
 * 
 * However, as the increment only occurs during an interrupt, code running as
 * part of an interrupt service routine, or with interrupts otherwise disabled,
 * can safely access millisTicks or millisTicks.word[0].  If separate low and
 * high priority interrupts are enabled, then this applies ONLY to interrupts
 * at the SAME priority as the millis interrupt.
 */


#include "millis.h"
#include "hardware.h"


// extern
volatile bytes32_t millisTicks = {.value = 0};


/**
 * Initialises timer/counter 0 for 1ms tick on 64MHz system clock.
 */
void initMillis() {

#if defined(CPU_FAMILY_PIC18_K80)
    INTCON2bits.TMR0IP = 0;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;

    T0CON = 0xC5;   // T0PS 1:64; T08BIT 8-bit; T0SE Increment_lo_hi; T0CS FOSC/4; TMR0ON enabled; PSA assigned; 

    // Count 6..256
    TMR0L = 0x06;
    TMR0H = 0x00;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    IPR3bits.TMR0IP = 0;
    PIR3bits.TMR0IF = 0;
    PIE3bits.TMR0IE = 1;
    
    T0CON0 = 0x80;  // T0OUTPS 1:1; T0EN enabled; T016BIT 8-bit; 
    T0CON1 = 0x46;  // T0CS FOSC/4; T0CKPS 1:64; T0ASYNC synchronised; 

    // Count 0..249
    TMR0L = 0x00;
    TMR0H = 0xF9;
#endif
}

/**
 * Gets the current 32-bit timer/counter value.
 * 
 * @return Timer/counter value.
 */
uint32_t getMillis() {

    uint32_t t;
    uint8_t b;
    do {
        b = millisTicks.bytes[0];
        t = millisTicks.value;
    } while (millisTicks.bytes[0] != b);

    return t;
}

/**
 * Gets the current 16-bit timer/counter value.
 * 
 * @return Timer/counter value.
 */
uint16_t getMillisShort() {

    uint16_t t;
    uint8_t b;
    do {
        b = millisTicks.bytes[0];
        t = millisTicks.words[0];
    } while (millisTicks.bytes[0] != b);

    return t;
}

/**
 * Pauses execution for the specified 32-bit milliseconds period.
 * 
 * @param ms Delay period in milliseconds.
 * 
 * @note This is a blocking delay.
 */
void delayMillis(uint32_t ms) {

    uint32_t t = getMillis();
    while (getMillis() - t < ms);   // Blocking loop
}

/**
 * Pauses execution for the specified 16-bit milliseconds period.
 * 
 * @param ms Delay period in milliseconds.
 * 
 * @note This is a blocking delay.
 */
void delayMillisShort(uint16_t ms) {

    uint16_t t = getMillisShort();
    while (getMillis() - t < ms);   // Blocking loop
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Service routine for TIMER0 interrupts.
 * 
 * @return true if timer has 'ticked'.
 */
bool millisIsr() {
    
    bool ticked = false;

    // If TIMER0 has 'ticked' (once every millisecond)...
#if defined(CPU_FAMILY_PIC18_K80)
    if (INTCONbits.TMR0IF == 1) {

        // reload TMR0
        TMR0L = 0x06;
        TMR0H = 0x00;

        INTCONbits.TMR0IF = 0;
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    if (PIR3bits.TMR0IF == 1) {

        PIR3bits.TMR0IF = 0;
#endif

        // Increment timer/counter value
        ++millisTicks.value;
        ticked = true;
    }

    return ticked;
}


// </editor-fold>
