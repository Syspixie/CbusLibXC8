/**
 * @file CbusLibXC8 main.c
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
 * @author Konrad Orlowski
 * @date August 2022
 */


#pragma warning disable 520     // function is never called
#pragma warning disable 1471    // indirect function call via a NULL pointer ignored
#pragma warning disable 1498    // pointer in expression may have no targets
#pragma warning disable 1510    // non-reentrant routine has been duplicated


#include "hardware.h"
#include "ports.h"
#include "millis.h"
#include "module.h"


/**
 * Application entry point.
 */
void __section("mainSec") main() {

#if defined(CPU_FAMILY_PIC18_K80)
    RCONbits.IPEN = 1;              // Enable high/low interrupt priorities
#endif
#if defined(CPU_FAMILY_PIC18_K83)
    INTCON0bits.IPEN = 1;           // Enable high/low interrupt priorities
#endif

#if defined(IVT_BASE_ADDRESS)
    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00;   // unlock IVT

    IVTBASE = IVT_BASE_ADDRESS;     // Move the IVT (Interrupt Vector Table)

    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01;   // lock IVT
#endif

    initPorts();
    initMillis();

    processModule();
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


#if defined(CPU_FAMILY_PIC18_K80)
/**
 * High priority interrupt service handler.
 */
void __interrupt(high_priority) __section("mainSec") isrHigh() {

    moduleHighPriorityIsr();
}

/**
 * Low priority interrupt service handler.
 */
void __interrupt(low_priority) __section("mainSec") isrLow() {

    moduleLowPriorityIsr();
}
#endif
#if defined(CPU_FAMILY_PIC18_K83)
void __interrupt(irq(default), base(IVT_BASE_ADDRESS)) Default_ISR() {}
#endif


// </editor-fold>
