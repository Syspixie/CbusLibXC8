/**
 * @file CbusLibXC8 ports.c
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
 * Code to configure hardware.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 */

#include "ports.h"
#include "hardware.h"


/**
 * Initialises I/O ports.
 */
void initPorts() {

#if defined(CPU_FAMILY_PIC18_K83)
    RB2PPS = 0x33;      //RB2->ECAN:CANTX0;    
    CANRXPPS = 0x0B;    //RB3->ECAN:CANRX;    
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
}
