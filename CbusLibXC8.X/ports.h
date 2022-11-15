/**
 * @file CbusLibXC8 ports.h
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

#ifndef PORTS_H
#define	PORTS_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"


// get/set BtnProg aliases
#define BtnProg_TRIS                 TRISAbits.TRISA2
#define BtnProg_LAT                  LATAbits.LATA2
#define BtnProg_PORT                 PORTAbits.RA2
#define BtnProg_ANS                  ANCON0bits.ANSEL2
#define BtnProg_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define BtnProg_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define BtnProg_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define BtnProg_GetValue()           PORTAbits.RA2
#define BtnProg_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define BtnProg_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define BtnProg_SetAnalogMode()      do { ANCON0bits.ANSEL2 = 1; } while(0)
#define BtnProg_SetDigitalMode()     do { ANCON0bits.ANSEL2 = 0; } while(0)

// get/set LedFlim aliases
#define LedFlim_TRIS                 TRISBbits.TRISB6
#define LedFlim_LAT                  LATBbits.LATB6
#define LedFlim_PORT                 PORTBbits.RB6
#define LedFlim_WPU                  WPUBbits.WPUB6
#define LedFlim_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define LedFlim_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define LedFlim_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define LedFlim_GetValue()           PORTBbits.RB6
#define LedFlim_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define LedFlim_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define LedFlim_SetPullup()          do { WPUBbits.WPUB6 = 1; } while(0)
#define LedFlim_ResetPullup()        do { WPUBbits.WPUB6 = 0; } while(0)

// get/set LedSlim aliases
#define LedSlim_TRIS                 TRISBbits.TRISB7
#define LedSlim_LAT                  LATBbits.LATB7
#define LedSlim_PORT                 PORTBbits.RB7
#define LedSlim_WPU                  WPUBbits.WPUB7
#define LedSlim_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define LedSlim_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define LedSlim_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define LedSlim_GetValue()           PORTBbits.RB7
#define LedSlim_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define LedSlim_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define LedSlim_SetPullup()          do { WPUBbits.WPUB7 = 1; } while(0)
#define LedSlim_ResetPullup()        do { WPUBbits.WPUB7 = 0; } while(0)


    void initPorts(void);


#ifdef	__cplusplus
}
#endif

#endif	/* PORTS_H */
