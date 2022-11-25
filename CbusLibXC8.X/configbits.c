/**
 * @file CbusLibXC8 configbits.c
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
 * @date October 2022
 */


#include "hardware.h"


// *****************************************************************************
#if defined(CPU_FAMILY_PIC18_K80)

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = HS1       // Oscillator (HS oscillator (Medium power, 4 MHz - 16 MHz))
#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power Up Timer (Enabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 0         // Brown-out Reset Voltage bits (3.0V)
#pragma config BORPWR = LOW     // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB1K     // Boot Block Size (1K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-01FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 02000-03FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 04000-05FFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 06000-07FFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-01FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 02000-03FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 04000-05FFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 06000-07FFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-01FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 02000-03FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 04000-05FFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 06000-07FFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

#endif
// *****************************************************************************
#if defined(CPU_FAMILY_PIC18_K83)

// CONFIG1L
#pragma config FEXTOSC = HS     // External Oscillator Selection->HS (crystal oscillator) above 8 MHz; PFM set to high power
#pragma config RSTOSC = EXTOSC_4PLL // Reset Oscillator Selection->EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit->CLKOUT function is disabled
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit->PRLOCK bit can be set and cleared repeatedly
#pragma config CSWEN = OFF      // Clock Switch Enable bit->The NOSC and NDIV bits cannot be changed by user software
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor disabled

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTS = PWRT_64  // Power-up timer selection bits->PWRT set at 64ms
#pragma config MVECEN = ON      // Multi-vector enable bit->Multi-vector enabled, Vector table used for interrupts
#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit->IVTLOCK bit can be cleared and set repeatedly
#pragma config LPBOREN = ON     // Low Power BOR Enable bit->ULPBOR enabled
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored

// CONFIG2H
#pragma config BORV = VBOR_2P85 // Brown-out Reset Voltage Selection bits->Brown-out Reset Voltage (VBOR) set to 2.8V
#pragma config ZCD = OFF        // ZCD Disable bit->ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit->PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config DEBUG = OFF      // Debugger Enable bit->Background debugger disabled
#pragma config XINST = OFF      // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31   // WDT Period selection bits->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = OFF       // WDT operating mode->WDT Disabled; SWDTEN is ignored

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7    // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = SC      // WDT input clock selector->Software Control

// CONFIG4L
#pragma config BBSIZE = BBSIZE_1024 // Boot Block Size selection bits->Boot Block size is 1024 words
#pragma config BBEN = ON        // Boot Block enable bit->Boot block enabled
#pragma config SAFEN = OFF      // Storage Area Flash enable bit->SAF disabled
#pragma config WRTAPP = OFF     // Application Block write protection bit->Application Block not write protected

// CONFIG4H
#pragma config WRTB = OFF       // Boot Block Write Protection bit->Boot Block not write-protected
#pragma config WRTC = OFF       // Configuration Register Write Protection bit->Configuration registers (300000-30000Bh) not write-protected
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit->Data EEPROM not write-protected
#pragma config WRTSAF = OFF     // SAF Write protection bit->SAF not Write Protected
#pragma config LVP = ON         // Low Voltage Programming Enable bit->HV on MCLR/VPP must be used for programming

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit->PFM and Data EEPROM code protection disabled

#endif
// *****************************************************************************
#if defined(CPU_FAMILY_PIC18_Q83Q84)

// CONFIG1
#pragma config FEXTOSC = HS     // External Oscillator Selection->HS (crystal oscillator) above 8 MHz
#pragma config RSTOSC = EXTOSC  // Reset Oscillator Selection->EXTOSC operating per FEXTOSC bits (device manufacturing default)

// CONFIG2
#pragma config CLKOUTEN = OFF   // Clock out Enable bit->CLKOUT function is disabled
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit->PRLOCKED bit can be cleared and set only once
#pragma config CSWEN = ON       // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config JTAGEN = OFF     // JTAG Enable bit->Disable JTAG Boundary Scan mode, JTAG pins revert to user functions
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor enabled
#pragma config FCMENP = ON      // Fail-Safe Clock Monitor -Primary XTAL Enable bit->FSCM timer will set FSCMP bit and OSFIF interrupt on Primary XTAL failure
#pragma config FCMENS = ON      // Fail-Safe Clock Monitor -Secondary XTAL Enable bit->FSCM timer will set FSCMS bit and OSFIF interrupt on Secondary XTAL failure

// CONFIG3
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits->PWRT is disabled
#pragma config MVECEN = ON      // Multi-vector enable bit->Multi-vector enabled, Vector table used for interrupts
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit->IVTLOCKED bit can be cleared and set only once
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit->Low-Power BOR disabled
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored

// CONFIG4
#pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits->Brown-out Reset Voltage (VBOR) set to 1.9V
#pragma config ZCD = OFF        // ZCD Disable bit->ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit->PPSLOCKED bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config LVP = ON         // Low Voltage Programming Enable bit->Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored
#pragma config XINST = OFF      // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled

// CONFIG5
#pragma config WDTCPS = WDTCPS_31   // WDT Period selection bits->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = OFF       // WDT operating mode->WDT Disabled; SWDTEN is ignored

// CONFIG6
#pragma config WDTCWS = WDTCWS_7    // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = SC      // WDT input clock selector->Software Control

// CONFIG7
#pragma config BBSIZE = BBSIZE_1024 // Boot Block Size selection bits->Boot Block size is 1024 words
#pragma config BBEN = ON        // Boot Block enable bit->Boot block enabled
#pragma config SAFEN = OFF      // Storage Area Flash enable bit->SAF disabled

// CONFIG8
#pragma config WRTB = OFF       // Boot Block Write Protection bit->Boot Block not Write protected
#pragma config WRTC = OFF       // Configuration Register Write Protection bit->Configuration registers not Write protected
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit->Data EEPROM not Write protected
#pragma config WRTSAF = OFF     // SAF Write protection bit->SAF not Write Protected
#pragma config WRTAPP = OFF     // Application Block write protection bit->Application Block not write protected

// CONFIG9
#pragma config BOOTPINSEL = RC5 // CRC on boot output pin selection->CRC on boot output pin is RC5
#pragma config BPEN = OFF       // CRC on boot output pin enable bit->CRC on boot output pin disabled
#pragma config ODCON = OFF      // CRC on boot output pin open drain bit->Pin drives both high-going and low-going signals

// CONFIG10
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit->PFM and Data EEPROM code protection disabled

// CONFIG11
#pragma config BOOTSCEN = OFF   // CRC on boot scan enable for boot area->CRC on boot will not include the boot area of program memory in its calculation
#pragma config BOOTCOE = HALT   // CRC on boot Continue on Error for boot areas bit->CRC on boot will stop device if error is detected in boot areas
#pragma config APPSCEN = OFF    // CRC on boot application code scan enable->CRC on boot will not include the application area of program memory in its calculation
#pragma config SAFSCEN = OFF    // CRC on boot SAF area scan enable->CRC on boot will not include the SAF area of program memory in its calculation
#pragma config DATASCEN = OFF   // CRC on boot Data EEPROM scan enable->CRC on boot will not include data EEPROM in its calculation
#pragma config CFGSCEN = OFF    // CRC on boot Config fuses scan enable->CRC on boot will not include the configuration fuses in its calculation
#pragma config COE = HALT       // CRC on boot Continue on Error for non-boot areas bit->CRC on boot will stop device if error is detected in non-boot areas
#pragma config BOOTPOR = OFF    // Boot on CRC Enable bit->CRC on boot will not run

// CONFIG12
#pragma config BCRCPOLT = hFF   // Boot Sector Polynomial for CRC on boot bits 31-24->Bits 31:24 of BCRCPOL are 0xFF

// CONFIG13
#pragma config BCRCPOLU = hFF   // Boot Sector Polynomial for CRC on boot bits 23-16->Bits 23:16 of BCRCPOL are 0xFF

// CONFIG14
#pragma config BCRCPOLH = hFF   // Boot Sector Polynomial for CRC on boot bits 15-8->Bits 15:8 of BCRCPOL are 0xFF

// CONFIG15
#pragma config BCRCPOLL = hFF   // Boot Sector Polynomial for CRC on boot bits 7-0->Bits 7:0 of BCRCPOL are 0xFF

// CONFIG16
#pragma config BCRCSEEDT = hFF  // Boot Sector Seed for CRC on boot bits 31-24->Bits 31:24 of BCRCSEED are 0xFF

// CONFIG17
#pragma config BCRCSEEDU = hFF  // Boot Sector Seed for CRC on boot bits 23-16->Bits 23:16 of BCRCSEED are 0xFF

// CONFIG18
#pragma config BCRCSEEDH = hFF  // Boot Sector Seed for CRC on boot bits 15-8->Bits 15:8 of BCRCSEED are 0xFF

// CONFIG19
#pragma config BCRCSEEDL = hFF  // Boot Sector Seed for CRC on boot bits 7-0->Bits 7:0 of BCRCSEED are 0xFF

// CONFIG20
#pragma config BCRCEREST = hFF  // Boot Sector Expected Value for CRC on boot bits 31-24->Bits 31:24 of BCRCERES are 0xFF

// CONFIG21
#pragma config BCRCERESU = hFF  // Boot Sector Expected Value for CRC on boot bits 23-16->Bits 23:16 of BCRCERES are 0xFF

// CONFIG22
#pragma config BCRCERESH = hFF  // Boot Sector Expected Value for CRC on boot bits 15-8->Bits 15:8 of BCRCERES are 0xFF

// CONFIG23
#pragma config BCRCERESL = hFF  // Boot Sector Expected Value for CRC on boot bits 7-0->Bits 7:0 of BCRCERES are 0xFF

// CONFIG24
#pragma config CRCPOLT = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 31-24->Bits 31:24 of CRCPOL are 0xFF

// CONFIG25
#pragma config CRCPOLU = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 23-16->Bits 23:16 of CRCPOL are 0xFF

// CONFIG26
#pragma config CRCPOLH = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 15-8->Bits 15:8 of CRCPOL are 0xFF

// CONFIG27
#pragma config CRCPOLL = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 7-0->Bits 7:0 of CRCPOL are 0xFF

// CONFIG28
#pragma config CRCSEEDT = hFF   // Non-Boot Sector Seed for CRC on boot bits 31-24->Bits 31:24 of CRCSEED are 0xFF

// CONFIG29
#pragma config CRCSEEDU = hFF   // Non-Boot Sector Seed for CRC on boot bits 23-16->Bits 23:16 of CRCSEED are 0xFF

// CONFIG30
#pragma config CRCSEEDH = hFF   // Non-Boot Sector Seed for CRC on boot bits 15-8->Bits 15:8 of CRCSEED are 0xFF

// CONFIG31
#pragma config CRCSEEDL = hFF   // Non-Boot Sector Seed for CRC on boot bits 7-0->Bits 7:0 of CRCSEED are 0xFF

// CONFIG32
#pragma config CRCEREST = hFF  // Non-Boot Sector Expected Value for CRC on boot bits 31-24->Bits 31:24 of CRCERES are 0xFF

// CONFIG33
#pragma config CRCERESU = hFF  // Non-Boot Sector Expected Value for CRC on boot bits 23-16->Bits 23:16 of CRCERES are 0xFF

// CONFIG34
#pragma config CRCERESH = hFF  // Non-Boot Sector Expected Value for CRC on boot bits 15-8->Bits 15:8 of CRCERES are 0xFF

// CONFIG35
#pragma config CRCERESL = hFF  // Non-Boot Sector Expected Value for CRC on boot bits 7-0->Bits 7:0 of CRCERES are 0xFF

#endif
