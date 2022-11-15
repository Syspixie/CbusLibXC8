/**
 * @file CbusLibBootXC8 global.h
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
 * @date August 2022
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>


    typedef union {
        uint8_t value;
        struct {
            unsigned bit0 : 1;
            unsigned bit1 : 1;
            unsigned bit2 : 1;
            unsigned bit3 : 1;
            unsigned bit4 : 1;
            unsigned bit5 : 1;
            unsigned bit6 : 1;
            unsigned bit7 : 1;
        };
    } bits8_t;

    typedef union {
        uint16_t value;
        struct {
            uint8_t valueL;
            uint8_t valueH;
        };
        uint8_t bytes[2];
    } bytes16_t;

    typedef union {
        uint24_t value;
        struct {
            uint8_t valueL;
            uint8_t valueH;
            uint8_t valueU;
        };
        uint8_t bytes[3];
    } bytes24_t;

    typedef union {
        uint32_t value;
        uint8_t bytes[4];
        uint16_t words[2];
    } bytes32_t;

    typedef union {
        struct {
            uint8_t d0;
            uint8_t d1;
            uint8_t d2;
            uint8_t d3;
            uint8_t d4;
            uint8_t d5;
            uint8_t d6;
            uint8_t d7;
        };
        uint8_t bytes[8];
        uint16_t words[4];
    } bytes64_t;


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_H */
