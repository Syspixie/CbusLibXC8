/**
 * @file CbusLibXC8 opcode.h
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

#ifndef OPCODE_H
#define	OPCODE_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include "global.h"


    int8_t opCodeQNN(void);     // 0x0D
    int8_t opCodeRQNP(void);    // 0x10
    int8_t opCodeRQMN(void);    // 0x0D
    int8_t opCodeSNN(void);     // 0x42
    int8_t opCodeNNRSM(void);   // 0x4F
    int8_t opCodeNNLRN(void);   // 0x53
    int8_t opCodeNNULN(void);   // 0x54
    int8_t opCodeNNCLR(void);   // 0x55
    int8_t opCodeNNEVN(void);   // 0x56
    int8_t opCodeNERD(void);    // 0x57
    int8_t opCodeRQEVN(void);   // 0x58
    int8_t opCodeBOOT(void);    // 0x5C
    int8_t opCodeENUM(void);    // 0x5D
    int8_t opCodeNNRST(void);   // 0x5E
    int8_t opCodeNVRD(void);    // 0x71
    int8_t opCodeNENRD(void);   // 0x72
    int8_t opCodeRQNPN(void);   // 0x73
    int8_t opCodeCANID(void);   // 0x75
    int8_t opCodeACOxn(void);   // 0x90 0x91 0xB0 0xB1 )xD0 0xD1 0xF0 0xF1
    int8_t opCodeEVULN(void);   // 0x95
    int8_t opCodeNVSET(void);   // 0x96
    int8_t opCodeASOxn(void);   // 0x98 0x99 0xB8 0xB9 )xD8 0xD9 0xF8 0xF9
    int8_t opCodeREVAL(void);   // 0x9C
    int8_t opCodeREQEV(void);   // 0xB2
    int8_t opCodeEVLRN(void);   // 0xD2
    int8_t opCodeEVLRNI(void);  // 0xF5


    int8_t executeOpCode(void);
    void opCodeAddNodeNumber(void);


#ifdef	__cplusplus
}
#endif

#endif	/* OPCODE_H */
