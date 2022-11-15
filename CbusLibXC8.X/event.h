/**
 * @file CbusLibXC8 event.h
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

#ifndef EVENT_H
#define	EVENT_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"


    void initEvent(void);
    uint8_t findEvent(uint16_t nodeNumber, uint16_t eventNumber);
    uint8_t addEvent(uint16_t nodeNumber, uint16_t eventNumber, uint8_t evNumber, uint8_t evValue, bool useMyNodeNumber);
    uint8_t writeEventVar(uint8_t eventIndex, uint8_t evNumber, uint8_t evValue);
    uint8_t updateEvent(uint8_t eventIndex, uint16_t nodeNumber, uint16_t eventNumber, uint8_t evNumber, uint8_t evValue);
    uint8_t getEventVarValue(uint8_t eventIndex, uint8_t varNumber, uint8_t* varValuePtr);
    uint8_t getEventVarCount(uint8_t eventIndex, uint8_t* varCountPtr);
    uint8_t removeEvent(uint8_t eventIndex);
    uint8_t countEvents(void);
    uint8_t countEventSlotsFree(void);
    void removeAllEvents(void);


#ifdef	__cplusplus
}
#endif

#endif	/* EVENT_H */
