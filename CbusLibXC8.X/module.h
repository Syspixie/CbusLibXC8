/**
 * @file CbusLibXC8 module.h
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

#ifndef MODULE_H
#define	MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"
#include "moduledefs.h"


    typedef enum {
        interactStateSlim = 0,      // SLiM mode
        interactStateFlim,          // FLiM mode
        interactStatePressed,       // Button pressed in SLiM or FLiM mode
        interactStateFlashing,      // Button still pressed; flashing
        interactStateFlimSetup,     // FLiM Setup mode; flashing
        interactStateFlimLearn,     // FLiM Learn mode
        interactStatePressedSetup,  // Button pressed in FLiM Setup mode
#ifdef INCLUDE_INTERACT_TEST_MODE
        interactStatePressedTest,   // Pressed during test
        interactStateTestMode,      // Self test mode
        interactStateNextTest,      // Move on to next test
        interactStateTestInput      // Input to current test mode
#endif //INCLUDE_INTERACT_TEST_MODE
    } interactState_t;


    extern interactState_t interactState;
    extern bytes16_t cbusNodeNumber;
    extern uint8_t canBusID;
    extern bytes16_t encodedCanBusID;
    extern uint8_t cbusMsg[8];


    void moduleHighPriorityIsr(void);
    void moduleLowPriorityIsr(void);
    void moduleTimerIsr(void);
    void processModule(void);
    int8_t processCbusMessage(void);
    int8_t generateCbusMessage(void);
    void processCbusEvent(uint8_t eventIndex);
    bool supportedOpCode(void);
    bool stallMemoryWrite(void);
    void enterFlimMode(void);
    void enterSlimMode(void);
    bool validateNodeVar(uint8_t index, uint8_t oldValue, uint8_t newValue);
    void nodeVarChanged(uint8_t index, uint8_t oldValue, uint8_t newValue);


#ifdef	__cplusplus
}
#endif

#endif	/* MODULE_H */
