/**
 * @file CbusLibXC8 interact.c
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
 * Code to process user interactions (button & LEDs), and the SLiM/FLiM state.
 * 
 * @author Konrad Orlowski
 * @date September 2022
 */


#include "interact.h"
#include "ports.h"
#include "millis.h"
#include "module.h"


#define FLIM_HOLD_TIME_MILLIS 4000
#ifdef INCLUDE_INTERACT_TEST_MODE
#define SET_TEST_MODE_TIME_MILLIS 8000
#define NEXT_TEST_TIME_MILLIS 1000
#endif //INCLUDE_INTERACT_TEST_MODE

#define SLOW_FLASH_TIME_MILLIS 500      // On time for a slow flash
#define FAST_FLASH_TIME_MILLIS 167      // On time for a fast flash - 3 times as fast as slow flash
#define SHORT_FLICKER_TIME_MILLIS 100   // a short flicker
#define LONG_FLICKER_TIME_MILLIS 500    // a long flicker


typedef enum {
    flashStateNone = 0,     // No flashing
    flashStateFlimSlow,     // Slow flash of FLiM LED
    flashStateFlimFast      // Fast flash of FLiM LED
} flashState_t;

typedef enum {
    flickerStateNone = 0,   // No flicker
    flickerStateShort,      // Short flicker
    flickerStateLong        // Long flicker
} flickerState_t;


// extern
interactState_t interactState;


interactState_t baseInteractState;
bool btnProgIsPressed = false;
uint16_t btnProgMillis;

flashState_t flashState;
uint16_t flashMillis;

flickerState_t flickerState;
uint16_t flickerMillis;


/**
 * Toggles the FLiM LED.
 * 
 * @post flashMillis timer reset.
 */
static void toggleFlimLed() {

    LedFlim_Toggle();
    flashMillis = getMillisShort();
}

/**
 * Turns on the LED associated with the current SLiM/FLiM state.
 * 
 * @post flashState and flickerState set to none.
 */
static void setAppropriateLedOn() {

    if (interactState == interactStateFlim) {
        LedFlim_SetHigh();
        LedSlim_SetLow();
    } else {
        LedFlim_SetLow();
        LedSlim_SetHigh();
    }
    flashState = flashStateNone;
    flickerState = flickerStateNone;
}

/**
 * Turns the flicker LED on.
 */
static void flickerLedOn() {

    if (interactState == interactStateSlim) {
        LedFlim_SetHigh();
    } else {
        LedSlim_SetHigh();
    }
}

/**
 * Initialises states.
 */
void initInteract() {

    setAppropriateLedOn();
    baseInteractState = interactState;
}

/**
 * Processes interactions.
 * 
 * Checks for program button presses; handles LEDs; processes SLiM/FLiM state
 * changes.
 * 
 * @return 1: send response; 0: no response.
 * @post cbusMsg[] RQNN or NNREL response.
 */
int8_t processInteract() {

    uint16_t millis = getMillisShort();
    int8_t tx = 0;

    // Process LED flashing
    if (flashState != flashStateNone) {
        // Toggle FLiM LED after either slow or fast interval
        uint16_t cmp = (flashState == flashStateFlimSlow) ? SLOW_FLASH_TIME_MILLIS : FAST_FLASH_TIME_MILLIS;
        if (millis - flashMillis >= cmp) toggleFlimLed();
    }

    // Process LED flickering
    if (flickerState != flickerStateNone) {
        // Turn off flicker LED after either short or long interval
        uint16_t cmp = (flickerState == flickerStateShort) ? SHORT_FLICKER_TIME_MILLIS : LONG_FLICKER_TIME_MILLIS;
        if (millis - flickerMillis >= cmp) {
            flickerState = flickerStateNone;
            if (interactState == interactStateSlim) {
                LedFlim_SetLow();
            } else {
                LedSlim_SetLow();
            }
        }
    }

    // Program button state machine
    switch (interactState) {

        // Stable SLiM mode or FLiM mode; button not pressed:
        case interactStateFlim:
        case interactStateSlim: {
            if (btnProgIsPressed) {
                // Button has been pressed
                // Start timer and wait for release
                baseInteractState = interactState;
                interactState = interactStatePressed;
                btnProgMillis = millis;
            }
        } break;

        // Button pressed; LED not yet flashing:
        case interactStatePressed: {
            if (btnProgIsPressed) {
                // Button is still pressed
                if (millis - btnProgMillis >= FLIM_HOLD_TIME_MILLIS) {
                    // Button has been pressed for 4 seconds
                    // Start flashing and wait for button release
                    interactState = interactStateFlashing;
                    flashState = flashStateFlimSlow;
                    toggleFlimLed();
                }
            } else if (baseInteractState == interactStateFlim) {
                // Button released before 4 seconds; base state was FLiM ... short press
                // Now in FLiM setup mode; start flashing
                interactState = interactStateFlimSetup;
                flashState = flashStateFlimSlow;
                toggleFlimLed();
                cbusMsg[0] = OPC_RQNN;
                tx = 1;
            } else {
                // Button released before 4 seconds; base state was SLiM
                // Ignore press
                interactState = baseInteractState;
            }
        } break;

        // Button pressed and LED flashing:
        case interactStateFlashing: {
            if (!btnProgIsPressed) {
                // Button released after 4 seconds ... long press
                if (baseInteractState == interactStateFlim) {
                    // Long press in FLiM
                    // Revert to SLiM mode
                    interactState = interactStateSlim;
                    cbusNodeNumber.value = 0;
                    enterSlimMode();
                    setAppropriateLedOn();
                    cbusMsg[0] = OPC_NNREL;
                    tx = 1;
                } else {
                    // Long press in SLiM
                    // Now in FLiM setup mode; already flashing
                    interactState = interactStateFlimSetup;
                    
                    cbusMsg[0] = OPC_RQNN;
                    tx = 1;
                }
#ifdef INCLUDE_INTERACT_TEST_MODE
            } else if (millis - btnProgMillis >= SET_TEST_MODE_TIME_MILLIS) {
                // Button has been pressed for 8 seconds
                // Start fast flashing, restart timer, and wait for button release
                interactState = interactStatePressedTest;
                btnProgMillis = millis;
                flashState = flashStateFlimFast;
                toggleFlimLed();
#endif // INCLUDE_INTERACT_TEST_MODE
            }
        } break;

        // Stable FLiM setup mode; button not pressed:
        case interactStateFlimSetup: {
            if (btnProgIsPressed) {
                // Button pressed in FLiM setup mode
                // Start timer, and wait for release
                interactState = interactStatePressedSetup;
                btnProgMillis = millis;
            }
        } break;

        // FLiM setup mode; button pressed:
        case interactStatePressedSetup: {
            if (!btnProgIsPressed) {
                // Button released in FLiM setup mode
                // Revert to base mode
                interactState = baseInteractState;
                setAppropriateLedOn();
            }
        } break;

#ifdef INCLUDE_INTERACT_TEST_MODE
        // Stable test mode; button not pressed:
        case interactStateTestMode: {
            if (btnProgIsPressed) {
                // Button has been pressed
                // Start timer and wait for release
                interactState = interactStatePressedTest;
                btnProgMillis = millis;
            }
        } break;

        // Test mode; button pressed:
        case interactStatePressedTest: {
            if (!btnProgIsPressed) {
                if (millis - btnProgMillis >= FLIM_HOLD_TIME_MILLIS) {
                    // Button released after 4 seconds  ... long press
                    // Revert to base mode
                    interactState = baseInteractState;
                    setAppropriateLedOn();
                } else if (millis - btnProgMillis >= NEXT_TEST_TIME_MILLIS) {
                    // Button released after 1 second ... short press
                    // Start next test
                    interactState = interactStateNextTest;
                } else {
                    // Button released before 1 second ... very short press
                    // Input to current test
                    interactState = interactStateTestInput;
                }
            }
        } break;
        /*
         * Someone, somewhere, needs to handle states interactStateTestInput and
         * interactStateNextTest, and get back to interactStateTestMode.         
         */
#endif //INCLUDE_INTERACT_TEST_MODE

        default: {
        } break;
    }
    return tx;
}

/**
 * Requests a short LED flicker.
 */
void shortFlicker() {

    // Short flicker can't override a long flicker
    if (flickerState == flickerStateLong) return;

    // Start of short flicker
    flickerState = flickerStateShort;
    flickerMillis = getMillisShort();
    flickerLedOn();
}

/**
 * Requests a long LED flicker.
 */
void longFlicker() {

    // Long flicker can override a short flicker
    // Start of long flicker
    flickerState = flickerStateLong;
    flickerMillis = getMillisShort();
    flickerLedOn();
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


/**
 * Checks the state of the program button to detect press and release.
 * 
 * Called at 2ms intervals.  Press defined as 'off' followed by 15 'on's;
 * release defined as 'on' followed by 15 'off's.  So state must have changed,
 * and the new state have been stable for 30ms.
 * 
 * @post btnProgIsPressed updated.
 */
static void btnProgDebounceIsr() {

    // Static variable for button state shift register.
    static bytes16_t shift = {.value = (uint16_t) ~0};

    // Shift old states and append current state
    shift.value <<= 1;
    shift.bytes[0] |= BtnProg_GetValue();    // pressed = 0

    // Look for state change followed by stable period
    if (shift.value == 0x8000) {
        btnProgIsPressed = true;
    } else if (shift.value == 0x7FFF) {
        btnProgIsPressed = false;
    }
}

/**
 * Performs regular interaction operations (called every millisecond).
 */
void interactTimerIsr(void) {

    static uint8_t toggle = 0;

    // Simple divide by 2 to give reasonable button state table time
    toggle ^= 1;
    if (toggle == 0) btnProgDebounceIsr();      // Called every 2ms
}


// </editor-fold>


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">


/**
 * Processes opcode 0x42 "Set Node Number"
 * 
 * @pre cbusMsg[] SNN message.
 * @return 1: send response.
 * @post cbusMsg[] NNACK response.
 */
int8_t opCodeSNN() {

    interactState = interactStateFlim;
    cbusNodeNumber.valueH = cbusMsg[1];
    cbusNodeNumber.valueL = cbusMsg[2];
    enterFlimMode();
    setAppropriateLedOn();
    cbusMsg[0] = OPC_NNACK;
    return 1;
}


// </editor-fold>
