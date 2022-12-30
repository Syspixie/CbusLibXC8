/**
 * @file CbusLibXC8 module.c
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
 * Interfaces to application-specific code.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 * 
 * Contains all the entry points for application-specific code.
 * 
 *      PRIMARY ROUTINES
 * 
 * 'processModule' - called from the 'main' entry point, and runs all the
 * application code.  Should never exit.  Typically some setup code followed by
 * an infinite loop calling 'receiveCbusCan' to process incoming messages, and
 * 'transmitCbusCan' to process outgoing messages.
 * 
 * 'processCbusMessage' - called when a message has been received.  Return value
 * determines whether or not a response message or error code is sent.
 * 
 * 'generateCbusMessage' - called to allow the application to send an unprompted
 * message.
 * 
 * 'processCbusEvent' - called when an event ON or OFF message has been
 * received.
 * 
 *      CALLBACKS
 * 
 * 'supportedOpCode' - called when a supported opcode is found, and any required
 * checks (node number, learn mode, setup mode) have passed.
 * 
 * 'stallMemoryWrite' - called before a memory write operation (which could take
 * a couple of milliseconds) is about to be performed.  Causes a blocking loop
 * until it returns 'true'.  Used to prevent disruption to a timing-critical
 * application operation.  Use with care!
 * 
 * 'enterFlimMode' - called when entering FLiM mode from SLiM mode.
 * 
 * 'enterSlimMode' - called when reverting to SLiM mode from FLiM mode.
 * 
 * 'validateNodeVar' - called to validate the change to a node variable.
 * 
 * 'nodeVarChanged' - called when a node variable value has changed.
 * 
 * 'validateEventVar' - called to validate the change to an event variable.
 * 
 * 'eventVarChanged' - called when an event variable value has changed.
 * 
 * 'eventRemoved' - called when an event, or all events, removed
 * 
 *      INTERRUPT SERVIVCE ROUTINES
 * 
 * 'moduleHighPriorityIsr' - called on high priority interrupt.
 * 
 * 'moduleLowPriorityIsr' - called on low priority interrupt.  The default timer
 * and the CAN BUS interface use low priority interrupts.
 */


#include "module.h"
#include "ports.h"
#include "millis.h"
#include "eeprom.h"
#include "flash.h"
#include "txmsg.h"
#include "opcode.h"
#include "interact.h"
#include "event.h"
#include "flim.h"
#include "cbuscan.h"
//#include "app.h"        // Application header


extern const uint8_t flashVersion __at(FLASH_VERSION_ADDRESS);


/**
 * Initialises EEPROM.
 * 
 * @param init Flag indicating EEPROM is uninitialised or wrong version.
 * 
 * Parameter 'init' is set during setup if the EEPROM version number does not
 * match the CURRENT_EEPROM_VERSION value; it is clear when called from a
 * 'factory reset' opcode.
 */
static void resetEeprom(bool init) {

    if (init) {
        writeEeprom8(EEPROM_BOOT_FLAG, 0);
        writeEeprom8(EEPROM_CAN_ID, 0);
        writeEeprom16(EEPROM_NODE_NUMBER, 0); 
        writeEeprom8(EEPROM_FLIM_MODE, interactStateSlim);
    }

//    appResetEeprom(init);   // Application initialise EEPROM

    if (init) writeEeprom8(EEPROM_VERSION, CURRENT_EEPROM_VERSION);
}

/**
 * Initialise FLASH (events & node variables).
 * 
 * @param init Flag indicating FLASH is uninitialised or wrong version.
 * 
 * Parameter 'init' is set during setup if the FLASH version number does not
 * match the CURRENT_FLASH_VERSION value; it is clear when called from a
 * 'factory reset' opcode.
 */
static void resetFlash(bool init) {

    if (!init) removeAllEvents();

//    appResetFlash(init);    // Application initialise FLASH

    if (init) writeFlashCached8((flashAddr_t) &flashVersion, CURRENT_FLASH_VERSION);
    flushFlashCache();
}

/**
 * Execute all application code - effectively the main() function.
 * 
 * Should comprise setup code, followed by an infinite loop.
 */
void processModule() {

    initEeprom();
    initFlash();

    // Initialise EEPROM and FLASH data if required
    if (readEeprom8(EEPROM_VERSION) != CURRENT_EEPROM_VERSION) {
        resetEeprom(true);
    }
    if (flashVersion != CURRENT_FLASH_VERSION) {
        resetFlash(true);
    }

    // Read global variables
    interactState = readEeprom8(EEPROM_FLIM_MODE);
    cbusNodeNumber.value = readEeprom16(EEPROM_NODE_NUMBER);
    canBusID = readEeprom8(EEPROM_CAN_ID);

    initTXMsg();
    initInteract();
    initEvent();
    initCbusCan();

//    appInit();      // Application initialisation

    // Enable interrupts
    INTERRUPT_EnableHigh();
    INTERRUPT_EnableLow();

    // Forever...
    while (true) {

        // Process incoming messages and responses
        if (receiveCbusCan()) {     // Calls ProcessCbusMessage
            shortFlicker();
        }

//        appProcess();   // Application process

        // Process unsolicited outgoing messages
        if (transmitCbusCan()) {    // Calls GenerateCbusMessage
            shortFlicker();
        }
    }
}

/**
 * Called when a CBUS message has been received.  Processes the message defined
 * by the opcode; optionally sends a response message.
 * 
 * @pre cbusMsg[] incoming message.
 * @return 0 no response; >0 send response; <0 cmderr.
 * @post cbusMsg[] response.
 */
int8_t processCbusMessage() {

    int8_t tx = executeOpCode();    // Process according to opcode
    return tx;
}

/**
 * Called to send a CBUS message other than as the response to a received
 * message.
 * 
 * @return 0 no response; >0 send response; <0 cmderr.
 * @post cbusMsg[] response.
 */
int8_t generateCbusMessage() {

    int8_t tx = processInteract();          // Process mode button and LEDs
    if (!tx) tx = processTXMsg();           // Process queued messages
    if (tx) return tx;

//    tx = appGenerateCbusMessage();          // Application outgoing messages

    return  tx;
}

/**
 * Called when an event ON or OFF message is received.
 * 
 * @pre cbusMsg[] incoming message.
 * @param eventIndex Index of event.
 * 
 * CBUS message data in cbusMsg[].
 * 
 *      cbusMsg[0] bit 0 gives event ON (= 0) or OFF (= 1).
 * 
 *      cbusMsg[0] bits 5:6 give number of additional data bytes (= 0..3)
 *      in cbusMsg[5..7].
 */
void processCbusEvent(uint8_t eventIndex) {

//    appProcessCbusEvent(eventIndex);        // Application incoming events
}

/**
 * Called from executeOpCode when an opcode is recognised as supported.
 * 
 * @pre cbusMsg[] incoming message.
 * @return false to prevent processing of the opcode.
 */
bool supportedOpCode() {

    longFlicker();
    return true;
}

/**
 * Called before a memory (FLASH or EEPROM) write operation.
 * 
 * @return false if temporary time-critical operation in progress.
 * 
 * Allows the application to temporarily delay FLASH and EEPROM write operations
 * in timing critical situations.  Use with caution!
 */
bool stallMemoryWrite() {

    return false;
}

/**
 * Updates the CBUS node number and FLiM mode in EEPROM.
 */
static void updateEepromMode() {

    writeEeprom16(EEPROM_NODE_NUMBER, cbusNodeNumber.value);
    writeEeprom8(EEPROM_FLIM_MODE, interactState);
}

/**
 * Called when mode button press causes switch from SLiM mode to FLiM mode.
 */
void enterFlimMode() {

    // Start process to get a new CAN ID (updates EEPROM on completion)
    enumerateCbusCanID(false);

    // Save setting change
    updateEepromMode();

//    appEnterFlimMode();     // Application enter FLiM mode
}

/**
 * Called when mode button press causes switch from FLiM mode to SLiM mode.
 */
void enterSlimMode() {

//    appLeaveFlimMode();     // Application leave FLiM mode

    // Clear CAN ID (updates EEPROM)
    setCbusCanID(0, false);

    // Save setting change
    updateEepromMode();
}

/**
 * Called when a node variable change is requested.
 * 
 * @param varIndex Index of node variable.
 * @param curValue Current NV value.
 * @param newValue New NV value.
 * @return false to reject change (e.g invalid value).
 */
bool validateNodeVar(uint8_t varIndex, uint8_t curValue, uint8_t newValue) {

    bool ret = true;

//    ret = appValidateNodeVar(varIndex, curValue, newValue);

    return ret;
}

/**
 * Called when a node variable change has been made.
 * 
 * @param varIndex Index of node variable.
 * @param oldValue Old NV value.
 * @param curValue Current NV value.
 */
void nodeVarChanged(uint8_t varIndex, uint8_t oldValue, uint8_t curValue) {

//    appNodeVarChanged(varIndex, oldValue, curValue);
}

/**
 * Called when an event variable change is requested.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param curValue Current EV value.
 * @param newValue New EV value.
 * @return false to reject change (e.g invalid value).
 */
bool validateEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t curValue, uint8_t newValue) {

    bool ret = true;

//    ret = appValidateEventVar(eventIndex, varIndex, curValue, newValue);

    return ret;
}

/**
 * Called when an event variable change has been made.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param oldValue Old EV value.
 * @param curValue Current EV value.
 */
void eventVarChanged(uint8_t eventIndex, uint8_t varIndex, uint8_t oldValue, uint8_t curValue) {

//    appEventVarChanged(eventIndex, varIndex, oldValue, curValue);
}

/**
 * Called when an event is removed.
 * 
 * @param eventIndex Index of event, or 255 for all events.
 */
void eventRemoved(uint8_t eventIndex) {

//    appEventRemoved(eventIndex);
}


// <editor-fold defaultstate="expanded" desc="Interrupt service routines">


#if !defined(IVT_BASE_ADDRESS)
/**
 * Called on high priority interrupt.
 */
void moduleHighPriorityIsr() {
}

/**
 * Called on low priority interrupt.
 */
void moduleLowPriorityIsr() {

    // Process ECAN interrupts
    cbusCanIsr();
    
    // Process TMR0 interrupts
    millisIsr();
}
#endif

/**
 * Called when the timer 'ticks'.
 */
void moduleTimerIsr() {

    cbusCanTimerIsr();      // Process 'tick' CAN bus operations
    interactTimerIsr();     // Process 'tick' button and LED operations

//    appTimerIsr();          // Application 'tick' operations
}


// </editor-fold>


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">


/**
 * Processes opcode 0x4F "Reset to manufacturers defaults".
 * 
 * @pre cbusMsg[] NNRSM message.
 * @return 0: no response.
 */
int8_t opCodeNNRSM() {

    resetEeprom(false);
    resetFlash(false);
    return 0;
}

/**
 * Processes opcode 0x5C "Put node into bootloader mode".
 * 
 * @return 0: no response.
 * 
 * @note Performs a software device reset, so will not return.
 */
int8_t opCodeBOOT() {

    writeEeprom8(EEPROM_BOOT_FLAG, 0xFF);
    RESET();    // MCU reset
    return 0;
}

/**
 * Processes opcode 0x5E "Restart node".
 * 
 * @return 0: no response.
 * 
 * @note Performs a software device reset, so will not return.
 */
int8_t opCodeNNRST() {

    RESET();    // MCU reset
    return 0;
}


// </editor-fold>
