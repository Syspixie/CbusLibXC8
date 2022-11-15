/**
 * @file CbusLibXC8 event.c
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
 * Code to handle CBUS events and event variables.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 */


#include "event.h"
#include "util.h"
#include "module.h"
#include "flash.h"
#include "timedresponse.h"


#define NO_INDEX 0xFF
#define NULL_VAR 0

#define EVENT_SHORT_SIZE 6      // Everything but event variables themselves


typedef union {
    struct {
        unsigned useMyNodeNumber : 1;
    };
    uint8_t byte;
} eventFlags_t;

typedef union {
    struct {
        eventFlags_t flags;     // 0xFF indicates undefined (free) event entry
        bytes16_t nodeNumber;
        bytes16_t eventNumber;
        uint8_t numVars;
        uint8_t vars[EVENT_NUM_VARS];
    };
    uint8_t bytes[EVENT_SIZE];
} event_t;


extern const event_t eventTable[MAX_NUM_EVENTS] __at(EVENTS_FLASH_ADDRESS);


bytes16_t myNodeNumber;     // cbusNodeNumber when the hash table was built

uint8_t freeHead;                       // Head of chain of events that are free
uint8_t hashHead[EVENT_HASH_SIZE];      // Heads of chains of events by hash value
uint8_t chainMap[MAX_NUM_EVENTS];       // Further chain links

eventFlags_t eventFlags[MAX_NUM_EVENTS];    // Cache of event flags

event_t event;      // Event read from FLASH


/**
 * Returns the hash value of an event ID (node number and event number).
 * 
 * @param nn Node number.
 * @param en Event number.
 * @return Hash value between 0 and EVENT_HASH_SIZE-1.
 */
static inline uint8_t calculateHash(bytes16_t nn, bytes16_t en) {

    uint8_t hash = nn.bytes[0] ^ nn.bytes[1];
    hash *= 7;
    hash += en.bytes[0] ^ en.bytes[1];
    return hash % EVENT_HASH_SIZE;
}

/**
 * Checks whether an event index is within range, and whether it is a defined
 * event.
 * 
 * @param eventIndex Index to check.
 * @return 0 (success) if eventIndex is between 0 and MAX_NUM_EVENTS-1, and is
 * the index of a defined event; returns CMDERR error value otherwise.
 */
static uint8_t checkEventIndex(uint8_t eventIndex) {

    if (eventIndex >= MAX_NUM_EVENTS) return CMDERR_INVALID_EVENT;
    if (eventFlags[eventIndex].byte == 0xFF) return CMDERR_INVALID_EVENT;
    return 0;
}

/**
 * Checks whether an event variable index is within range.
 * 
 * @param varIndex Index to check.
 * @return 0 (success) if varIndex is between 0 and EVENT_NUM_VARS-1; returns
 * CMDERR error value otherwise.
 */
static uint8_t checkVarIndex(uint8_t varIndex) {

    if (varIndex >= EVENT_NUM_VARS) return CMDERR_INV_EV_IDX;
    return 0;
}

/**
 * Fetches event data header (no event variables) from FLASH.  Returns false if
 * location is not a defined event.  If the useMyNodeNumber flags is set, the
 * node number is replaced with the current node number.
 * 
 * @param eventIndex Index of event to fetch.
 * @return true if a defined event.
 * @post event populated with event header (no variables).
 */
static bool fetchEventHeader(uint8_t eventIndex) {
    
    readFlash((flashAddr_t) &eventTable[eventIndex], &event, EVENT_SHORT_SIZE);
    if (event.flags.byte == 0xFF) return false;

    if (event.flags.useMyNodeNumber) {
        event.nodeNumber.value = myNodeNumber.value;
    }
    return true;
}

/**
 * Rebuilds the chains of linked events in RAM, speeding up access.  Must be
 * called on initialisation, and when any event data other than variable values
 * has been written to FLASH.
 * 
 * @post myNodeNumber set to current node number.
 * @post freeHead start of free event index chain.
 * @post hashHead[] start of hash valued event index chains.
 * @post chainMap[] event index chains.
 * @post eventFlags[] copies of event flags.
 */
static void rebuildChainLinks() {

    // Save our node number - need to rebuild if this changes
    myNodeNumber.value = cbusNodeNumber.value;

    // Set all links (head and map) no NO_INDEX;
    freeHead = NO_INDEX;
    utilMemset(hashHead, NO_INDEX, EVENT_HASH_SIZE);
    utilMemset(chainMap, NO_INDEX, MAX_NUM_EVENTS);

    // Loop for all event indexes backwards; indexes end up on free list in
    // reverse order - looping backwards means that we end up with free indexes
    // being allocated to events in the 'correct' order (... trivial detail)
    for (uint8_t i = 0; i < MAX_NUM_EVENTS; ++i) {
        uint8_t eventIndex = (MAX_NUM_EVENTS - 1) - i;

        // If this is an event...
        if (fetchEventHeader(eventIndex)) {

            // Add this event to hash valued chain
            uint8_t hash = calculateHash(event.nodeNumber, event.eventNumber);
            chainMap[eventIndex] = hashHead[hash];
            hashHead[hash] = eventIndex;

        } else {

            // Add this event to free chain
            chainMap[eventIndex] = freeHead;
            freeHead = eventIndex;
        }

        // Cache event flags
        eventFlags[eventIndex] = event.flags;
    }
}

/**
 * Returns a free event index.
 * 
 * @return Event index, or NO_INDEX if none free.
 * @post freeHead updated.
 */
static uint8_t getFreeIndex() {

    // Get the first free event; assuming there is one, the next link in the
    // chain becomes the head
    uint8_t eventIndex = freeHead;
    if (eventIndex != NO_INDEX) freeHead = chainMap[eventIndex];
    return eventIndex;
}

/**
 * Writes a new event to FLASH at a given index.
 * 
 * @param eventIndex Index to which event is written.
 * @param nodeNumber Node number.
 * @param eventNumber Event number.
 * @param useMyNodeNumber Event flag.
 * @return 0 (success).
 * @post event populated with the new event data.
 * @post FLASH cache updated but not flushed.
 * 
 * @note flushFlashCache() and rebuildChainLinks() must be called to complete
 * the process.
 */
static uint8_t doAddEvent(uint8_t eventIndex, uint16_t nodeNumber, uint16_t eventNumber, bool useMyNodeNumber) {

    event.flags.byte = 0;
    event.flags.useMyNodeNumber = useMyNodeNumber;
    event.nodeNumber.value = nodeNumber;
    event.eventNumber.value = eventNumber;
    event.numVars = 0;
    utilMemset(event.vars, NULL_VAR, EVENT_NUM_VARS);
    writeFlashCached((flashAddr_t) &eventTable[eventIndex], &event, EVENT_SIZE);

    return 0;
}

/**
 * Writes an event variable value to FLASH.
 * 
 * @param eventIndex index of event.
 * @param varIndex Index of event variable.
 * @param varValue Value to be written.
 * @return 0 (success).
 * @post FLASH cache updated but not flushed.
 * 
 * @note flushFlashCache() must be called to complete the process.
 */
static uint8_t doWriteEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t varValue) {

    // Just update the event
    writeFlashCached8((flashAddr_t) &eventTable[eventIndex].vars[varIndex], varValue);

    // Update varsInUse if this variable number is larger
    uint8_t n = readFlashCached8((flashAddr_t) &eventTable[eventIndex].numVars);
    if (varIndex >= n) {
        writeFlashCached8((flashAddr_t) &eventTable[eventIndex].numVars, varIndex + 1);
    }

    return 0;
}

/**
 * Reads an event variable value from FLASH.
 * 
 * @param eventIndex index of event.
 * @param varIndex Index of event variable.
 * @param[out] varValuePtr pointer to location where value is to be written.
 * @return 0 (success) if event variable is in use and has a value; returns
 * CMDERR error value otherwise.
 */
static uint8_t doGetEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t* varValuePtr) {

    // Check that the event variable has been written
    uint8_t inUse = readFlash8((flashAddr_t) &eventTable[eventIndex].numVars);
    if (varIndex >= inUse) return CMDERR_NO_EV;

    // Read the value
    *varValuePtr = readFlash8((flashAddr_t) &eventTable[eventIndex].vars[varIndex]);
    return 0;
}

/**
 * Removes (deletes) an event.
 * 
 * @param eventIndex index of event to be removed.
 * @return 0 (success).
 * @post FLASH cache updated but not flushed.
 * 
 * @note flushFlashCache() and rebuildChainLinks() must be called to complete
 * the process.
 */
static uint8_t doRemoveEvent(uint8_t eventIndex) {

    // Erase variable from FLASH memory by writing 0xFFs
    fillFlashCached((flashAddr_t) &eventTable[eventIndex], 0xFF, EVENT_SIZE);

    return 0;
}

/**
 * Initialises variables.
 */
void initEvent() {

    rebuildChainLinks();
}

/**
 * Finds the index of an event in FLASH memory given its node number and event
 * number.
 * 
 * @param nodeNumber Node number.
 * @param eventNumber Event number.
 * @return Event index, or NO_INDEX if none free.
 */
uint8_t findEvent(uint16_t nodeNumber, uint16_t eventNumber) {

    // If cbusNodeNumber has changed since the chain links were built, they need
    // to be rebuilt because cbusNodeNumber can be used in generating hash
    if (myNodeNumber.value != cbusNodeNumber.value) rebuildChainLinks();

    // Hash the event ID we are looking for
    uint8_t hash = calculateHash((bytes16_t) nodeNumber, (bytes16_t) eventNumber);

    // Loop for all events in this chain
    uint8_t eventIndex = hashHead[hash];
    while (eventIndex != NO_INDEX) {

        // Get event
        fetchEventHeader(eventIndex);

        // Return index if event's ID matches what we are looking for
        if (event.nodeNumber.value == nodeNumber
                && event.eventNumber.value == eventNumber)
            return eventIndex;

        eventIndex = chainMap[eventIndex];
    }

    // Event not found
    return NO_INDEX;
}

/**
 * Adds an event to FLASH if it does not exist, and then sets the value of one
 * of its variables.
 * 
 * @param nodeNumber Node number.
 * @param eventNumber Event number.
 * @param varIndex Index of event variable to be written.
 * @param varValue event variable value to be written.
 * @param useMyNodeNumber Event flag.
 * @return 0 (success) if event added and event variable written; returns
 * CMDERR error value otherwise. 
 */
uint8_t addEvent(uint16_t nodeNumber, uint16_t eventNumber, uint8_t varIndex, uint8_t varValue, bool useMyNodeNumber) {

    // Check the validity of the event variable index
    uint8_t err = checkVarIndex(varIndex);
    if (err) return err;

    // Lookup the event
    uint8_t eventIndex = findEvent(nodeNumber, eventNumber);

    // If not found...
    if (eventIndex == NO_INDEX) {

        // Get a free event index
        eventIndex = getFreeIndex();
        if (eventIndex == NO_INDEX) return CMDERR_TOO_MANY_EVENTS;

        // Add the event to FLASH
        err = doAddEvent(eventIndex, nodeNumber, eventNumber, useMyNodeNumber);
    }

    // Write the event variable
    if (!err) err = doWriteEventVar(eventIndex, varIndex, varValue);

    // Clean up and finish off
    flushFlashCache();
    rebuildChainLinks();
    return err;
}

/**
 * Sets the value of an event variable.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param varValue Value to be written.
 * @return 0 (success) if event variable written; returns CMDERR error value
 * otherwise.
 */
uint8_t writeEventVar(uint8_t eventIndex, uint8_t varIndex, uint8_t varValue) {

    // Check the validity of the event and event variable indexes
    uint8_t err = checkEventIndex(eventIndex);
    if (!err) err = checkVarIndex(varIndex);
    if (err) return err;

    // Write the event variable
    err = doWriteEventVar(eventIndex, varIndex, varValue);

    // Clean up and finish off
    flushFlashCache();
    return err;
}

/**
 * Modifies the node number and event number of an event given its index, and
 * then sets the value of one of its variables.
 * 
 * @param eventIndex Index of event.
 * @param nodeNumber Updated node number.
 * @param eventNumber Updated event number.
 * @param varIndex Index of event variable.
 * @param varValue Value to be written.
 * @return 0 (success) if event updated and event variable written; returns
 * CMDERR error value otherwise.
 */
uint8_t updateEvent(uint8_t eventIndex, uint16_t nodeNumber, uint16_t eventNumber, uint8_t varIndex, uint8_t varValue) {

    // Check the validity of the event and event variable indexes
    uint8_t err = checkEventIndex(eventIndex);
    if (!err) err = checkVarIndex(varIndex);
    if (err) return err;

    // Lookup the supplied event ID
    uint8_t idx = findEvent(nodeNumber, eventNumber);

    // If the same index returned, then the eventIndex event already has the
    // supplied ID, so do nothing; however, if not...
    if (idx != eventIndex) {

        // If the supplied ID matches a different existing event, we can't make
        // the change requested
        if (idx != NO_INDEX) return CMDERR_INVALID_EVENT;

        // Update the unique ID (node number and event number)
        writeFlashCached16((flashAddr_t) &eventTable[eventIndex].nodeNumber, nodeNumber);
        writeFlashCached16((flashAddr_t) &eventTable[eventIndex].eventNumber, eventNumber);
    }

    // Write the event variable
    err = doWriteEventVar(eventIndex, varIndex, varValue);

    // Clean up and finish off
    flushFlashCache();
    rebuildChainLinks();
    return err;
}

/**
 * Gets the value of an event variable.
 * 
 * @param eventIndex Index of event.
 * @param varIndex Index of event variable.
 * @param[out] varValuePtr pointer to location where value is to be written.
 * @return 0 (success) if event variable is in use and has a value; returns
 * CMDERR error value otherwise.
 */
uint8_t getEventVarValue(uint8_t eventIndex, uint8_t varIndex, uint8_t* varValuePtr) {

    // Check the validity of the event and event variable indexes
    uint8_t err = checkEventIndex(eventIndex);
    if (!err) err = checkVarIndex(varIndex);
    if (err) return err;

    // Get and return the event variable value
    return doGetEventVar(eventIndex, varIndex, varValuePtr);
}

/**
 * Gets the number of event variables in use.
 * 
 * @param eventIndex Index of event.
 * @param[out] varCounrPtr pointer to location where count is to be written.
 * @return 0 (success) if event exists; returns CMDERR error value otherwise.
 */
uint8_t getEventVarCount(uint8_t eventIndex, uint8_t* varCountPtr) {

    // Check the validity of the event index
    uint8_t err = checkEventIndex(eventIndex);
    if (err) return err;

    // Get and return the number of event variables used
    *varCountPtr = readFlash8((flashAddr_t) &eventTable[eventIndex].numVars);
    return 0;
}

/**
 * Removes (deletes) an event.
 * 
 * @param eventIndex Index of event.
 * @return 0 (success) if event is removed; returns CMDERR error value
 * otherwise.
 */
uint8_t removeEvent(uint8_t eventIndex) {

    // Check the validity of the event index
    uint8_t err = checkEventIndex(eventIndex);
    if (err) return err;

    // Remove the event
    err = doRemoveEvent(eventIndex);

    // Clean up and finish off
    flushFlashCache();
    rebuildChainLinks();
    return err;
}

/**
 * Returns the number of defined events.
 * 
 * @return count of events.
 */
uint8_t countEvents() {

    uint8_t count = 0;
    for (uint8_t eventIndex = 0; eventIndex < MAX_NUM_EVENTS; ++eventIndex) {
        if (eventFlags[eventIndex].byte != 0xFF) ++count;
    }
    return count;
}

/**
 * Returns the number of free event entries available.
 * 
 * @return count of free event entries.
 */
uint8_t countEventSlotsFree() {

    uint8_t count = 0;
    for (uint8_t eventIndex = 0; eventIndex < MAX_NUM_EVENTS; ++eventIndex) {
        if (eventFlags[eventIndex].byte == 0xFF) ++count;
    }
    return count;
}

/**
 * Removes (deletes) all events.
 */
void removeAllEvents() {

    // Erase entire event table by writing 0xFFs.
    fillFlashCached((flashAddr_t) &eventTable, 0xFF, MAX_NUM_EVENTS * EVENT_SIZE);

    // Clean up and finish off
    flushFlashCache();
    rebuildChainLinks();
}


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">


/**
 * Builds an ENRSP opcode response for a specified event.
 * 
 * @param eventIndex Index of event.
 * @post cbusMsg[] ENRSP response.
 */
static void buildENRSP(uint8_t eventIndex) {

    fetchEventHeader(eventIndex);

    cbusMsg[0] = OPC_ENRSP;
    cbusMsg[3] = event.nodeNumber.valueH;
    cbusMsg[4] = event.nodeNumber.valueL;
    cbusMsg[5] = event.eventNumber.valueH;
    cbusMsg[6] = event.eventNumber.valueL;
    cbusMsg[7] = eventIndex + 1;
}

/**
 * A timedResponse function to send a series of ENRSP opcode responses, one for
 * each defined event.
 * 
 * @param call Call number (0 to numCalls-1).
 * @param numCalls Total number of calls scheduled.
 * @return 1: send response; 0: no response.
 * @post cbusMsg[] ENRSP response.
 */
static int8_t timedResponseENRSP(uint8_t call, uint8_t numCalls) {

    // Static variable for the last eventIndex used
    static uint8_t eventIndex = 0;

    // Initialise or update the index
    eventIndex = (call == 0) ? 0 : eventIndex + 1;

    // Loop for all indexes
    while (eventIndex < MAX_NUM_EVENTS) {

        // If an event, send the ENRSP response
        if (eventFlags[eventIndex].byte != 0xFF) {
            buildENRSP(eventIndex);
            return 1;
        }

        ++eventIndex;
    }

    // Run of of events, so cancel any further timedResponse calls
    cancelTimedResponse();
    return 0;
}

/**
 * Processes opcode 0x53 "Set node into learn mode".
 * 
 * @pre cbusMsg[] NNLRN message.
 * @return 0: no response.
 */
int8_t opCodeNNLRN() {

    if (cbusMsg[1] == cbusNodeNumber.valueH && cbusMsg[2] == cbusNodeNumber.valueL) {
        interactState = interactStateFlimLearn;
    } else if (interactState == interactStateFlimLearn) {
        interactState = interactStateFlim;
    }
    return 0;
}

/**
 * Processes opcode 0x54 "Release node from learn mode".
 * 
 * @pre cbusMsg[] NNULN message.
 * @return 0: no response.
 */
int8_t opCodeNNULN() {

    interactState = interactStateFlim;
    return 0;
}

/**
 * Processes opcode 0x55 "Clear all events from a node".
 * 
 * @pre cbusMsg[] NNCLR message.
 * @return 1: send response.
 * @post cbusMsg[] WRACK response.
 */
int8_t opCodeNNCLR() {

    removeAllEvents();
    cbusMsg[0] = OPC_WRACK;
    return 1;
}

/**
 * Processes opcode 0x56 "Read number of events available in a node".
 * 
 * @pre cbusMsg[] NNEVN message.
 * @return 1: send response.
 * @post cbusMsg[] EVNLF response.
 */
int8_t opCodeNNEVN() {

    cbusMsg[0] = OPC_EVNLF;
    cbusMsg[3] = countEventSlotsFree();
    return 1;
}

/**
 * Processes opcode 0x57 "Read back all stored events in a node".
 * 
 * @pre cbusMsg[] NERD message.
 * @return 0: no response.
 * 
 * @note ENRSP responses will be sent at regular intervals by timedResponse.
 */
int8_t opCodeNERD() {

    uint8_t count = countEvents();
    enqueueTimedResponse(timedResponseENRSP, count);
    return 0;
}

/**
 * Processes opcode 0x58 "Request to read number of stored events".
 * 
 * @pre cbusMsg[] RQEVN message.
 * @return 1: send response.
 * @post cbusMsg[] NUMEV response.
 */
int8_t opCodeRQEVN() {

    cbusMsg[0] = OPC_NUMEV;
    cbusMsg[3] = countEvents();
    return 1;
}

/**
 * Processes opcode 0x72 "Request read of stored events by event index".
 * 
 * @pre cbusMsg[] NENRD message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] ENRSP response.
 */
int8_t opCodeNENRD() {

    uint8_t eventIndex = cbusMsg[3] - 1;
    uint8_t err = checkEventIndex(eventIndex);
    if (err) return (int8_t) -err;
    buildENRSP(eventIndex);
    return 1;
}

/**
 * Processes opcode 0x95 "Unlearn an event in learn mode".
 * 
 * @pre cbusMsg[] EVULN message.
 * @return 0: no response; <0: send error response.
 */
int8_t opCodeEVULN() {

    bytes16_t nn, en;
    nn.valueH = cbusMsg[1];
    nn.valueL = cbusMsg[2];
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t eventIndex = findEvent(nn.value, en.value);
    if (eventIndex == NO_INDEX) return -CMDERR_INVALID_EVENT;
    uint8_t err = removeEvent(eventIndex);
    if (err) return (int8_t) -err;
//    cbusMsg[0] = OPC_WRACK;
//    return 1;
    return 0;   // Apparently we shouldn't acknowledge an unlearn
}

/**
 * Processes opcode 0x9C "Request for read of an event variable".
 * 
 * @pre cbusMsg[] REVAL message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] NEVAL response.
 */
int8_t opCodeREVAL() {

    uint8_t eventIndex = cbusMsg[3] - 1;
    uint8_t err = 0;
    if (cbusMsg[4] == 0) {
        err = getEventVarCount(eventIndex, &cbusMsg[5]);
    } else {
        err = getEventVarValue(eventIndex, cbusMsg[4] - 1, &cbusMsg[5]);
    }
    if (err) return (int8_t) -err;
    cbusMsg[0] = OPC_NEVAL;
    return 1;
}

/**
 * Processes opcode 0xB2 "Read event variable in learn mode".
 * 
 * @pre cbusMsg[] REQEV message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] EVANS response.
 */
int8_t opCodeREQEV() {

    bytes16_t nn, en;
    nn.valueH = cbusMsg[1];
    nn.valueL = cbusMsg[2];
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t eventIndex = findEvent(nn.value, en.value);
    if (eventIndex == NO_INDEX) return -CMDERR_INVALID_EVENT;
    uint8_t err;
    if (cbusMsg[5] == 0) {
        err = getEventVarCount(eventIndex, &cbusMsg[6]);
    } else {
        err = getEventVarValue(eventIndex, cbusMsg[5] - 1, &cbusMsg[6]);
    }
    if (err) return (int8_t) -err;
    cbusMsg[0] = OPC_EVANS;
    return 1;
}

/**
 * Processes opcode 0xD2 "Teach an event in learn mode".
 * 
 * @pre cbusMsg[] EVLRN message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] WRACK response.
 */
int8_t opCodeEVLRN() {

    if (cbusMsg[5] == 0) return -CMDERR_INV_EV_IDX;;
    bytes16_t nn, en;
    nn.valueH = cbusMsg[1];
    nn.valueL = cbusMsg[2];
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t err = addEvent(nn.value, en.value, cbusMsg[5] - 1, cbusMsg[6], false);
    if (err) return (int8_t) -err;
    cbusMsg[0] = OPC_WRACK;
    return 1;
}

/**
 * Processes opcode 0xF5 "Teach an event in learn mode using event indexing".
 * 
 * @pre cbusMsg[] EVLRNI message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] WRACK response.
 */
int8_t opCodeEVLRNI() {

    if (cbusMsg[5] == 0) return -CMDERR_INV_EV_IDX;;
    bytes16_t nn, en;
    nn.valueH = cbusMsg[1];
    nn.valueL = cbusMsg[2];
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t err = updateEvent(cbusMsg[5] - 1, nn.value, en.value,
            cbusMsg[6] - 1, cbusMsg[7]);
    if (err) return (int8_t) -err;
    cbusMsg[0] = OPC_WRACK;
    return 1;
}

/**
 * Processes opcodes 0x90 0x91 0xB0 0xB1 xD0 0xD1 0xF0 & 0xF1
 * "Accessory ON/OFF".
 * 
 * @pre cbusMsg[] ACON(n) or ACOF(n) message.
 * @return 0: no response.
 */
int8_t opCodeACOxn() {

    bytes16_t nn, en;
    nn.valueH = cbusMsg[1];
    nn.valueL = cbusMsg[2];
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t eventIndex = findEvent(nn.value, en.value);
    if (eventIndex != NO_INDEX) processCbusEvent(eventIndex);
    return 0;
}

/**
 * Processes opcodes 0x98 0x99 0xB8 0xB9 xD8 0xD9 0xF8 & 0xF9
 * "Accessory Short ON/OFF".
 * 
 * @pre cbusMsg[] ASON(n) or ASOF(n) message.
 * @return 0: no response.
 */
int8_t opCodeASOxn() {

    bytes16_t en;
    en.valueH = cbusMsg[3];
    en.valueL = cbusMsg[4];
    uint8_t eventIndex = findEvent(0, en.value);
    if (eventIndex != NO_INDEX) processCbusEvent(eventIndex);
    return 0;
}


// </editor-fold>
