/**
 * @file CbusLibXC8 opcode.c
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
 * Code and data associated with processing opcodes in incoming and outgoing
 * CBUS messages.
 * 
 * @author Konrad Orlowski
 * @date August 2022
 * 
 * [ Useful article about jump tables and arrays of pointers to functions:
 * https://barrgroup.com/embedded-systems/how-to/c-function-pointers ]
 */


#include "opcode.h"
#include "module.h"


// Flags defining the checks to be performed in incoming messages, and actions
// to me performed on outgoing messages.

#define V_CheckNN 0     // Incoming: validate cbusMsg[1..2] is our node number
#define V_AddNN 1       // Outgoing: put our node number into cbusMsg[1..2]
#define V_Setup 2       // Incoming: validate interact state is 'FLiM setup'
#define V_Learn 3       // Incoming: validate interact state is 'FLiM learn'

#define M_CheckNN (1 << V_CheckNN)
#define M_AddNN (1 << V_AddNN)
#define M_Setup (1 << V_Setup)
#define M_Learn (1 << V_Learn)


// extern
bytes16_t cbusNodeNumber;   // Our node number (or 0 if in SLiM mode)


/**
 * CBUS opcode lookup table.  Stores address of opcode function handler for
 * incoming messages, and flags for both incoming and outgoing messages.
 * 
 * Whilst the table is quite sparse, storing only the non-zero entries and using
 * binary split to lookup opcodes resulted in only modest space saving, but a
 * lot more processing time.
 */

typedef struct {
    int8_t (* func)(void);
    uint8_t flags;
} op_t;

const op_t ops[] = {
    {/* 0x00 */ NULL, 0},
    {/* 0x01 */ NULL, 0},
    {/* 0x02 */ NULL, 0},
    {/* 0x03 */ NULL, 0},
    {/* 0x04 */ NULL, 0},
    {/* 0x05 */ NULL, 0},
    {/* 0x06 */ NULL, 0},
    {/* 0x07 */ NULL, 0},
    {/* 0x08 */ NULL, 0},
    {/* 0x09 */ NULL, 0},
    {/* 0x0A */ NULL, 0},
    {/* 0x0B */ NULL, 0},
    {/* 0x0C */ NULL, 0},
    {/* 0x0D */ opCodeQNN, 0},                      // Query node number
    {/* 0x0E */ NULL, 0},
    {/* 0x0F */ NULL, 0},
    {/* 0x10 */ opCodeRQNP, M_Setup},               // Request node parameters
    {/* 0x11 */ opCodeRQMN, M_Setup},               // Request module name
    {/* 0x12 */ NULL, 0},
    {/* 0x13 */ NULL, 0},
    {/* 0x14 */ NULL, 0},
    {/* 0x15 */ NULL, 0},
    {/* 0x16 */ NULL, 0},
    {/* 0x17 */ NULL, 0},
    {/* 0x18 */ NULL, 0},
    {/* 0x19 */ NULL, 0},
    {/* 0x1A */ NULL, 0},
    {/* 0x1B */ NULL, 0},
    {/* 0x1C */ NULL, 0},
    {/* 0x1D */ NULL, 0},
    {/* 0x1E */ NULL, 0},
    {/* 0x1F */ NULL, 0},
    {/* 0x20 */ NULL, 0},
    {/* 0x21 */ NULL, 0},
    {/* 0x22 */ NULL, 0},
    {/* 0x23 */ NULL, 0},
    {/* 0x24 */ NULL, 0},
    {/* 0x25 */ NULL, 0},
    {/* 0x26 */ NULL, 0},
    {/* 0x27 */ NULL, 0},
    {/* 0x28 */ NULL, 0},
    {/* 0x29 */ NULL, 0},
    {/* 0x2A */ NULL, 0},
    {/* 0x2B */ NULL, 0},
    {/* 0x2C */ NULL, 0},
    {/* 0x2D */ NULL, 0},
    {/* 0x2E */ NULL, 0},
    {/* 0x2F */ NULL, 0},
    {/* 0x30 */ NULL, 0},
    {/* 0x31 */ NULL, 0},
    {/* 0x32 */ NULL, 0},
    {/* 0x33 */ NULL, 0},
    {/* 0x34 */ NULL, 0},
    {/* 0x35 */ NULL, 0},
    {/* 0x36 */ NULL, 0},
    {/* 0x37 */ NULL, 0},
    {/* 0x38 */ NULL, 0},
    {/* 0x39 */ NULL, 0},
    {/* 0x3A */ NULL, 0},
    {/* 0x3B */ NULL, 0},
    {/* 0x3C */ NULL, 0},
    {/* 0x3D */ NULL, 0},
    {/* 0x3E */ NULL, 0},
    {/* 0x3F */ NULL, 0},
    {/* 0x40 */ NULL, 0},
    {/* 0x41 */ NULL, 0},
    {/* 0x42 */ opCodeSNN, M_Setup},                // Set Node Number
    {/* 0x43 */ NULL, 0},
    {/* 0x44 */ NULL, 0},
    {/* 0x45 */ NULL, 0},
    {/* 0x46 */ NULL, 0},
    {/* 0x47 */ NULL, 0},
    {/* 0x48 */ NULL, 0},
    {/* 0x49 */ NULL, 0},
    {/* 0x4A */ NULL, 0},
    {/* 0x4B */ NULL, 0},
    {/* 0x4C */ NULL, 0},
    {/* 0x4D */ NULL, 0},
    {/* 0x4E */ NULL, 0},
    {/* 0x4F */ opCodeNNRSM, M_CheckNN},            // Reset to manufacturers defaults
    {/* 0x50 */ NULL /* RQNN */, M_AddNN},          // Request node number
    {/* 0x51 */ NULL /* NNREL */, M_AddNN},         // Node number release
    {/* 0x52 */ NULL /* NNACK */, M_AddNN},         // Node number acknowledge
    {/* 0x53 */ opCodeNNLRN, 0},                    // Set node into learn mode
    {/* 0x54 */ opCodeNNULN, M_CheckNN | M_Learn},  // Release node from learn mode
    {/* 0x55 */ opCodeNNCLR, M_CheckNN | M_Learn},  // Clear all events from a node
    {/* 0x56 */ opCodeNNEVN, M_CheckNN},            // Read number of events available in a node
    {/* 0x57 */ opCodeNERD, M_CheckNN},             // Read back all stored events in a node
    {/* 0x58 */ opCodeRQEVN, M_CheckNN},            // Request to read number of stored events
    {/* 0x59 */ NULL /* WRACK */, M_AddNN} ,        // Write acknowledge
    {/* 0x5A */ NULL, 0},
    {/* 0x5B */ NULL, 0},
    {/* 0x5C */ opCodeBOOT, M_CheckNN},             // Put node into bootloader mode (aka BOOTM)
    {/* 0x5D */ opCodeENUM, M_CheckNN},             // Force a self enumeration cycle for use with CAN
    {/* 0x5E */ opCodeNNRST, M_CheckNN},            // Restart node
    {/* 0x5F */ NULL, 0},
    {/* 0x60 */ NULL, 0},
    {/* 0x61 */ NULL, 0},
    {/* 0x62 */ NULL, 0},
    {/* 0x63 */ NULL, 0},
    {/* 0x64 */ NULL, 0},
    {/* 0x65 */ NULL, 0},
    {/* 0x66 */ NULL, 0},
    {/* 0x67 */ NULL, 0},
    {/* 0x68 */ NULL, 0},
    {/* 0x69 */ NULL, 0},
    {/* 0x6A */ NULL, 0},
    {/* 0x6B */ NULL, 0},
    {/* 0x6C */ NULL, 0},
    {/* 0x6D */ NULL, 0},
    {/* 0x6E */ NULL, 0},
    {/* 0x6F */ NULL /* CMDERR */, M_AddNN},        // Error messages from nodes during configuration
    {/* 0x70 */ NULL /* EVNLF */, M_AddNN},         // Event space left reply from node
    {/* 0x71 */ opCodeNVRD, M_CheckNN},             // Request read of a node variable
    {/* 0x72 */ opCodeNENRD, M_CheckNN},            // Request read of stored events by event index
    {/* 0x73 */ opCodeRQNPN, M_CheckNN},            // Request read of a node parameter by index
    {/* 0x74 */ NULL /* NUMEV */, M_AddNN},         // Number of events stored in node
    {/* 0x75 */ opCodeCANID, M_CheckNN},            // Set a CAN_ID in existing FLiM node
    {/* 0x76 */ NULL, 0},
    {/* 0x77 */ NULL, 0},
    {/* 0x78 */ NULL, 0},
    {/* 0x79 */ NULL, 0},
    {/* 0x7A */ NULL, 0},
    {/* 0x7B */ NULL, 0},
    {/* 0x7C */ NULL, 0},
    {/* 0x7D */ NULL, 0},
    {/* 0x7E */ NULL, 0},
    {/* 0x7F */ NULL, 0},
    {/* 0x80 */ NULL, 0},
    {/* 0x81 */ NULL, 0},
    {/* 0x82 */ NULL, 0},
    {/* 0x83 */ NULL, 0},
    {/* 0x84 */ NULL, 0},
    {/* 0x85 */ NULL, 0},
    {/* 0x86 */ NULL, 0},
    {/* 0x87 */ NULL, 0},
    {/* 0x88 */ NULL, 0},
    {/* 0x89 */ NULL, 0},
    {/* 0x8A */ NULL, 0},
    {/* 0x8B */ NULL, 0},
    {/* 0x8C */ NULL, 0},
    {/* 0x8D */ NULL, 0},
    {/* 0x8E */ NULL, 0},
    {/* 0x8F */ NULL, 0},
    {/* 0x90 */ opCodeACOxn, M_CheckNN},            // Accessory ON
    {/* 0x91 */ opCodeACOxn, M_CheckNN},            // Accessory OFF
    {/* 0x92 */ NULL, 0},
    {/* 0x93 */ NULL, 0},
    {/* 0x94 */ NULL, 0},
    {/* 0x95 */ opCodeEVULN, M_Learn},              // Unlearn an event in learn mode
    {/* 0x96 */ opCodeNVSET, M_CheckNN},            // Set a node variable
    {/* 0x97 */ NULL /* NVANS */, M_AddNN},         // Response to a request for a node variable value
    {/* 0x98 */ opCodeASOxn, 0},                    // Accessory Short ON
    {/* 0x99 */ opCodeASOxn, 0},                    // Accessory Short OFF
    {/* 0x9A */ NULL, 0},
    {/* 0x9B */ NULL /* PARAN*/, M_AddNN},          // Response to request for individual node parameter
    {/* 0x9C */ opCodeREVAL, M_CheckNN},            // Request for read of an event variable
    {/* 0x9D */ NULL, 0},
    {/* 0x9E */ NULL, 0},
    {/* 0x9F */ NULL, 0},
    {/* 0xA0 */ NULL, 0},
    {/* 0xA1 */ NULL, 0},
    {/* 0xA2 */ NULL, 0},
    {/* 0xA3 */ NULL, 0},
    {/* 0xA4 */ NULL, 0},
    {/* 0xA5 */ NULL, 0},
    {/* 0xA6 */ NULL, 0},
    {/* 0xA7 */ NULL, 0},
    {/* 0xA8 */ NULL, 0},
    {/* 0xA9 */ NULL, 0},
    {/* 0xAA */ NULL, 0},
    {/* 0xAB */ NULL, 0},
    {/* 0xAC */ NULL, 0},
    {/* 0xAD */ NULL, 0},
    {/* 0xAE */ NULL, 0},
    {/* 0xAF */ NULL, 0},
    {/* 0xB0 */ opCodeACOxn, M_CheckNN},            // Accessory ON
    {/* 0xB1 */ opCodeACOxn, M_CheckNN},            // Accessory OFF
    {/* 0xB2 */ opCodeREQEV, M_Learn},              // Read event variable in learn mode
    {/* 0xB3 */ NULL, 0},
    {/* 0xB4 */ NULL, 0},
    {/* 0xB5 */ NULL /* NEVAL */, M_AddNN},         // Response to request for read of EV value
    {/* 0xB6 */ NULL /* PNN */, M_AddNN},           // Response to Query Node
    {/* 0xB7 */ NULL, 0},
    {/* 0xB8 */ opCodeASOxn, 0},                    // Accessory Short ON
    {/* 0xB9 */ opCodeASOxn, 0},                    // Accessory Short OFF
    {/* 0xBA */ NULL, 0},
    {/* 0xBB */ NULL, 0},
    {/* 0xBC */ NULL, 0},
    {/* 0xBD */ NULL, 0},
    {/* 0xBE */ NULL, 0},
    {/* 0xBF */ NULL, 0},
    {/* 0xC0 */ NULL, 0},
    {/* 0xC1 */ NULL, 0},
    {/* 0xC2 */ NULL, 0},
    {/* 0xC3 */ NULL, 0},
    {/* 0xC4 */ NULL, 0},
    {/* 0xC5 */ NULL, 0},
    {/* 0xC6 */ NULL, 0},
    {/* 0xC7 */ NULL, 0},
    {/* 0xC8 */ NULL, 0},
    {/* 0xC9 */ NULL, 0},
    {/* 0xCA */ NULL, 0},
    {/* 0xCB */ NULL, 0},
    {/* 0xCC */ NULL, 0},
    {/* 0xCD */ NULL, 0},
    {/* 0xCE */ NULL, 0},
    {/* 0xCF */ NULL, 0},
    {/* 0xD0 */ opCodeACOxn, M_CheckNN},            // Accessory ON
    {/* 0xD1 */ opCodeACOxn, M_CheckNN},            // Accessory OFF
    {/* 0xD2 */ opCodeEVLRN, M_Learn},              // Teach an event in learn mode
    {/* 0xD3 */ NULL /* EVANS */, M_AddNN},         // Response to a request for an EV value in learn mode
    {/* 0xD4 */ NULL, 0},
    {/* 0xD5 */ NULL, 0},
    {/* 0xD6 */ NULL, 0},
    {/* 0xD7 */ NULL, 0},
    {/* 0xD8 */ opCodeASOxn, 0},                    // Accessory Short ON
    {/* 0xD9 */ opCodeASOxn, 0},                    // Accessory Short OFF
    {/* 0xDA */ NULL, 0},
    {/* 0xDB */ NULL, 0},
    {/* 0xDC */ NULL, 0},
    {/* 0xDD */ NULL, 0},
    {/* 0xDE */ NULL, 0},
    {/* 0xDF */ NULL, 0},
    {/* 0xE0 */ NULL, 0},
    {/* 0xE1 */ NULL, 0},
    {/* 0xE2 */ NULL /* NAME */, 0},                // Response to request for node name string
    {/* 0xE3 */ NULL, 0},
    {/* 0xE4 */ NULL, 0},
    {/* 0xE5 */ NULL, 0},
    {/* 0xE6 */ NULL, 0},
    {/* 0xE7 */ NULL, 0},
    {/* 0xE8 */ NULL, 0},
    {/* 0xE9 */ NULL, 0},
    {/* 0xEA */ NULL, 0},
    {/* 0xEB */ NULL, 0},
    {/* 0xEC */ NULL, 0},
    {/* 0xED */ NULL, 0},
    {/* 0xEE */ NULL, 0},
    {/* 0xEF */ NULL /* PARAMS */, 0},              // Response to request for node parameters
    {/* 0xF0 */ opCodeACOxn, M_CheckNN},            // Accessory ON
    {/* 0xF1 */ opCodeACOxn, M_CheckNN},            // Accessory OFF
    {/* 0xF2 */ NULL /* ENRSP */, M_AddNN},         // Response to request to read node events
    {/* 0xF3 */ NULL, 0},
    {/* 0xF4 */ NULL, 0},
    {/* 0xF5 */ opCodeEVLRNI, M_Learn},             // Teach an event in learn mode using event indexing
    {/* 0xF6 */ NULL /* ACDAT */, M_AddNN},         // Accessory node data event (aka debug)
    {/* 0xF7 */ NULL, 0},
    {/* 0xF8 */ opCodeASOxn, 0},                    // Accessory Short ON
    {/* 0xF9 */ opCodeASOxn, 0},                    // Accessory Short OFF
    {/* 0xFA */ NULL, 0},
    {/* 0xFB */ NULL, 0},
    {/* 0xFC */ NULL, 0},
    {/* 0xFD */ NULL, 0},
    {/* 0xFE */ NULL, 0},
    {/* 0xFF */ NULL, 0}
};


/**
 * Executes an incoming CBUS message.
 * 
 * @pre cbusMsg[] Incoming message.
 * @return 1: send response; 0: no response; <0: send error response.
 * @post cbusMsg[] response.
 * 
 * Looks up the opcode in an incoming message, checks that we have a matching
 * node number and that the interact state is correct if required, then, if OK,
 * calls the opcode handing routine.
 */
int8_t executeOpCode() {

    // Lookup opcode
    op_t op = ops[cbusMsg[0]];
    if (op.func == NULL) return 0;      // Unknown or unsupported opcode

    // Perform checks
    if (op.flags & M_CheckNN) {
        if (cbusMsg[1] != cbusNodeNumber.valueH || cbusMsg[2] != cbusNodeNumber.valueL) return 0;
    }
    if (op.flags & M_Setup) {
        if (interactState != interactStateFlimSetup) return 0;
    }
    if (op.flags & M_Learn) {
        if (interactState != interactStateFlimLearn) return -CMDERR_NOT_LRN;
    }

    // Callback to application
    if (!supportedOpCode()) return 0;

    // Call opcode handling routine
    return (*(op.func))();
}

/**
 * Looks up the opcode in an outgoing CBUS message, and add our node number to
 * the message if required.
 * 
 * @pre cbusMsg[] Outgoing message.
 */
void opCodeAddNodeNumber() {

    // Add our node number if required
    if (ops[cbusMsg[0]].flags & M_AddNN) {
        cbusMsg[1] = cbusNodeNumber.valueH;
        cbusMsg[2] = cbusNodeNumber.valueL;
    }
}
