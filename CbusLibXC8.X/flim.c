/**
 * @file CbusLibXC8 flim.c
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
 * Code and data associated with the FLiM parameter block, and with node
 * variables.
 * 
 * @author Konrad Orlowski
 * @date September 2022
 */


#include "flim.h"
#include "module.h"
#include "flash.h"


// Calculated checksum
#define PARAMETERS_CHECKSUM ( \
            MODULE_MANUFACTURER \
            + MINOR_VERSION \
            + MODULE_ID \
            + MAX_NUM_EVENTS \
            + EVENT_NUM_VARS \
            + NUM_NODE_VARS \
            + MAJOR_VERSION \
            + MODULE_FLAGS \
            + CPU_ID \
            + CBUS_BUS \
            + (APPLICATION_BASE_ADDRESS >> 8) + (APPLICATION_BASE_ADDRESS & 0xFF) \
            + CPUM_MICROCHIP \
            + BETA_VERSION \
            + sizeof(parameterBlock.flimParameters) \
            + (MODULE_TYPE_NAME_ADDRESS >> 8) + (MODULE_TYPE_NAME_ADDRESS & 0xFF) \
        )


// Parameter block, set at fixed FLASH address
const parameterBlock_t parameterBlock __at(PARAMETER_BLOCK_ADDRESS) = {
    {   // flimParameters
        MODULE_MANUFACTURER,
        MINOR_VERSION,
        MODULE_ID,
        MAX_NUM_EVENTS,
        EVENT_NUM_VARS,
        NUM_NODE_VARS,
        MAJOR_VERSION,
        MODULE_FLAGS,           // Static flags
        CPU_ID,
        CBUS_BUS,
        APPLICATION_BASE_ADDRESS,
        0x00000000,             // Device ID read a runtime
        CPU_MANUFACTURER,
        BETA_VERSION
    },
    {0, 0, 0, 0},   // spare
    {   // fcuParameters
        sizeof(parameterBlock.flimParameters),
        MODULE_TYPE_NAME_ADDRESS,
        PARAMETERS_CHECKSUM
    }
};

// Module name, set at fixed FLASH address
const char moduleTypeName[8] __at(MODULE_TYPE_NAME_ADDRESS) = MODULE_TYPE_NAME;

extern const uint8_t nodeVariables[NUM_NODE_VARS] __at(NODE_VAR_ADDRESS);


/**
 * Adds dynamic flags (FLiM mode and learn mode) to the static flags defined in
 * the parameter block.
 * 
 * @return Parameter flags.
 */
static uint8_t parameterFlags() {

    uint8_t flags = parameterBlock.flimParameters.moduleFlags;
    if (interactState == interactStateFlim) {
        flags |= PF_FLiM;
    } else if (interactState == interactStateFlimLearn) {
        flags |= PF_LRN;
    }
    return flags;
}


// <editor-fold defaultstate="expanded" desc="OpCode handling routines">


/**
 * Processes opcode 0x0D "Query node number".
 * 
 * @pre cbusMsg[] QNN message.
 * @return 1: send response.
 * @post cbusMsg[] PNN response.
 */
int8_t opCodeQNN() {

    if (cbusNodeNumber.value == 0) return 0;
    cbusMsg[0] = OPC_PNN;
    cbusMsg[3] = parameterBlock.flimParameters.manufacturer;
    cbusMsg[4] = parameterBlock.flimParameters.moduleID;
    cbusMsg[5] = parameterFlags();
    return 1;
}

/**
 * Processes opcode 0x10 "Request node parameters".
 * 
 * @pre cbusMsg[] RQNP message.
 * @return 1: send response.
 * @post cbusMsg[] PARAMS response.
 */
int8_t opCodeRQNP() {

    cbusMsg[0] = OPC_PARAMS;
    for (uint8_t param = 1; param <= 7; ++param) {
        cbusMsg[param] = parameterBlock.flimParameters.bytes[param - 1];
    }
    return 1;
}

/**
 * Processes opcode 0x11 "Request module name".
 * 
 * @pre cbusMsg[] RQMN message.
 * @return 1: send response.
 * @post cbusMsg[] NAME response.
 */
int8_t opCodeRQMN() {

    cbusMsg[0] = OPC_NAME;
    for (uint8_t param = 1; param <= 7; ++param) {
        cbusMsg[param] = moduleTypeName[param - 1];
    }
    return 1;
}

/**
 * Processes opcode 0x73 "Request read of a node parameter by index".
 * 
 * @pre cbusMsg[] RQNPN message.
 * @return 1: send response.
 * @post cbusMsg[] PARAN response.
 */
int8_t opCodeRQNPN() {

    uint8_t param = cbusMsg[3];
    if (param > sizeof(parameterBlock.flimParameters)) return -CMDERR_INV_PARAM_IDX;
    cbusMsg[0] = OPC_PARAN;
    if (param == 0) {
        // Reading parameter 0 returns parameter block size
        cbusMsg[4] = sizeof(parameterBlock.flimParameters);
    } else if (param == PAR_FLAGS) {
        cbusMsg[4] = parameterFlags();      // Static and dynamic flags
    } else if (param == PAR_CPUMID) {
        cbusMsg[4] = *(const uint8_t*)0x3FFFFE;  // Device ID byte 0
    } else if (param == PAR_CPUMID + 1) {
        cbusMsg[4] = *(const uint8_t*)0x3FFFFF;  // Device ID byte 1
    } else {
        // All other parameters read directly from the parameter block
        cbusMsg[4] = parameterBlock.flimParameters.bytes[param - 1];
    }
    return 1;
}

/**
 * Processes opcode 0x71 "Request read of a node variable".
 * 
 * @pre cbusMsg[] NVRD message.
 * @return 1: send response.
 * @post cbusMsg[] NVANS response.
 */
int8_t opCodeNVRD() {

    uint8_t nvIdx = cbusMsg[3] - 1;
    if (nvIdx >= NUM_NODE_VARS) return -CMDERR_INV_NV_IDX;
    cbusMsg[0] = OPC_NVANS;
    cbusMsg[4] = readFlashCached8((flashAddr_t) &nodeVariables[nvIdx]);
    return 1;
}

/**
 * Processes opcode 0x96 "Set a node variable".
 * 
 * @pre cbusMsg[] NVSET message.
 * @return 1: send response; <0: send error response.
 * @post cbusMsg[] WRACK response.
 */
// 0x96 Set a node variable
int8_t opCodeNVSET() {

    uint8_t nvIdx = cbusMsg[3] - 1;
    if (nvIdx >= NUM_NODE_VARS) return -CMDERR_INV_NV_IDX;
    uint8_t oldVal = readFlashCached8((flashAddr_t) &nodeVariables[nvIdx]);
    uint8_t newVal = cbusMsg[4];
    // Application callback to validate new value...
    if (validateNodeVar(nvIdx, oldVal, newVal)) {
        // Write the new value
        writeFlashCached8((flashAddr_t) &nodeVariables[nvIdx], newVal);
        flushFlashCache();
        // Application callback confirming value changed
        nodeVarChanged(nvIdx, oldVal, newVal);
    } else {
        return -CMDERR_INV_NV_VALUE;
    }
    cbusMsg[0] = OPC_WRACK;
    return 1;
}


// </editor-fold>
