/**
 * @file CbusLibXC8 cancommon.h
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
 * @date November 2022
 */

#ifndef CANCOMMON_H
#define	CANCOMMON_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "global.h"


/**
  CAN Message Object data structure

  @Summary
    Defines the CAN Message Object data structure.

  @Description
    This Data structure is to implement a CAN FIFO message object.
*/
typedef union 
{
    uint8_t msgfields;
    struct
    {
        uint8_t idType:1;       // 1 bit (Standard Frame or Extended Frame)
        uint8_t frameType:1;    // 1 bit (Data Frame or RTR Frame)
        uint8_t dlc:4;          // 4 bit (No of data bytes a message frame contains)
        uint8_t formatType:1;   // 1 bit (CAN 2.0 Format or CAN_FD Format)
        uint8_t brs:1;          // 1 bit (Bit Rate Switch)
    };
} CAN_MSG_FIELD;

typedef struct 
{
    uint32_t msgId;          // 29 bit (SID: 11bit, EID:18bit)
    CAN_MSG_FIELD field;     // CAN TX/RX Message Object Control
    uint8_t *data;           // Pointer to message data
} CAN_MSG_OBJ;   

/**
  CAN Message Object Bit Rate Switch Selection Enumeration

  @Summary
    Defines the CAN message object bit rate switch selection enumeration.

  @Description
    This enumeration defines the CAN message object bit rate switch selection option.
*/
typedef enum 
{   
    CAN_NON_BRS_MODE    = 0,
    CAN_BRS_MODE        = 1     //Supported only in CAN FD mode
} CAN_MSG_OBJ_BRS_MODE;

/**
  CAN Message Object Identifier Selection Enumeration

  @Summary
    Defines the CAN message object identifier selection enumeration.

  @Description
    This enumeration defines the CAN message object identifier selection option.
*/
typedef enum 
{   
    CAN_FRAME_STD       = 0,
    CAN_FRAME_EXT       = 1,
} CAN_MSG_OBJ_ID_TYPE;

/**
  CAN Message Object Frame Type Selection Enumeration

  @Summary
    Defines the CAN message object frame type selection enumeration.

  @Description
    This enumeration defines the CAN message object frame type selection option.
*/
typedef enum 
{   
    CAN_FRAME_DATA      = 0,
    CAN_FRAME_RTR       = 1,
} CAN_MSG_OBJ_FRAME_TYPE;

/**
  CAN Message Object Format Type Selection Enumeration

  @Summary
    Defines the CAN message object format type selection enumeration.

  @Description
    This enumeration defines the CAN message object format type selection option.
*/
typedef enum 
{   
    CAN_2_0_FORMAT      = 0,
    CAN_FD_FORMAT       = 1     //Supported only in CAN FD mode
} CAN_MSG_OBJ_TYPE;


#ifdef	__cplusplus
}
#endif

#endif	/* CANCOMMON_H */
