/**********************************************************************************************
 * Filename:       myButton.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "myButton.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// myButton Service UUID: 0xAA00
CONST uint8_t myButtonUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MYBUTTON_SERV_UUID), HI_UINT16(MYBUTTON_SERV_UUID)
};

// NumButton UUID: 0xAA01
CONST uint8_t myButton_NumButtonUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MYBUTTON_NUMBUTTON_UUID), HI_UINT16(MYBUTTON_NUMBUTTON_UUID)
};

// Counter UUID: 0xAA02
CONST uint8_t myButton_CounterUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MYBUTTON_COUNTER_UUID), HI_UINT16(MYBUTTON_COUNTER_UUID)
};
/*********************************************************************
 * LOCAL VARIABLES
 */

static myButtonCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t myButtonDecl = { ATT_BT_UUID_SIZE, myButtonUUID };

// Characteristic "NumButton" Properties (for declaration)
static uint8_t myButton_NumButtonProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "NumButton" Value variable
static uint8_t myButton_NumButtonVal[MYBUTTON_NUMBUTTON_LEN] = {0};

// Characteristic "NumButton" CCCD
static gattCharCfg_t *myButton_NumButtonConfig;

// Characteristic "NumButton" User Description
static uint8 myButtonUserDesp[17] = "Button 0 Counter";

// Characteristic "NumButton" Properties (for declaration)
static uint8_t myButton_CounterProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "NumButton" Value variable
static uint8_t myButton_CounterVal[MYBUTTON_COUNTER_LEN] = {0};

// Characteristic "NumButton" CCCD
static gattCharCfg_t *myButton_CounterConfig;

// Characteristic "NumButton" User Description
static uint8 counterUserDesp[14] = "Timing Counter";
/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t myButtonAttrTbl[] =
{
  // myButton Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&myButtonDecl
  },
    // NumButton Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myButton_NumButtonProps
    },
      // NumButton Characteristic Value
      {
        { ATT_BT_UUID_SIZE, myButton_NumButtonUUID },
        GATT_PERMIT_READ,
        0,
        myButton_NumButtonVal
      },

      // NumButton CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&myButton_NumButtonConfig
      },
      // "myButton" User Description
            {
                 { ATT_BT_UUID_SIZE, charUserDescUUID },
                    GATT_PERMIT_READ,
                    0,
                    myButtonUserDesp
            },
            // Counter Characteristic Declaration
                {
                  { ATT_BT_UUID_SIZE, characterUUID },
                  GATT_PERMIT_READ,
                  0,
                  &myButton_CounterProps
                },
                  // Counter Characteristic Value
                  {
                    { ATT_BT_UUID_SIZE, myButton_CounterUUID },
                    GATT_PERMIT_READ,
                    0,
                    myButton_CounterVal
                  },

                  // Counter CCCD
                  {
                    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                    GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                    0,
                    (uint8 *)&myButton_CounterConfig
                  },
                  // Counter User Description
                        {
                             { ATT_BT_UUID_SIZE, charUserDescUUID },
                                GATT_PERMIT_READ,
                                0,
                                counterUserDesp
                        },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t myButton_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t myButton_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t myButtonCBs =
{
  myButton_ReadAttrCB,  // Read callback function pointer
  myButton_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * MyButton_AddService- Initializes the MyButton service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyButton_AddService(uint8_t rspTaskId ) //extern
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  myButton_NumButtonConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( myButton_NumButtonConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, myButton_NumButtonConfig ); //LINKDB_CONNHANDLE_INVALID

  myButton_CounterConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
    if ( myButton_CounterConfig == NULL )
    {
      return ( bleMemAllocError );
    }
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, myButton_CounterConfig ); //LINKDB_CONNHANDLE_INVALID
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( myButtonAttrTbl,
                                        GATT_NUM_ATTRS( myButtonAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &myButtonCBs );

  return ( status );
}

/*
 * MyButton_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t MyButton_RegisterAppCBs( myButtonCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * MyButton_SetParameter - Set a MyButton parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MyButton_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MYBUTTON_NUMBUTTON_ID:
      if ( len == MYBUTTON_NUMBUTTON_LEN )
      {
        memcpy(myButton_NumButtonVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( myButton_NumButtonConfig, myButton_NumButtonVal, FALSE,//(uint8_t *)
                                    myButtonAttrTbl, GATT_NUM_ATTRS( myButtonAttrTbl ),
                                    INVALID_TASK_ID,  myButton_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case MYBUTTON_COUNTER_ID:
          if ( len == MYBUTTON_COUNTER_LEN )
          {
            memcpy(myButton_CounterVal, value, len);

            // Try to send notification.
            GATTServApp_ProcessCharCfg( myButton_CounterConfig, myButton_CounterVal, FALSE,//(uint8_t *)
                                        myButtonAttrTbl, GATT_NUM_ATTRS( myButtonAttrTbl ),
                                        INVALID_TASK_ID,  myButton_ReadAttrCB);
          }
          else
          {
            ret = bleInvalidRange;
          }
          break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * MyButton_GetParameter - Get a MyButton parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MyButton_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          myButton_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t myButton_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the NumButton Characteristic Value
if ( ! memcmp(pAttr->type.uuid, myButton_NumButtonUUID, pAttr->type.len) )
  {
    if ( offset > MYBUTTON_NUMBUTTON_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = 2;//MIN(maxLen, MYBUTTON_NUMBUTTON_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }

else if ( ! memcmp(pAttr->type.uuid, myButton_CounterUUID, pAttr->type.len) )
  {
    if ( offset > MYBUTTON_COUNTER_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = 2;//MIN(maxLen, MYBUTTON_NUMBUTTON_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      myButton_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t myButton_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb(paramID);//(connHandle, paramID, len, pValue); // Call app function from stack task context.

  return status;
}
