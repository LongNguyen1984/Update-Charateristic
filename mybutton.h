/**********************************************************************************************
 * Filename:       myButton.h
 *
 * Description:    This file contains the myButton service definitions and
 *                 prototypes.
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


#ifndef _MYBUTTON_H_
#define _MYBUTTON_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define MYBUTTON_SERV_UUID 0xAA00

//  Characteristic defines
#define MYBUTTON_NUMBUTTON_ID   0
#define MYBUTTON_NUMBUTTON_UUID 0xAA01
#define MYBUTTON_NUMBUTTON_LEN  2

#define MYBUTTON_COUNTER_ID   1
#define MYBUTTON_COUNTER_UUID 0xAA02
#define MYBUTTON_COUNTER_LEN  2
#include "bcomdef.h"
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
//typedef void (*myButtonChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);
typedef void (*myButtonChange_t)(uint8_t paramID);

typedef struct
{
  myButtonChange_t        pfnChangeCb;  // Called when characteristic value changes
  //myButtonChange_t        pfnCfgChangeCb;// Called when characteristic CCCD changes
} myButtonCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * MyButton_AddService- Initializes the MyButton service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyButton_AddService( uint8_t rspTaskId);

/*
 * MyButton_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t MyButton_RegisterAppCBs( myButtonCBs_t *appCallbacks );

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
extern bStatus_t MyButton_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * MyButton_GetParameter - Get a MyButton parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MyButton_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MYBUTTON_H_ */
