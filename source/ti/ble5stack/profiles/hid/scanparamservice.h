/******************************************************************************

 @file       scanparamservice.h

 @brief This file contains the Scan Parameters Service definitions and
        prototypes.

 Group: CMCU, SCS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2011-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

#ifndef SCANPARAMSERVICE_H
#define SCANPARAMSERVICE_H

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

// Scan Characteristic Lengths
#define SCAN_INTERVAL_WINDOW_CHAR_LEN     4
#define SCAN_PARAM_REFRESH_LEN            1

// Scan Parameter Refresh Values
#define SCAN_PARAM_REFRESH_REQ            0x00

// Callback events
#define SCAN_INTERVAL_WINDOW_SET          1

// Get/Set parameters
#define SCAN_PARAM_PARAM_INTERVAL         0
#define SCAN_PARAM_PARAM_WINDOW           1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Scan Parameters Service callback function
typedef void (*scanParamServiceCB_t)(uint8 event);

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      ScanParam_AddService
 *
 * @brief   Initializes the Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t ScanParam_AddService(void);

/*********************************************************************
 * @fn      ScanParam_Register
 *
 * @brief   Register a callback function with the Scan Parameters Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void ScanParam_Register(scanParamServiceCB_t pfnServiceCB);

/*********************************************************************
 * @fn      ScanParam_SetParameter
 *
 * @brief   Set a Scan Parameters Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t ScanParam_SetParameter(uint8 param, uint8 len, void *value);

/*********************************************************************
 * @fn      ScanParam_GetParameter
 *
 * @brief   Get a Scan Parameters Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t ScanParam_GetParameter(uint8 param, void *value);

/*********************************************************************
 * @fn      ScanParam_RefreshNotify
 *
 * @brief   Notify the peer to refresh the scan parameters.
 *
 * @param   connHandle - connection handle
 *
 * @return  None
 */
extern void ScanParam_RefreshNotify(uint16 connHandle);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SCANPARAMSERVICE_H */
