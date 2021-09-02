/******************************************************************************

 @file       scanparamservice.c

 @brief This file contains the Scan Parameters Service.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "scanparamservice.h"
#include "ti_ble_config.h"

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
// Scan parameters service
CONST uint8 scanParamServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_PARAM_SERV_UUID), HI_UINT16(SCAN_PARAM_SERV_UUID)
};

// Scan interval window characteristic
CONST uint8 scanIntervalWindowUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_INTERVAL_WINDOW_UUID), HI_UINT16(SCAN_INTERVAL_WINDOW_UUID)
};

// Scan parameter refresh characteristic
CONST uint8 scanParamRefreshUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SCAN_REFRESH_UUID), HI_UINT16(SCAN_REFRESH_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Application callback
static scanParamServiceCB_t scanParamServiceCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// Scan Parameters Service attribute
static CONST gattAttrType_t scanParamService = { ATT_BT_UUID_SIZE, scanParamServUUID };

// Scan Interval Window characteristic
static uint8 scanIntervalWindowProps = GATT_PROP_WRITE_NO_RSP;
static uint8 scanIntervalWindow[SCAN_INTERVAL_WINDOW_CHAR_LEN];

// Scan Parameter Refresh characteristic
static uint8 scanParamRefreshProps = GATT_PROP_NOTIFY;
static uint8 scanParamRefresh[SCAN_PARAM_REFRESH_LEN];
static gattCharCfg_t *scanParamRefreshClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t scanParamAttrTbl[] =
{
  // Scan Parameters Service attribute
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&scanParamService                /* pValue */
  },

    // Scan Interval Window declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &scanIntervalWindowProps
    },

      // Scan Interval Window characteristic
      {
        { ATT_BT_UUID_SIZE, scanIntervalWindowUUID },
        GATT_PERMIT_ENCRYPT_WRITE,
        0,
        scanIntervalWindow
      },

    // Scan Parameter Refresh declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &scanParamRefreshProps
    },

    // Scan Parameter Refresh characteristic
      {
        { ATT_BT_UUID_SIZE, scanParamRefreshUUID },
        0,
        0,
        scanParamRefresh
      },

      // Scan Parameter Refresh characteristic client characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8 *) &scanParamRefreshClientCharCfg
      }
};

// Attribute index enumeration-- these indexes match array elements above
enum
{
  SCAN_PARAM_SERVICE_IDX,           // Scan Parameters Service
  SCAN_PARAM_INTERVAL_DECL_IDX,     // Scan Interval Window declaration
  SCAN_PARAM_INTERVAL_IDX,          // Scan Interval Window characteristic
  SCAN_PARAM_REFRESH_DECL_IDX,      // Scan Parameter Refresh declaration
  SCAN_PARAM_REFRESH_IDX,           // Scan Parameter Refresh characteristic
  SCAN_PARAM_REFRESH_CCCD_IDX       // Scan Parameter Refresh characteristic
                                    // client characteristic configuration
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t scanParamReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t *pLen,
                                     uint16_t offset, uint16_t maxLen,
                                     uint8_t method);
static bStatus_t scanParamWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t len,
                                      uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t scanParamCBs =
{
  scanParamReadAttrCB,  // Read callback function pointer
  scanParamWriteAttrCB, // Write callback function pointer
  NULL                  // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ScanParam_AddService
 *
 * @brief   Initializes the Battery Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t ScanParam_AddService(void)
{
  uint8 status = SUCCESS;

  scanParamRefreshClientCharCfg = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                                linkDBNumConns);

  if (scanParamRefreshClientCharCfg == NULL)
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID, scanParamRefreshClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(scanParamAttrTbl,
                                       GATT_NUM_ATTRS(scanParamAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &scanParamCBs);

  return (status);
}

/*********************************************************************
 * @fn      ScanParam_Register
 *
 * @brief   Register a callback function with the Battery Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern void ScanParam_Register(scanParamServiceCB_t pfnServiceCB)
{
  scanParamServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn      ScanParam_SetParameter
 *
 * @brief   Set a Battery Service parameter.
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
bStatus_t ScanParam_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      ScanParam_GetParameter
 *
 * @brief   Get a Battery Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ScanParam_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case SCAN_PARAM_PARAM_INTERVAL:
      *((uint16*)value) = BUILD_UINT16(scanIntervalWindow[0],
                                       scanIntervalWindow[1]);
      break;

    case SCAN_PARAM_PARAM_WINDOW:
      *((uint16*)value) = BUILD_UINT16(scanIntervalWindow[2],
                                       scanIntervalWindow[3]);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      ScanParam_RefreshNotify
 *
 * @brief   Notify the peer to refresh the scan parameters.
 *
 * @param   connHandle - connection handle
 *
 * @return  None
 */
void ScanParam_RefreshNotify(uint16 connHandle)
{
  uint16 value;

  value  = GATTServApp_ReadCharCfg(connHandle, scanParamRefreshClientCharCfg);
  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
                                SCAN_PARAM_REFRESH_LEN, NULL);
    if (noti.pValue != NULL)
    {
      // send notification
      noti.handle = scanParamAttrTbl[SCAN_PARAM_REFRESH_CCCD_IDX].handle;
      noti.len = SCAN_PARAM_REFRESH_LEN;
      noti.pValue[0] = SCAN_PARAM_REFRESH_REQ;

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      }
    }
  }
}

/*********************************************************************
 * @fn          scanParamReadAttrCB
 *
 * @brief       GATT read callback.
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
static bStatus_t scanParamReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t *pLen,
                                     uint16_t offset, uint16_t maxLen,
                                     uint8_t method)
{
  bStatus_t status = SUCCESS;

  return (status);
}

/*********************************************************************
 * @fn      scanParamWriteAttrCB
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
static bStatus_t scanParamWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t len,
                                      uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // Only one writeable attribute
  if (uuid == SCAN_INTERVAL_WINDOW_UUID)
  {
    if (len == SCAN_INTERVAL_WINDOW_CHAR_LEN)
    {
      uint16 interval = BUILD_UINT16(pValue[0], pValue[1]);
      uint16 window = BUILD_UINT16(pValue[0], pValue[1]);

      // Validate values
      if (window <= interval)
      {
        memcpy(pAttr->pValue, pValue, len);

        (*scanParamServiceCB)(SCAN_INTERVAL_WINDOW_SET);
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }
  else if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
  {
    status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return (status);
}

