/******************************************************************************

 @file  hiddev.c

 @brief This file contains the common HID Device profile for use with the
 CC2652 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2652

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
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include "profiles/hid/scanparamservice.h"
#include "profiles/hid/battservice.h"
#include "hiddev.h"
#include "ti_ble_config.h"


/*********************************************************************
 * MACROS
 */

// Battery measurement period in ms.
#define DEFAULT_BATT_PERIOD                   15000

// TRUE to run scan parameters refresh notify test.
#define DEFAULT_SCAN_PARAM_NOTIFY_TEST        TRUE

// The following defines cover the different settings needed for the
// different advertising states require by the HOGP specification.
// These states are described in-depth in the official HOGP specification.

// Advertising intervals (units of 625us, 160=100ms).
#define HID_INITIAL_ADV_INT_MIN               48
#define HID_INITIAL_ADV_INT_MAX               80
#define HID_HIGH_ADV_INT_MIN                  32
#define HID_HIGH_ADV_INT_MAX                  48
#define HID_LOW_ADV_INT_MIN                   1600
#define HID_LOW_ADV_INT_MAX                   1600

// Advertising timeouts in sec.
#define HID_INITIAL_ADV_TIMEOUT               60
#define HID_HIGH_ADV_TIMEOUT                  5
#define HID_LOW_ADV_TIMEOUT                   0

// Advertising states
#define HID_ADV_STATE_NONE                    0
#define HID_ADV_STATE_INITIAL                 1
#define HID_ADV_STATE_HIGH                    2
#define HID_ADV_STATE_LOW                     3

/*
 * Time in ms to delay after reconnection. This is in place so that various
 * OS's have a chance to receive and process HID reports after reconnection.
 */
#define HID_REPORT_READY_TIME                 1000

#define HID_STATE_CHANGE_EVT                  1
#define HID_BATT_SERVICE_EVT                  2
#define HID_PASSCODE_EVT                      3
#define HID_PAIR_STATE_EVT                    4
#define HID_ADV_EVT                           5
#define HID_READ_RPA_EVT                      6
#define HID_SEND_PARAM_UPDATE_EVT             7

// HID Service Task Events.
#define HID_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define HID_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define HID_BATT_PERIODIC_EVT                 Event_Id_00
#define HID_IDLE_EVT                          Event_Id_01
#define HID_SEND_REPORT_EVT                   Event_Id_02

#define HID_ALL_EVENTS                        (HID_ICALL_EVT         | \
                                               HID_QUEUE_EVT         | \
                                               HID_BATT_PERIODIC_EVT | \
                                               HID_IDLE_EVT          | \
                                               HID_SEND_REPORT_EVT)

#define reportQEmpty()                        (firstQIdx == lastQIdx)

#define HIDDEVICE_TASK_PRIORITY               3

// Modified stack size
#ifndef HIDDEVICE_TASK_STACK_SIZE
#define HIDDEVICE_TASK_STACK_SIZE             1024
#endif


/*********************************************************************
 * CONSTANTS
 */

#define HID_DEV_DATA_LEN                      9

#ifdef HID_DEV_RPT_QUEUE_LEN
#define HID_DEV_REPORT_Q_SIZE               (HID_DEV_RPT_QUEUE_LEN+1)
#else
#define HID_DEV_REPORT_Q_SIZE               (10+1)
#endif

// HID Auto Sync Allow List configuration parameter. This parameter should be
// set to FALSE if the HID Host (i.e., the Central device) uses a Resolvable
// Private Address (RPA). It should be set to TRUE, otherwise.
#ifndef HID_AUTO_SYNC_AL
#define HID_AUTO_SYNC_AL                    FALSE
#endif

// Spin if the expression is not true
#define HIDDEV_ASSERT(expr) if (!(expr)) HidDev_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Event passed from other profiles.
typedef struct
{
    uint8_t event;                // event type
    void *pData;               // pointer to message
} hidDevEvt_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} hidGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
    uint8_t event;
    uint8_t data[];
} hidClockEventData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} hidBattEvtData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} hidPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t deviceAddr[B_ADDR_LEN];
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} hidPasscodeData_t;

typedef struct
{
    uint8_t id;
    uint8_t type;
    uint8_t len;
    uint8_t data[HID_DEV_DATA_LEN];
} hidDevReport_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
} hidConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                        // Connection Handle
    hidClockEventData_t *pParamUpdateEventData;
    Clock_Struct *pUpdateClock;                      // pointer to clock struct
} hidConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */


// Connection status
bool isConnected = false;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages.
ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct battPerClock;
static Clock_Struct idleTimeoutClock;

// Queue object used for app messages.
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration.
Task_Struct hidDeviceTask;
Char hidDeviceTaskStack[HIDDEVICE_TASK_STACK_SIZE];

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// GAP State
static uint8_t hidDevGapState = HID_GAPSTATE_INIT;

// TRUE if connection is secure
static uint8_t hidDevConnSecure = FALSE;

// GAP connection handle
static uint16_t gapConnHandle;

// TRUE if pairing in progress
static uint8_t hidDevPairingStarted = FALSE;

// Status of last pairing
static uint8_t pairingStatus = SUCCESS;

// Pairing state
static uint8_t hidDevGapBondPairingState = HID_GAPBOND_PAIRING_STATE_NONE;

// HID Report Map Table
static hidRptMap_t *pHidDevRptTbl;
static uint8_t hidDevRptTblLen;

static hidDevCB_t *pHidDevCB;

static hidDevCfg_t *pHidDevCfg;

// Pending reports
static uint8_t firstQIdx = 0;
static uint8_t lastQIdx = 0;
static hidDevReport_t hidDevReportQ[HID_DEV_REPORT_Q_SIZE];

// Last report sent out
static hidDevReport_t lastReport = { 0 };

// Advertising handles
static uint8 advHandleInitial;
static uint8 advHandleHigh;
static uint8 advHandleLow;

// Advertising State
static uint8_t hidAdvState;

// State when HID reports are ready to be sent out
static volatile uint8_t hidDevReportReadyState = TRUE;

// Report ready delay clock
static Clock_Struct reportReadyClock;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass RPA read event ID to clock handler
hidClockEventData_t argRpaRead = { .event = HID_READ_RPA_EVT };

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };

// Per-handle connection info
static hidConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for queued param updates
static List_List paramUpdateList;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Task events and processing functions.
static void HidDev_init(void);
static void HidDev_taskFxn(UArg a0, UArg a1);
static void HidDev_processStackMsg(ICall_Hdr *pMsg);
static void HidDev_processAppMsg(hidDevEvt_t *pMsg);
static void HidDev_processGapMessage(gapEventHdr_t *pMsg);
static void HidDev_processGattMsg(gattMsgEvent_t *pMsg);
static void HidDev_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void HidDev_disconnected(void);
void HidDev_startAdvertising(void);
void HidDev_StopAdvertising(void);
static uint8_t HidDev_bondCount(void);
static void HidDev_clockHandler(UArg arg);
static uint8_t HidDev_enqueueMsg(uint16_t event, uint8_t *pData);
static void HidDev_updateRPA();
static uint8_t HidDev_addConn(uint16_t connHandle);
static uint8_t HidDev_getConnIndex(uint16_t connHandle);
static uint8_t HidDev_clearConnListEntry(uint16_t connHandle);
void HidDev_clearPendingParamUpdate(uint16_t connHandle);
static uint8_t HidDev_removeConn(uint16_t connHandle);
static void HidDev_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void HidDev_processParamUpdate(uint16_t connHandle);
// HID reports.
static hidRptMap_t* HidDev_reportByHandle(uint16_t handle);
static hidRptMap_t* HidDev_reportById(uint8_t id, uint8_t type);
static hidRptMap_t* HidDev_reportByCccdHandle(uint16_t handle);
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len,
                                 uint8_t *pData);
static hidDevReport_t* HidDev_dequeueReport(void);
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len,
                              uint8_t *pData);
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData);
static uint8_t HidDev_isbufset(uint8_t *buf, uint8_t val, uint8_t len);
// Pair state.
static void HidDev_pairStateCB(uint16_t connHandle, uint8_t state,
                               uint8_t status);
static void HidDev_processPairStateEvt(hidPairStateData_t *pPairData);
// Passcode.
static void HidDev_passcodeCB(uint8_t *pDeviceAddr, uint16_t connHandle,
                              uint8_t uiInputs, uint8_t uiOutputs,
                              uint32_t numComparison);
static void HidDev_processPasscode(hidPasscodeData_t *pPasscodeData);
// Battery events.
static void HidDev_batteryCB(uint8_t event);
static void HidDev_processBatteryEvt(hidBattEvtData_t* pData);
static void HidDev_battPeriodicTask(void);
// Scan parameter events.
static void HidDev_scanParamCB(uint8_t event);
// Process reconnection delay
static void HidDev_reportReadyClockCB(UArg a0);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static const gapBondCBs_t hidDevBondCB = { (pfnPasscodeCB_t) HidDev_passcodeCB,
                                           HidDev_pairStateCB };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidDev_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void HidDev_spin(void)
{
    volatile uint8_t x = 0;

    while (1)
    {
        x++;
    }
}

/*********************************************************************
 * @fn      HidDev_createTask
 *
 * @brief   Task creation function for the HID service.
 *
 * @param   none
 *
 * @return  none
 */
void HidDev_createTask(void)
{
    Task_Params taskParams;

    // Configure task.
    Task_Params_init(&taskParams);
    taskParams.stack = hidDeviceTaskStack;
    taskParams.stackSize = HIDDEVICE_TASK_STACK_SIZE;
    taskParams.priority = HIDDEVICE_TASK_PRIORITY;

    Task_construct(&hidDeviceTask, HidDev_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HidDev_init
 *
 * @brief   Initialization function for the Hid Dev Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
static void HidDev_init(void)
{
    hidAdvState = HID_ADV_STATE_NONE;
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);
    setBondManagerParameters();

    // Set MITM Protection
    uint8_t mitm = true;
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);

    uint8_t gapbondSecure = GAPBOND_SECURE_CONNECTION_NONE;
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &gapbondSecure);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&battPerClock, HidDev_clockHandler,DEFAULT_BATT_PERIOD,
                        0, false, HID_BATT_PERIODIC_EVT);

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Configuring default connection parameters
    // Send request to update connection params to HID host
    bStatus_t status = FAILURE;
    gapPeriConnectParams_t defaultConnectParams;
    defaultConnectParams.intervalMax = 0x0006;      // 7.5ms Connection interval
    defaultConnectParams.intervalMin = 0x0006;
    defaultConnectParams.latency = 0;
    defaultConnectParams.timeout = 500;
    status = GGS_SetParameter(GGS_PERI_CONN_PARAM_ATT, sizeof(defaultConnectParams), &defaultConnectParams);
    HIDDEV_ASSERT(status == SUCCESS);

    // Configure GAP
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
    // section in the User's Guide
    setBondManagerParameters();

    // Set up services.
    GGS_AddService(GATT_ALL_SERVICES);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    // Set up HID keyboard service
    DevInfo_AddService();
    Batt_AddService();
    ScanParam_AddService();
    Batt_Register(HidDev_batteryCB);

    // Register for Scan Parameters service callback.
    ScanParam_Register(HidDev_scanParamCB);

    // Register with bond manager after starting device.
    VOID GAPBondMgr_Register((gapBondCBs_t* )&hidDevBondCB);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the HCI section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Initialize GATT Client
    GATT_InitClient();

    // Initialize report ready clock timer
    Util_constructClock(&reportReadyClock, HidDev_reportReadyClockCB,
                        HID_REPORT_READY_TIME, 0, false, NULL);

    // Initialize Connection List
    HidDev_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

    // Initialize GAP layer for Peripheral role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

    // Setting up default PHY (1M)
    HCI_LE_SetDefaultPhyCmd(HCI_PHY_1_MBPS, HCI_PHY_1_MBPS , HCI_PHY_1_MBPS);
    HCI_LE_ReadLocalSupportedFeaturesCmd();

}

/*********************************************************************
 * @fn      HidDev_taskFxn
 *
 * @brief   Hid Dev Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used
 *
 * @return  not used
 */
static void HidDev_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    HidDev_init();

    bStatus_t status = FAILURE;

    // Application main loop.
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, HID_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void**) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    // Process inter-task message.
                    HidDev_processStackMsg((ICall_Hdr*) pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & HID_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    hidDevEvt_t *pMsg = (hidDevEvt_t*) Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message.
                        HidDev_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }

            // Idle timeout.
            if (events & HID_IDLE_EVT)
            {
                if (hidDevGapState == HID_GAPSTATE_CONNECTED)
                {
                    // If pairing in progress then restart timer.
                    if (hidDevPairingStarted)
                    {
                        HidDev_StartIdleTimer();
                    }
                    // Else disconnect and don't allow reports to be sent
                    else
                    {
                        hidDevReportReadyState = FALSE;
                        status = GAP_TerminateLinkReq(LINKDB_CONNHANDLE_ALL, HCI_DISCONNECT_REMOTE_USER_TERM);
                        HIDDEV_ASSERT(status == SUCCESS);
                    }
                }
            }

            // Battery periodic event.
            if (events & HID_BATT_PERIODIC_EVT)
            {
                HidDev_battPeriodicTask();
            }

            // Send HID report event.
            if (events & HID_SEND_REPORT_EVT)
            {
                // If connection is secure
                if (hidDevConnSecure && hidDevReportReadyState)
                {
                    hidDevReport_t *pReport = HidDev_dequeueReport();
                    if (pReport != NULL)
                    {
                        // Send report.
                        HidDev_sendReport(pReport->id, pReport->type,
                                          pReport->len, pReport->data);
                    }

                    // If there is another report in the queue
                    if (!reportQEmpty())
                    {
                        // Set another event.
                        Event_post(syncEvent, HID_SEND_REPORT_EVT);
                    }
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg         - Parameter configuration.
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void HidDev_Register(hidDevCfg_t *pCfg, hidDevCB_t *pCBs)
{
    pHidDevCB = pCBs;
    pHidDevCfg = pCfg;

    // If configured and not zero, create the idle timeout clock.
    if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout != 0))
    {
        Util_constructClock(&idleTimeoutClock, HidDev_clockHandler,
                            pHidDevCfg->idleTimeout, 0, false, HID_IDLE_EVT);
    }
}

/*********************************************************************
 * @fn      HidDev_RegisterReports
 *
 * @brief   Register the report table with HID Dev.
 *
 * @param   numReports - Length of report table.
 * @param   pRpt       - Report table.
 *
 * @return  None.
 */
void HidDev_RegisterReports(uint8_t numReports, hidRptMap_t *pRpt)
{
    pHidDevRptTbl = pRpt;
    hidDevRptTblLen = numReports;
}

/*********************************************************************
 * @fn      HidDev_CreateReport
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
void HidDev_createReport(uint8_t id, uint8_t type, uint8_t len, uint8_t *pData)
{
    // Validate length of report
    if (len > HID_DEV_DATA_LEN)
    {
        return;
    }
    // If connected
    if (hidDevGapState == HID_GAPSTATE_CONNECTED)
    {
        // If connection is secure
        if (hidDevConnSecure)
        {
            // Make sure there're no pending reports.
            if (reportQEmpty())
            {
                // Send report.
                HidDev_sendReport(id, type, len, pData);
                return;
            }
        }
    }
    // Start advertising if device is idle
    else if (hidDevGapState != HID_GAPSTATE_ADVERTISING)
    {
        HidDev_startAdvertising();
    }

    // HidDev task will send report when secure connection is established.
    HidDev_enqueueReport(id, type, len, pData);
}

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_Close(void)
{
    bStatus_t status = FAILURE;

    // If connected then disconnect.
    if (hidDevGapState == HID_GAPSTATE_CONNECTED)
    {
        status = GAP_TerminateLinkReq(LINKDB_CONNHANDLE_ALL, HCI_DISCONNECT_REMOTE_USER_TERM);
        HIDDEV_ASSERT(status == SUCCESS);
    }
    else
    {
        HidDev_StopAdvertising();
    }
}

/*********************************************************************
 * @fn      HidDev_SetParameter
 *
 * @brief   Set a HID Dev parameter.
 *
 * @param   param  - profile parameter ID
 * @param   len    - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_SetParameter(uint8_t param, uint8_t len, void *pValue)
{
    bStatus_t ret = SUCCESS;
    bStatus_t status = FAILURE;
    switch (param)
    {
        case HIDDEV_ERASE_ALLBONDS:
            if (len == 0)
            {
                hidRptMap_t *pRpt;

                // Get ATT handle for last report
                if ((pRpt = HidDev_reportById(lastReport.id, lastReport.type))
                        != NULL)
                {
                    // See if the last report sent out wasn't a release key
                    if (HidDev_isbufset(lastReport.data, 0x00, lastReport.len) == FALSE)
                    {
                        // Send a release report before disconnecting, otherwise
                        // the last pressed key would get 'stuck' on the HID Host.
                        memset(lastReport.data, 0x00, lastReport.len);

                        // Send report notification
                        VOID HidDev_sendNoti(pRpt->handle, lastReport.len, lastReport.data);
                    }

                    // Clear out last report
                    memset(&lastReport, 0, sizeof(hidDevReport_t));
                }

                // Terminate connection
                if (hidDevGapState == HID_GAPSTATE_CONNECTED)
                {
                    status = GAP_TerminateLinkReq(LINKDB_CONNHANDLE_ALL, HCI_DISCONNECT_REMOTE_USER_TERM);
                    HIDDEV_ASSERT(status == SUCCESS);
                }

                // Flush report queue.
                firstQIdx = lastQIdx = 0;

                // Erase bonding info.
                GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
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

    return (ret);
}

/*********************************************************************
 * @fn      HidDev_GetParameter
 *
 * @brief   Get a HID Dev parameter.
 *
 * @param   param  - profile parameter ID
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_GetParameter(uint8_t param, void *pValue)
{
    bStatus_t ret = SUCCESS;

    switch (param)
    {
        case HIDDEV_GAPROLE_STATE:
            *((uint8_t*) pValue) = hidDevGapState;
            break;

        case HIDDEV_GAPBOND_STATE:
            *((uint8_t*) pValue) = hidDevGapBondPairingState;
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }

    return (ret);
}

/*********************************************************************
 * @fn      HidDev_PasscodeRsp
 *
 * @brief   Respond to a passcode request.
 *
 * @param   status - SUCCESS if passcode is available, otherwise
 *                   see @ref SMP_PAIRING_FAILED_DEFINES.
 * @param   passcode - integer value containing the passcode.
 *
 * @return  none
 */
void HidDev_PasscodeRsp(uint8_t status, uint32_t passcode)
{
    // Send passcode response.
    GAPBondMgr_PasscodeRsp(gapConnHandle, status, passcode);
}

/*********************************************************************
 * @fn          HidDev_ReadAttrCB
 *
 * @brief       HID Dev attribute read callback.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr      - pointer to attribute
 * @param       pValue     - pointer to data to be read
 * @param       pLen       - length of data to be read
 * @param       offset     - offset of the first octet to be read
 * @param       maxLen     - maximum length of data to be read
 * @param       method     - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t HidDev_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                            uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                            uint16_t maxLen, uint8_t method)
{
    bStatus_t status = SUCCESS;
    hidRptMap_t *pRpt;

    // 16-bit UUID
    uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

    // Make sure it's not a blob operation (only report map is long)
    if (offset > 0 && uuid != REPORT_MAP_UUID)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if (uuid == REPORT_UUID || uuid == BOOT_KEY_INPUT_UUID
            || uuid == BOOT_KEY_OUTPUT_UUID || uuid == BOOT_MOUSE_INPUT_UUID)
    {
        // Find report ID in table.
        if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
        {
            // Execute report callback.
            status = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
            HID_DEV_OPER_READ, pLen, pValue);
        }
        else
        {
            *pLen = 0;
        }
    }
    else if (uuid == REPORT_MAP_UUID)
    {
        // If the value offset of the Read Blob Request is greater than the
        // length of the attribute value, an Error Response shall be sent with
        // the error code Invalid Offset.
        if (offset > hidReportMapLen)
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // Determine read length.
            *pLen = MIN(maxLen, (hidReportMapLen - offset));

            // Copy data.
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    }
    else if (uuid == HID_INFORMATION_UUID)
    {
        *pLen = HID_INFORMATION_LEN;
        memcpy(pValue, pAttr->pValue, HID_INFORMATION_LEN);
    }
    else if (uuid == GATT_REPORT_REF_UUID)
    {
        *pLen = HID_REPORT_REF_LEN;
        memcpy(pValue, pAttr->pValue, HID_REPORT_REF_LEN);
    }
    else if (uuid == PROTOCOL_MODE_UUID)
    {
        *pLen = HID_PROTOCOL_MODE_LEN;
        pValue[0] = pAttr->pValue[0];
    }
    else if (uuid == GATT_EXT_REPORT_REF_UUID)
    {
        *pLen = HID_EXT_REPORT_REF_LEN;
        memcpy(pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN);
    }

    // Restart idle timer.
    if (status == SUCCESS)
    {
        HidDev_StartIdleTimer();
    }

    return (status);
}

/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute write callback.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr      - pointer to attribute
 * @param   pValue     - pointer to data to be written
 * @param   len        - length of data
 * @param   offset     - offset of the first octet to be written
 * @param   method     - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t HidDev_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                             uint8_t *pValue, uint16_t len, uint16_t offset,
                             uint8_t method)
{
    bStatus_t status = SUCCESS;
    hidRptMap_t *pRpt;

    // Make sure it's not a blob operation (no attributes in the profile are long).
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

    if (uuid == REPORT_UUID || uuid == BOOT_KEY_OUTPUT_UUID)
    {
        // Find report ID in table.
        if ((pRpt = HidDev_reportByHandle(pAttr->handle)) != NULL)
        {
            // Execute report callback.
            status = (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
            HID_DEV_OPER_WRITE, &len, pValue);
        }
    }
    else if (uuid == HID_CTRL_PT_UUID)
    {
        // Validate length and value range.
        if (len == 1)
        {
            if (pValue[0] == HID_CMD_SUSPEND
                    || pValue[0] == HID_CMD_EXIT_SUSPEND)
            {
                // Execute HID app event callback.
                (*pHidDevCB->evtCB)((pValue[0] == HID_CMD_SUSPEND) ? HID_DEV_SUSPEND_EVT : HID_DEV_EXIT_SUSPEND_EVT);
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
        if (status == SUCCESS)
        {
            uint16_t charCfg = BUILD_UINT16(pValue[0], pValue[1]);

            // Find report ID in table.
            if ((pRpt = HidDev_reportByCccdHandle(pAttr->handle)) != NULL)
            {
                // Execute report callback.
                (*pHidDevCB->reportCB)(pRpt->id, pRpt->type, uuid,
                        (charCfg == GATT_CLIENT_CFG_NOTIFY) ? HID_DEV_OPER_ENABLE : HID_DEV_OPER_DISABLE,
                        &len, pValue);
            }
        }
    }
    else if (uuid == PROTOCOL_MODE_UUID)
    {
        if (len == HID_PROTOCOL_MODE_LEN)
        {
            if (pValue[0] == HID_PROTOCOL_MODE_BOOT
                    || pValue[0] == HID_PROTOCOL_MODE_REPORT)
            {
                pAttr->pValue[0] = pValue[0];

                // Execute HID app event callback.
                (*pHidDevCB->evtCB)(
                        (pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
                                HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT);
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

    // Restart idle timer.
    if (status == SUCCESS)
    {
        HidDev_StartIdleTimer();
    }

    return (status);
}

/*********************************************************************
 * @fn      HidDev_StartIdleTimer
 *
 * @brief   Start the idle timer.
 *
 * @return  None.
 */
void HidDev_StartIdleTimer(void)
{
    if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
    {
        Util_startClock(&idleTimeoutClock);
    }
}

/*********************************************************************
 * @fn      HidDev_StopIdleTimer
 *
 * @brief   Stop the idle timer.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StopIdleTimer(void)
{
    if ((pHidDevCfg != NULL) && (pHidDevCfg->idleTimeout > 0))
    {
        Util_stopClock(&idleTimeoutClock);
    }
}

/*********************************************************************
 * @fn      HidDev_StartAdvertising
 *
 * @brief   Start advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_startAdvertising(void)
{
    bStatus_t status = FAILURE;

    if (hidAdvState != HID_ADV_STATE_NONE)
    {
        HidDev_StopAdvertising();
    }

    // If previously bonded
    if (HidDev_bondCount() > 0)
    {
        // Start high duty cycle advertising.
        status = GapAdv_enable(advHandleHigh, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        HIDDEV_ASSERT(status == SUCCESS);
        hidAdvState = HID_ADV_STATE_HIGH;
    }
    // Else not bonded.
    else
    {
        // Start initial advertising.
        status = GapAdv_enable(advHandleInitial, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        HIDDEV_ASSERT(status == SUCCESS);
        hidAdvState = HID_ADV_STATE_INITIAL;
    }
}

/*********************************************************************
 * @fn      HidDev_StopAdvertising
 *
 * @brief   Stop advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
void HidDev_StopAdvertising(void)
{
    bStatus_t status = FAILURE;

    // If in initial advertising state
    if (hidAdvState == HID_ADV_STATE_INITIAL)
    {
        status = GapAdv_disable(advHandleInitial, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }
    // If in high rate advertising state
    else if (hidAdvState == HID_ADV_STATE_HIGH)
    {
        status = GapAdv_disable(advHandleHigh, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }
    // If in low rate advertising state
    else if (hidAdvState == HID_ADV_STATE_LOW)
    {
        status = GapAdv_disable(advHandleLow, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }

    HIDDEV_ASSERT(status == SUCCESS);
    hidAdvState = HID_ADV_STATE_NONE;
}

/*********************************************************************
 * @fn      HidDev_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidDev_processAppMsg(hidDevEvt_t *pMsg)
{
    switch (pMsg->event)
    {
        case HID_ADV_EVT:
            break;

        case HID_BATT_SERVICE_EVT:
            HidDev_processBatteryEvt((hidBattEvtData_t*)pMsg->pData);
            break;

        case HID_PAIR_STATE_EVT:
            HidDev_processPairStateEvt(pMsg->pData);
            ICall_free(pMsg->pData);
            break;

        case HID_PASSCODE_EVT:
            HidDev_processPasscode((hidPasscodeData_t*)(pMsg->pData));
            break;

        case HID_READ_RPA_EVT:
            HidDev_updateRPA();
            break;

        default:
            // Do nothing.
            break;
    }
}

/*********************************************************************
 * @fn      HidDev_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidDev_processStackMsg(ICall_Hdr *pMsg)
{
    switch (pMsg->event)
    {
        case GAP_MSG_EVENT:
            HidDev_processGapMessage((gapEventHdr_t*) pMsg);
            break;

        case GATT_MSG_EVENT:
            HidDev_processGattMsg((gattMsgEvent_t*) pMsg);
            break;

        case HCI_GAP_EVENT_EVENT:
        {
            // Process HCI message
            switch (pMsg->status)
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                    // Process HCI Command Complete Events here
                    {
                    HidDev_processCmdCompleteEvt((hciEvt_CmdComplete_t*) pMsg);
                    break;
                    }

                case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
                    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
                    break;

                default:
                    break;
            }
            break;
        }
        default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      HidDev_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void HidDev_processGattMsg(gattMsgEvent_t *pMsg)
{
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HidDev_processGapMsg
 *
 * @brief   Process GAP messages
 *
 * @return  none
 */
static void HidDev_processGapMessage(gapEventHdr_t *pMsg)
{
    bStatus_t status = FAILURE;
    switch (pMsg->opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;
            hidDevGapState = HID_GAPSTATE_INIT;
            if (pPkt->hdr.status == SUCCESS)
            {
                // Store the system ID
                uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

                // use 6 bytes of device address for 8 bytes of system ID value
                systemId[0] = pPkt->devAddr[0];
                systemId[1] = pPkt->devAddr[1];
                systemId[2] = pPkt->devAddr[2];

                // set middle bytes to zero
                systemId[4] = 0x00;
                systemId[3] = 0x00;

                // shift three bytes up
                systemId[7] = pPkt->devAddr[5];
                systemId[6] = pPkt->devAddr[4];
                systemId[5] = pPkt->devAddr[3];

                // Set Device Info Service Parameter
                DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                    systemId);

                // Setup and start Advertising
                // For more information, see the GAP section in the User's Guide:
                // http://software-dl.ti.com/lprf/ble5stack-latest/

                // Adverisement set #1, initial
                // Create Advertisement set #1 and assign handle
                status = GapAdv_create(&HidDev_advCallback, &advParamsInitial, &advHandleInitial);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load advertising data for set #1 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleInitial, GAP_ADV_DATA_TYPE_ADV,
                                            sizeof(advData), advData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load scan response data for set #1 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleInitial, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                            sizeof(scanData), scanData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Set event mask for set #1
                status = GapAdv_setEventMask(advHandleInitial, GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                        | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);

                // Adverisement set #2, high
                // Create Advertisement set #2 and assign handle
                status = GapAdv_create(&HidDev_advCallback, &advParamsHigh, &advHandleHigh);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load advertising data for set #2 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleHigh, GAP_ADV_DATA_TYPE_ADV,
                                            sizeof(advData), advData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load scan response data for set #2 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleHigh, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                            sizeof(scanData), scanData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Set event mask for set #2
                status = GapAdv_setEventMask(advHandleHigh,GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);

                // Adverisement set #3, low
                // Create Advertisement set #3 and assign handle
                status = GapAdv_create(&HidDev_advCallback, &advParamsLow, &advHandleLow);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load advertising data for set #3 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleLow, GAP_ADV_DATA_TYPE_ADV,
                                            sizeof(advData), advData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Load scan response data for set #3 that is statically allocated by the app
                status = GapAdv_loadByHandle(advHandleLow,  GAP_ADV_DATA_TYPE_SCAN_RSP,
                                            sizeof(scanData), scanData);
                HIDDEV_ASSERT(status == SUCCESS);

                // Set event mask for set #3
                status = GapAdv_setEventMask(advHandleLow, GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);

                // Enable legacy advertising for set #1
                status = GapAdv_enable(advHandleInitial,  GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
                HIDDEV_ASSERT(status == SUCCESS);
                hidAdvState = HID_ADV_STATE_INITIAL;

                if (addrMode > ADDRMODE_RANDOM)
                {
                    HidDev_updateRPA();

                    // Create one-shot clock for RPA check event.
                    Util_constructClock(&clkRpaRead, HidDev_clockHandler,
                                        READ_RPA_PERIOD, 0, true, (UArg) &argRpaRead);
                }
            }
            hidDevGapState = HID_GAPSTATE_ADVERTISING;
            break;
        }

        case GAP_LINK_ESTABLISHED_EVENT:
        {
            gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t*) pMsg;
            uint8_t numActive = linkDB_NumActive();
            hidDevGapState = HID_GAPSTATE_CONNECTED;
            hidAdvState = HID_ADV_STATE_NONE;
            if (pPkt->hdr.status == SUCCESS)
            {
                // Add connection to list and start RSSI
                HidDev_addConn(pPkt->connectionHandle);

                // Start Periodic Clock.
                Util_startClock(&clkPeriodic);

                // Connection not secure yet.
                hidDevConnSecure = FALSE;

                // Start idle timer.
                HidDev_StartIdleTimer();
                isConnected = true;

                // If there are reports in the queue
                if (!reportQEmpty())
                {
                    Event_post(syncEvent, HID_SEND_REPORT_EVT);
                }

                // Update connection params
                HidDev_processParamUpdate(pPkt->connectionHandle);
            }
            break;
        }

        case GAP_LINK_TERMINATED_EVENT:
        {
            gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t*) pMsg;
            hidDevGapState = HID_GAPSTATE_WAITING;
            // The amount of current connections
            uint8_t numActive = linkDB_NumActive();

            // Remove the connection from the list and disable RSSI if needed
            HidDev_removeConn(pPkt->connectionHandle);

            HidDev_disconnected();

            // If no active connections
            if (numActive == 0)
            {
                // Stop periodic clock
                Util_stopClock(&clkPeriodic);
            }

            if (pairingStatus == SMP_PAIRING_FAILED_CONFIRM_VALUE)
            {
                // Bonding failed due to mismatched confirm values.
                status = GapAdv_enable(advHandleInitial, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
                HIDDEV_ASSERT(status == SUCCESS)
                hidAdvState = HID_ADV_STATE_INITIAL;
                pairingStatus = SUCCESS;
                hidDevGapState = HID_GAPSTATE_ADVERTISING;
            }

            break;
        }

        case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
        {
            gapUpdateLinkParamReqReply_t rsp;

            gapUpdateLinkParamReqEvent_t *pReq =
                    (gapUpdateLinkParamReqEvent_t*) pMsg;

            rsp.connectionHandle = pReq->req.connectionHandle;
            rsp.signalIdentifier = pReq->req.signalIdentifier;

            // Only accept connection intervals with slave latency of 0
            // This is just an example of how the application can send a response
            if (pReq->req.connLatency == 0)
            {
                rsp.intervalMin = pReq->req.intervalMin;
                rsp.intervalMax = pReq->req.intervalMax;
                rsp.connLatency = pReq->req.connLatency;
                rsp.connTimeout = pReq->req.connTimeout;
                rsp.accepted = TRUE;
            }
            else
            {
                rsp.accepted = FALSE;
            }

            // Send Reply
            VOID GAP_UpdateLinkParamReqReply(&rsp);

            break;
        }

        case GAP_LINK_PARAM_UPDATE_EVENT:
        {
            gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

            // Get the address from the connection handle
            linkDBInfo_t linkInfo;
            linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

            // Check if there are any queued parameter updates
            hidConnHandleEntry_t *connHandleEntry = (hidConnHandleEntry_t*) List_get(&paramUpdateList);
            if (connHandleEntry != NULL)
            {
                // Attempt to send queued update now
                HidDev_processParamUpdate(connHandleEntry->connHandle);

                // Free list element
                ICall_free(connHandleEntry);
            }
            break;
        }

#ifdef NOTIFY_PARAM_UPDATE_RJCT
        case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
        {
            linkDBInfo_t linkInfo;
            gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

            // Get the address from the connection handle
            linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

            break;
        }
#endif

        default:
            break;
    }
}

/*********************************************************************
 * @fn      HidDev_disconnected
 *
 * @brief   Handle disconnect.
 *
 * @return  none
 */
static void HidDev_disconnected(void)
{
    bStatus_t status = FAILURE;

    // Update connection status
    isConnected = false;

    // Stop idle timer.
    HidDev_StopIdleTimer();

    // Reset state variables.
    hidDevConnSecure = FALSE;
    hidProtocolMode = HID_PROTOCOL_MODE_REPORT;
    hidDevPairingStarted = FALSE;
    hidDevGapBondPairingState = HID_GAPBOND_PAIRING_STATE_NONE;

    // Reset last report sent out
    memset(&lastReport, 0, sizeof(hidDevReport_t));

    // If bonded and normally connectable start advertising.
    if ((HidDev_bondCount() > 0) && (pHidDevCfg->hidFlags & HID_FLAGS_NORMALLY_CONNECTABLE))
    {
        if (hidAdvState != HID_ADV_STATE_NONE)
        {
            HidDev_StopAdvertising();
        }
        status = GapAdv_enable(advHandleLow, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        HIDDEV_ASSERT(status == SUCCESS);
        hidAdvState = HID_ADV_STATE_LOW;
    }

    // Notify application
    (*pHidDevCB->evtCB)(HID_DEV_GAPBOND_STATE_CHANGE_EVT);
}

/*********************************************************************
 * @fn      HidDev_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HidDev_addConn(uint16_t connHandle)
{
    uint8_t i;
    uint8_t status = bleNoResources;

    // Try to find an available entry
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;

            // Allocate data to send through clock handler
            connList[i].pParamUpdateEventData = ICall_malloc(sizeof(hidClockEventData_t) + sizeof(uint16_t));
            if (connList[i].pParamUpdateEventData)
            {
                connList[i].pParamUpdateEventData->event = HID_SEND_PARAM_UPDATE_EVT;
                *((uint16_t*) connList[i].pParamUpdateEventData->data) = connHandle;

                // Create a clock object and start
                connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

                if (connList[i].pUpdateClock)
                {
                    Util_constructClock(connList[i].pUpdateClock, HidDev_clockHandler,
                        SEND_PARAM_UPDATE_DELAY, 0, true, (UArg) (connList[i].pParamUpdateEventData));
                }
                else
                {
                    ICall_free(connList[i].pParamUpdateEventData);
                }
            }
            else
            {
                status = bleMemAllocError;
            }
            break;
        }
    }

    return status;
}

/*********************************************************************
 * @fn      HidDev_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HidDev_getConnIndex(uint16_t connHandle)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == connHandle)
        {
            return i;
        }
    }

    return MAX_NUM_BLE_CONNS;
}

/*********************************************************************
 * @fn      HidDev_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t HidDev_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != LINKDB_CONNHANDLE_ALL)
    {
        // Get connection index from handle
        connIndex = HidDev_getConnIndex(connHandle);
        if (connIndex >= MAX_NUM_BLE_CONNS)
        {
            return (bleInvalidRange);
        }
    }

    // Clear specific handle or all handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
        }
    }

    return (SUCCESS);
}

/*********************************************************************
 * @fn      HidDev_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void HidDev_clearPendingParamUpdate(uint16_t connHandle)
{
    List_Elem *curr;

    for (curr = List_head(&paramUpdateList); curr != NULL;
            curr = List_next(curr))
    {
        if (((hidConnHandleEntry_t*) curr)->connHandle == connHandle)
        {
            List_remove(&paramUpdateList, curr);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t HidDev_removeConn(uint16_t connHandle)
{
    uint8_t connIndex = HidDev_getConnIndex(connHandle);

    if (connIndex != MAX_NUM_BLE_CONNS)
    {
        Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

        if (pUpdateClock != NULL)
        {
            // Stop and destruct the RTOS clock if it's still alive
            if (Util_isActive(pUpdateClock))
            {
                Util_stopClock(pUpdateClock);
            }

            // Destruct the clock object
            Clock_destruct(pUpdateClock);
            // Free clock struct
            ICall_free(pUpdateClock);
            // Free ParamUpdateEventData
            ICall_free(connList[connIndex].pParamUpdateEventData);
        }
        // Clear pending update requests from paramUpdateList
        HidDev_clearPendingParamUpdate(connHandle);
        // Clear Connection List Entry
        HidDev_clearConnListEntry(connHandle);
    }

    return connIndex;
}

/*********************************************************************
 * @fn      HidDev_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void HidDev_updateRPA(void)
{
    uint8_t *pRpaNew;

    // Read the current RPA.
    pRpaNew = GAP_GetDevAddress(FALSE);

    if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
    {
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
    }
}

/*********************************************************************
 * @fn      HidDev_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void HidDev_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
    hidGapAdvEventData_t *pData = ICall_malloc(sizeof(hidGapAdvEventData_t));

    if (pData)
    {
        pData->event = event;
        pData->pBuf = pBuf;

        if (HidDev_enqueueMsg(HID_ADV_EVT, (uint8_t*) pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle.
 * @param   state      - pairing state
 * @param   status     - status upon entering this state.
 *
 * @return  none
 */
static void HidDev_pairStateCB(uint16_t connHandle, uint8_t state,
                               uint8_t status)
{
    hidPairStateData_t *pData = ICall_malloc(sizeof(hidPairStateData_t));

    // Allocate space for the event data.
    if (pData)
    {
        pData->state = state;
        pData->connHandle = connHandle;
        pData->status = status;

        // Queue the event.
        if(HidDev_enqueueMsg(HID_PAIR_STATE_EVT, (uint8_t*)pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_processPairStateEvt
 *
 * @brief   Process pairing state callback.
 *
 * @param   state  - pairing state
 * @param   status - status upon entering this state.
 *
 * @return  none
 */
static void HidDev_processPairStateEvt(hidPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;
    
    if (state == GAPBOND_PAIRING_STATE_STARTED)
    {
        hidDevPairingStarted = TRUE;
    }
    else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        hidDevPairingStarted = FALSE;
        pairingStatus = status;

        if (status == SUCCESS)
        {
            hidDevConnSecure = TRUE;
            Util_restartClock(&reportReadyClock, HID_REPORT_READY_TIME);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
    {
        if (status == SUCCESS)
        {
            hidDevConnSecure = TRUE;
            Util_restartClock(&reportReadyClock, HID_REPORT_READY_TIME);

#if DEFAULT_SCAN_PARAM_NOTIFY_TEST == TRUE
            ScanParam_RefreshNotify(gapConnHandle);
#endif
        }
    }

    // Update GAP Bond pairing state
    hidDevGapBondPairingState = state;

    // Notify application
    (*pHidDevCB->evtCB)(HID_DEV_GAPBOND_STATE_CHANGE_EVT);

    // Process HID reports
    if (!reportQEmpty() && hidDevConnSecure)
    {
        // Notify our task to send out pending reports.
        Event_post(syncEvent, HID_SEND_REPORT_EVT);
    }
}

/*********************************************************************
 * @fn      HidDev_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either
 *                       public or random.
 * @param   connHandle - connection handle
 * @param   uiInputs   - pairing User Interface Inputs - Ask user to input
 *                       passcode.
 * @param   uiOutputs  - pairing User Interface Outputs - Display passcode.
 *
 * @return  none
 */
static void HidDev_passcodeCB(uint8_t *pDeviceAddr, uint16_t connHandle,
                              uint8_t uiInputs, uint8_t uiOutputs,
                              uint32_t numComparison)
{
  hidPasscodeData_t *pData = ICall_malloc(sizeof(hidPasscodeData_t));

    // Allocate space for the passcode event.
    if (pData )
    {
        pData->connHandle = connHandle;
        memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
        pData->uiInputs = uiInputs;
        pData->uiOutputs = uiOutputs;
        pData->numComparison = numComparison;

        // Enqueue the event.
        if(HidDev_enqueueMsg(HID_PASSCODE_EVT, (uint8_t*)pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_processPasscodeEvt
 *
 * @brief   Process passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either
 *                       public or random.
 * @param   connHandle - connection handle
 * @param   uiInputs   - pairing User Interface Inputs - Ask user to input
 *                       passcode.
 * @param   uiOutputs  - pairing User Interface Outputs - Display passcode.
 *
 * @return  none
 */
static void HidDev_processPasscode(hidPasscodeData_t *pPasscodeData)
{
    // Send passcode response
    GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                            B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      HidDev_batteryCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_batteryCB(uint8_t event)
{
    // Queue the event.
    hidBattEvtData_t *pData = ICall_malloc(sizeof(hidBattEvtData_t));

    // Allocate space for the event data.
    if (pData)
    {
        pData->state = event;
        pData->connHandle = 0;
        pData->status = 0;

        // Queue the event.
        if(HidDev_enqueueMsg(HID_BATT_SERVICE_EVT, (uint8_t*)pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_processBatteryEvt
 *
 * @brief   Processes callback from the battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_processBatteryEvt(hidBattEvtData_t* pData)
{
    if (pData->state == BATT_LEVEL_NOTI_ENABLED)
    {
        // If connected start periodic measurement.
        if (hidDevGapState == HID_GAPSTATE_CONNECTED)
        {
            Util_startClock(&battPerClock);
        }
    }
    else if (pData->state == BATT_LEVEL_NOTI_DISABLED)
    {
        // Stop periodic measurement.
        Util_stopClock(&battPerClock);
    }
}

/*********************************************************************
 * @fn      HidDev_scanParamCB
 *
 * @brief   Callback function for scan parameter service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HidDev_scanParamCB(uint8_t event)
{
    // Do nothing.
}

/*********************************************************************
 * @fn      HidDev_battPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HidDev_battPeriodicTask(void)
{
    if (hidDevGapState == HID_GAPSTATE_CONNECTED)
    {
        // Perform battery level check.
        Batt_MeasLevel();

        // Restart clock.
        Util_startClock(&battPerClock);
    }
}

/*********************************************************************
 * @fn      HidDev_reportByHandle
 *
 * @brief   Find the HID report structure for the given handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t* HidDev_reportByHandle(uint16_t handle)
{
    uint8_t i;
    hidRptMap_t *p = pHidDevRptTbl;

    for (i = hidDevRptTblLen; i > 0; i--, p++)
    {
        if (p->handle == handle && p->mode == hidProtocolMode)
        {
            return p;
        }
    }

    return NULL;
}

/*********************************************************************
 * @fn      HidDev_reportByCccdHandle
 *
 * @brief   Find the HID report structure for the given CCC handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t* HidDev_reportByCccdHandle(uint16_t handle)
{
    uint8_t i;
    hidRptMap_t *p = pHidDevRptTbl;

    for (i = hidDevRptTblLen; i > 0; i--, p++)
    {
        if ((p->pCccdAttr != NULL) && (p->pCccdAttr->handle == handle))
        {
            return p;
        }
    }

    return NULL;
}

/*********************************************************************
 * @fn      HidDev_reportById
 *
 * @brief   Find the HID report structure for the Report ID and type.
 *
 * @param   id   - HID report ID
 * @param   type - HID report type
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t* HidDev_reportById(uint8_t id, uint8_t type)
{
    uint8_t i;
    hidRptMap_t *p = pHidDevRptTbl;

    for (i = hidDevRptTblLen; i > 0; i--, p++)
    {
        if (p->id == id && p->type == type && p->mode == hidProtocolMode)
        {
            return p;
        }
    }

    return NULL;
}

/*********************************************************************
 * @fn      HidDev_sendReport
 *
 * @brief   Send a HID report.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_sendReport(uint8_t id, uint8_t type, uint8_t len,
                              uint8_t *pData)
{
    hidRptMap_t *pRpt;
    // Get ATT handle for report.
    if ((pRpt = HidDev_reportById(id, type)) != NULL)
    {
        uint8_t value = GATTServApp_ReadCharCfg(
                gapConnHandle, GATT_CCC_TBL(pRpt->pCccdAttr->pValue));

        // If notifications are enabled
        if (value & GATT_CLIENT_CFG_NOTIFY)
        {
            // Send report notification
            if (HidDev_sendNoti(pRpt->handle, len, pData) == SUCCESS)
            {
                // Save the report just sent out
                lastReport.id = id;
                lastReport.type = type;
                lastReport.len = len;
                memcpy(lastReport.data, pData, len);
            }

            // Start idle timer.
            HidDev_StartIdleTimer();
        }
    }
}

/*********************************************************************
 * @fn      hidDevSendNoti
 *
 * @brief   Send a HID notification.
 *
 * @param   handle - Attribute handle.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  Success or failure.
 */
static uint8_t HidDev_sendNoti(uint16_t handle, uint8_t len, uint8_t *pData)
{
    uint8_t status;
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI, len,
                                NULL);
    if (noti.pValue != NULL)
    {
        noti.handle = handle;
        noti.len = len;
        memcpy(noti.pValue, pData, len);

        // Attempt to send a notification
        status = GATT_Notification(gapConnHandle, &noti, FALSE);
        if (status != SUCCESS)
        {
          GATT_bm_free((gattMsg_t*) &noti, ATT_HANDLE_VALUE_NOTI);
        }
    }
    else
    {
      status = bleMemAllocError;
    }

    return status;
}

/*********************************************************************
 * @fn      HidDev_enqueueReport
 *
 * @brief   Enqueue a HID report to be sent later.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void HidDev_enqueueReport(uint8_t id, uint8_t type, uint8_t len,
                                 uint8_t *pData)
{
    // Enqueue only if bonded.
    if (HidDev_bondCount() > 0)
    {
        // Update last index.
        lastQIdx = (lastQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

        if (lastQIdx == firstQIdx)
        {
            // Queue overflow; discard oldest report.
            firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;
        }

        // Save report.
        hidDevReportQ[lastQIdx].id = id;
        hidDevReportQ[lastQIdx].type = type;
        hidDevReportQ[lastQIdx].len = len;
        memcpy(hidDevReportQ[lastQIdx].data, pData, len);
        if (hidDevConnSecure)
        {
            // Notify our task to send out pending reports.
            Event_post(syncEvent, HID_SEND_REPORT_EVT);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_dequeueReport
 *
 * @brief   Dequeue a HID report to be sent out.
 *
 * @param   id    - HID report ID.
 * @param   type  - HID report type.
 * @param   len   - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static hidDevReport_t* HidDev_dequeueReport(void)
{
    if (reportQEmpty())
    {
        return NULL;
    }

    // Update first index.
    firstQIdx = (firstQIdx + 1) % HID_DEV_REPORT_Q_SIZE;

    return (&(hidDevReportQ[firstQIdx]));
}

/*********************************************************************
 * @fn      HidDev_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void HidDev_processParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    connIndex = HidDev_getConnIndex(connHandle);
    if (connIndex >= MAX_NUM_BLE_CONNS)
    {
        return;
    }

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct, only in case it is not NULL
    if (connList[connIndex].pUpdateClock != NULL)
    {
        ICall_free(connList[connIndex].pUpdateClock);
        connList[connIndex].pUpdateClock = NULL;
    }
    // Free ParamUpdateEventData, only in case it is not NULL
    if (connList[connIndex].pParamUpdateEventData != NULL)
    {
        ICall_free(connList[connIndex].pParamUpdateEventData);
    }

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the udpate completes
    if (status == bleAlreadyInRequestedMode)
    {
        hidConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(hidConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = connHandle;

            List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
        }
    }
}

/*********************************************************************
 * @fn      HidDev_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidDev_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
    uint8_t status = pMsg->pReturnParam[0];

    //Find which command this command complete is for
    switch (pMsg->cmdOpcode)
    {
        // Disable DLE feature
        case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
        {
            if (status == SUCCESS)
            {
                uint8_t featSet[8];

                // Get current feature set from received event (bits 1-9
                // of the returned data
                memcpy(featSet, &pMsg->pReturnParam[1], 8);

                // Clear bit 2 of byte 0 of feature set to disable LL
                // Feature Reject extended
                CLR_FEATURE_FLAG(featSet[0],
                                 LL_FEATURE_DATA_PACKET_LENGTH_EXTENSION);

                // Clear bits 0 and 3 of byte 1
                CLR_FEATURE_FLAG( featSet[1], LL_FEATURE_2M_PHY );
                CLR_FEATURE_FLAG( featSet[1], LL_FEATURE_CODED_PHY );

                //CLR_FEATURE_FLAG( featSet[1], LL_FEATURE_CHAN_ALGO_2 );
                // Update controller with modified features
                HCI_EXT_SetLocalSupportedFeaturesCmd(featSet);
            }
            break;
        }

        case HCI_LE_READ_PHY:
        {
            break;
        }

        default:
            break;
    }
}

/*********************************************************************
 * @fn      HidDev_bondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   None.
 *
 * @return  number of bonded devices.
 */
static uint8_t HidDev_bondCount(void)
{
    uint8_t bondCnt = 0;

    VOID GAPBondMgr_GetParameter(GAPBOND_BOND_COUNT, &bondCnt);

    return bondCnt;
}

/*********************************************************************
 * @fn      HidDev_isbufset
 *
 * @brief   Is all of the array elements set to a value?
 *
 * @param   buf - buffer to check.
 * @param   val - value to check each array element for.
 * @param   len - length to check.
 *
 * @return  TRUE if all "val".
 *          FALSE otherwise.
 */
static uint8_t HidDev_isbufset(uint8_t *buf, uint8_t val, uint8_t len)
{
    uint8_t x;

    // Validate pointer and length of report
    if ((buf == NULL) || (len > HID_DEV_DATA_LEN))
    {
        return ( FALSE);
    }

    for (x = 0; x < len; x++)
    {
        // Check for non-initialized value
        if (buf[x] != val)
        {
            return FALSE;
        }
    }

    return TRUE;
}

/*********************************************************************
 * @fn      HidDev_clockHandler
 *
 * @brief   Clock handle for all clock events.  This function stores an event
 *          flag and wakes up the application's event processor.
 *
 * @param   arg - event flag.
 *
 * @return  None
 */
static void HidDev_clockHandler(UArg arg)
{
    Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      HidDev_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   state  - message state.
 * @param   pData  - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HidDev_enqueueMsg(uint16_t event, uint8_t *pData)
{
    uint8_t success;
    hidDevEvt_t *pMsg = ICall_malloc(sizeof(hidDevEvt_t));

    // Create dynamic pointer to message.
    if(pMsg)
    {
      pMsg->event = event;
      pMsg->pData = pData;

      // Enqueue the message.
      success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
      return (success) ? SUCCESS : FAILURE;
    }

    return bleMemAllocError;
}

/*********************************************************************
 * @fn      HidDev_reportReadyClockCB
 *
 * @brief   Handles HID reports when delay has expired
 *
 * @param   None.
 *
 * @return  None.
 */
static void HidDev_reportReadyClockCB(UArg a0)
{
    // Allow reports to be sent
    hidDevReportReadyState = TRUE;

    // If there are reports in the queue
    if (!reportQEmpty())
    {
        Event_post(syncEvent, HID_SEND_REPORT_EVT);
    }
}

/*********************************************************************
 *********************************************************************/
