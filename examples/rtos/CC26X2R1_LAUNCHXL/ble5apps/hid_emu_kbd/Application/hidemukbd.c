/******************************************************************************

 @file       hidemukbd.c

 @brief This file contains the HID emulated keyboard sample application for use
 with the CC2652 Bluetooth Low Energy
 Protocol Stack.

 Group: CMCU, SCS
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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <icall.h>
#include <string.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include "ll_common.h"

#include <devinfoservice.h>
#include "profiles/hid/hidkbdservice.h"
#include "hiddev.h"

#include "board_key.h"
#include "ti_ble_config.h"
#include "ti_drivers_config.h"
#include "hidemukbd.h"

/*********************************************************************
 * MACROS
 */

// No Key Press ID
#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        6


// HID consumer input report length
#define HID_CONSUMER_IN_RPT_LEN     1

/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              60000

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

// Task configuration
#define HIDEMUKBD_TASK_PRIORITY               1
#define HIDEMUMSE_TASK_PRIORITY               2

#ifndef HIDEMUKBD_TASK_STACK_SIZE
#define HIDEMUKBD_TASK_STACK_SIZE             8192
#endif

#ifndef HIDEMUMSE_TASK_STACK_SIZE
#define HIDEMUMSE_TASK_STACK_SIZE             8192
#endif

// App Events
#define HIDEMUKBD_KEYBOARD_REPORT_EVT         0x0001
#define HIDEMUKBD_MOUSE_REPORT_EVT            0x0002
#define HIDEMUKBD_CONSUMER_REPORT_EVT         0x0003

// Task Events
#define HIDEMUKBD_ICALL_EVT                   ICALL_MSG_EVENT_ID // Event_Id_31
#define HIDEMUKBD_QUEUE_EVT                   UTIL_QUEUE_EVENT_ID // Event_Id_30

#define HIDEMUKBD_ALL_EVENTS                  (HIDEMUKBD_ICALL_EVT | \
                                               HIDEMUKBD_QUEUE_EVT)

#define HID_MOUSE_BUTTONS_OFFSET    0
#define HID_MOUSE_DELTAX_OFFSET     8
#define HID_MOUSE_DELTAY_OFFSET     16
#define HID_MOUSE_WHEEL_OFFSET      24

#define HID_MOUSE_BUTTONS_MASK      0xFF
#define HID_MOUSE_DELTAX_MASK       0xFF00
#define HID_MOUSE_DELTAY_MASK       0xFF0000
#define HID_MOUSE_WHEEL_MASK        0xFF000000

// Consumer Report IDs
#define HID_CON_POWER               1
#define HID_CON_PLAY_PAUSE          2
#define HID_CON_STOP                3
#define HID_CON_SKIP_TRACK          4
#define HID_CON_PREV_TRACK          5
#define HID_CON_FAST_FORWARD        6
#define HID_CON_REWIND              7
#define HID_CON_RECORD              8
#define HID_CON_VOLUME_UP           9
#define HID_CON_VOLUME_DOWN         10
#define HID_CON_MUTE                11

#define WAIT_FOREVER (~(0U))
/*********************************************************************
 * TYPEDEFS
 */

// HID Input Data union/structure
typedef union {
     struct
     {
         uint8_t modifiers;  // Modifier Keys
         uint8_t reserved;   // Reserved field
         uint8_t keypress1;  // First keypress
         uint8_t keypress2;  // Second keypress
         uint8_t keypress3;  // Third keypress
         uint8_t keypress4;  // Fourth keypress
         uint8_t keypress5;  // Fifth keypress
         uint8_t keypress6;  // Sixth keypress
     } keyboard;
     struct
     {
         uint8_t buttons;    // Mouse Buttons
         int16_t deltaX;     // Cursor Delta X
         int16_t deltaY;     // Cursor Delta Y
         int8_t wheel;       // Mouse Wheel
     } mouse;
} hid_input;

// App event
typedef struct
{
    uint8_t event; // Event type
    hid_input input; // Event data
} hidEmuKbdEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */


uint32_t counter = 0;
uint32_t mouseCount = 0;
uint32_t mouseRepCount = 0;
bool mouseControl = false;
uint8_t Consumer_Report_Data;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern bool isConnected;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct hidEmuKbdTask;
Char hidEmuKbdTaskStack[HIDEMUKBD_TASK_STACK_SIZE];
Task_Struct hidEmuMseTask;
Char hidEmuMseTaskStack[HIDEMUMSE_TASK_STACK_SIZE];

SemaphoreP_Handle mouseSem;

// HID Dev configuration
static hidDevCfg_t hidEmuKbdCfg =
{
     DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
     HID_KBD_FLAGS               // HID feature flags
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Application task and event processing.
static void HidEmuKbd_init(void);
static void HidEmuKbd_taskFxn(UArg a0, UArg a1);
static void HidEmuKbd_mouseTaskFxn(UArg a0, UArg a1);
static void HidEmuKbd_processAppMsg(hidEmuKbdEvt_t *pMsg);
static void HidEmuKbd_processStackMsg(ICall_Hdr *pMsg);
static void HidEmuKbd_processGattMsg(gattMsgEvent_t *pMsg);
static uint8_t HidEmuKbd_enqueueMsg(uint8_t event, hid_input input);
// Key press.
static void HidEmuKbd_keyPressHandler(uint8_t keys);
// HID reports.
static void HidEmuKbd_sendKeyboardInput(hid_input input);
static void HidEmuKbd_sendMouseInput(hid_input input);
static void HidEmuKbd_sendConsumerInput(hid_input input);

static uint8_t HidEmuKbd_receiveReport(uint8_t len, uint8_t *pData);
static uint8_t HidEmuKbd_reportCB(uint8_t id, uint8_t type, uint16_t uuid,
                                  uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void HidEmuKbd_hidEventCB(uint8_t evt);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuKbdHidCBs =
{
     HidEmuKbd_reportCB,
     HidEmuKbd_hidEventCB,
     NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmuKbd_createTask
 *
 * @brief   Task creation function for the HID emulated keyboard.
 *
 * @param   none
 *
 * @return  none
 */
void HidEmuKbd_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = hidEmuKbdTaskStack;
    taskParams.stackSize = HIDEMUKBD_TASK_STACK_SIZE;
    taskParams.priority = HIDEMUKBD_TASK_PRIORITY;

    Task_construct(&hidEmuKbdTask, HidEmuKbd_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HidEmuKbd_createMouseTask
 *
 * @brief   Task creation function for the HID emulated mouse.
 *
 * @param   none
 *
 * @return  none
 */
void HidEmuKbd_createMouseTask(void)
{
    Task_Params taskParams;

    // Mouse task needs a semaphore
    mouseSem = SemaphoreP_createBinary(0);

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = hidEmuMseTaskStack;
    taskParams.stackSize = HIDEMUMSE_TASK_STACK_SIZE;
    taskParams.priority = HIDEMUMSE_TASK_PRIORITY;

    Task_construct(&hidEmuMseTask, HidEmuKbd_mouseTaskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HidEmuKbd_init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void HidEmuKbd_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Set up HID keyboard service
    HidKbd_AddService();

    // Register for HID Dev callback
    HidDev_Register(&hidEmuKbdCfg, &hidEmuKbdHidCBs);
    
    // Init key debouncer
    Board_initKeys(HidEmuKbd_keyPressHandler);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

#ifndef USE_LL_CONN_PARAM_UPDATE
    // Get the currently set local supported LE features
    // The HCI will generate an HCI event that will get received in the main
    // loop
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

    // Use default data Tx / Rx data length and times
    HCI_EXT_SetMaxDataLenCmd(LL_MIN_LINK_DATA_LEN, LL_MIN_LINK_DATA_TIME,
                             LL_MIN_LINK_DATA_LEN, LL_MIN_LINK_DATA_TIME);

#ifndef DISABLE_MOUSE
    HidEmuKbd_createMouseTask();
#endif

}

/*********************************************************************
 * @fn      HidEmuKbd_taskFxn
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used.
 *
 * @return  none
 */
void HidEmuKbd_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    HidEmuKbd_init();

    uint8_t gapbondSecure = GAPBOND_SECURE_CONNECTION_NONE;
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &gapbondSecure);

    // Application main loop.
    for (;;)
    {
        uint32_t events;
        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, HIDEMUKBD_ALL_EVENTS,
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
                    // Process inter-task message
                    HidEmuKbd_processStackMsg((ICall_Hdr*) pMsg);
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & HIDEMUKBD_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    hidEmuKbdEvt_t *pMsg = (hidEmuKbdEvt_t*) Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message.
                        HidEmuKbd_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }
        }
    }
}

/*
 *  ======== mouseTaskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
static void HidEmuKbd_mouseTaskFxn(UArg a0, UArg a1)
{
    // Loop control variables
    int i;
    int j;
    // HID input variable
    hid_input mouseInput = {0};

    while (true)
    {
        /* For each entry in the lookup table */
        for (j = 0; j < 4; j++)
        {
            // Determine what input we need to provide the mouse for this side of the square
            switch (j)
            {
                case 0:
                    mouseInput.mouse.deltaX = 2;
                    mouseInput.mouse.deltaY = 0;
                    break;
                case 1:
                    mouseInput.mouse.deltaX = 0;
                    mouseInput.mouse.deltaY = -2;
                    break;
                case 2:
                    mouseInput.mouse.deltaX = -2;
                    mouseInput.mouse.deltaY = 0;
                    break;
                case 3:
                    mouseInput.mouse.deltaX = 0;
                    mouseInput.mouse.deltaY = 2;
                    break;
                default:
                    mouseInput.mouse.deltaX = 0;
                    mouseInput.mouse.deltaY = 0;
                    break;
            }

            /* Perform the action i times to make it look gradual */
            for (i = 0; i < 20; i++)
            {
                // if mouse demo mode is enabled and the device is connected
                if (mouseControl && isConnected)
                {
                    HidEmuKbd_enqueueMsg(HIDEMUKBD_MOUSE_REPORT_EVT, mouseInput);
                    Task_sleep(800);
                }
                // If both buttons are no longer being pressed, we want to
                // sleep this thread until they are both pressed again
                else
                {
                    SemaphoreP_pend(mouseSem, WAIT_FOREVER);
                }
            }
        }
    }
}



/*********************************************************************
 * @fn      HidEmuKbd_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidEmuKbd_processStackMsg(ICall_Hdr *pMsg)
{
    switch (pMsg->event)
    {
        case GATT_MSG_EVENT:
            HidEmuKbd_processGattMsg((gattMsgEvent_t*) pMsg);
            break;

        case HCI_GAP_EVENT_EVENT:
        {
            // Process HCI message
            switch (pMsg->status)
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                // Process HCI Command Complete Event
                {
                    #ifndef USE_LL_CONN_PARAM_UPDATE
                    // This code will disable the use of the LL_CONNECTION_PARAM_REQ
                    // control procedure (for connection parameter updates, the
                    // L2CAP Connection Parameter Update procedure will be used
                    // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
                    // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

                    // Parse Command Complete Event for opcode and status
                    hciEvt_CmdComplete_t *command_complete =
                    (hciEvt_CmdComplete_t*) pMsg;
                    uint8_t pktStatus = command_complete->pReturnParam[0];

                    //find which command this command complete is for
                    switch (command_complete->cmdOpcode)
                    {
                        case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                        {
                            if (pktStatus == SUCCESS)
                            {
                                uint8_t featSet[8];

                                // get current feature set from received event (bits 1-9 of
                                // the returned data
                                memcpy(featSet, &command_complete->pReturnParam[1], 8);

                                // Clear bit 1 of byte 0 of feature set to disable LL
                                // Connection Parameter Updates
                                CLR_FEATURE_FLAG(featSet[0], LL_FEATURE_CONN_PARAMS_REQ);

                                // Update controller with modified features
                                HCI_EXT_SetLocalSupportedFeaturesCmd(featSet);
                            }
                            break;
                        }

                        default:
                            //do nothing
                            break;
                    }
                    #endif // !defined (USE_LL_CONN_PARAM_UPDATE)
                    break;
                }


                default:
                    break;
            }
            break;
        }

        default:
            // Do nothing
            break;
    }
}

/*********************************************************************
 * @fn      HidEmuKbd_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void HidEmuKbd_processGattMsg(gattMsgEvent_t *pMsg)
{
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HidEmuKbd_processAppMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidEmuKbd_processAppMsg(hidEmuKbdEvt_t *pMsg)
{
    switch (pMsg->event)
    {
    case HIDEMUKBD_KEYBOARD_REPORT_EVT:
        HidEmuKbd_sendKeyboardInput(pMsg->input);
        break;

    case HIDEMUKBD_MOUSE_REPORT_EVT:
        HidEmuKbd_sendMouseInput(pMsg->input);
        break;

    case HIDEMUKBD_CONSUMER_REPORT_EVT:
        HidEmuKbd_sendConsumerInput(pMsg->input);
        break;

    default:
        //Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      HidKEmukbd_keyPressHandler
 *
 * @brief   Key event handler function.
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void HidEmuKbd_keyPressHandler(uint8_t keys)
{
    hid_input input = {0};


    // Both buttons are not being pressed
    if ((keys & (KEY_LEFT | KEY_RIGHT)) != (KEY_LEFT | KEY_RIGHT))
    {
        if (keys & KEY_LEFT)
        {
            // Key Press.
            input.keyboard.keypress1 = HID_KEYBOARD_LEFT_ARROW;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEYBOARD_REPORT_EVT, input);
            // Key Release.
            // NB: releasing a key press will not propagate a signal to this function,
            // so a "key release" is reported immediately afterwards here.
            input.keyboard.keypress1 = KEY_NONE;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEYBOARD_REPORT_EVT, input);

            // Volume down press
            input.keyboard.reserved = HID_CON_VOLUME_DOWN;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_CONSUMER_REPORT_EVT, input);

            // Volume down release
            input.keyboard.reserved = KEY_NONE;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_CONSUMER_REPORT_EVT, input);
        }

        else if (keys & KEY_RIGHT)
        {
            // Key Press.
            input.keyboard.keypress1 = HID_KEYBOARD_RIGHT_ARROW;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEYBOARD_REPORT_EVT, input);
            // Key Release
            // NB: releasing a key press will not propagate a signal to this function,
            // so a "key release" is reported immediately afterwards here.
            input.keyboard.keypress1 = KEY_NONE;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_KEYBOARD_REPORT_EVT, input);

            // Volume up press
            input.keyboard.reserved = HID_CON_VOLUME_UP;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_CONSUMER_REPORT_EVT, input);

            // Volume up release
            input.keyboard.reserved = KEY_NONE;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_CONSUMER_REPORT_EVT, input);
        }
    }
#ifndef DISABLE_MOUSE
    // Toggle mouse movement
    else
    {
        if (mouseControl)
        {
            mouseControl = false;
            HidEmuKbd_enqueueMsg(HIDEMUKBD_MOUSE_REPORT_EVT, input);
        }
        else
        {
            mouseControl = true;
            SemaphoreP_post(mouseSem);
        }
    }
#endif
}

/*********************************************************************
 * @fn      HidEmuKbd_sendKeyboardInput
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void HidEmuKbd_sendKeyboardInput(hid_input input)
{
    uint8_t buf[HID_KEYBOARD_IN_RPT_LEN];

    buf[0] = input.keyboard.modifiers;      // Modifier keys
    buf[1] = input.keyboard.reserved;       // Reserved
    buf[2] = input.keyboard.keypress1;      // Keycode 1
    buf[3] = input.keyboard.keypress2;      // Keycode 2
    buf[4] = input.keyboard.keypress3;      // Keycode 3
    buf[5] = input.keyboard.keypress4;      // Keycode 4
    buf[6] = input.keyboard.keypress5;      // Keycode 5
    buf[7] = input.keyboard.keypress6;      // Keycode 6

    HidDev_createReport(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, HID_KEYBOARD_IN_RPT_LEN, buf);
}

/*********************************************************************
 * @fn      HidEmuKbd_sendMouseInput
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   input - Mouse inputs
 *
 * @return  none
 */
static void HidEmuKbd_sendMouseInput(hid_input input)
{
    uint8_t buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = input.mouse.buttons;                   // Mouse Buttons
    buf[1] = input.mouse.deltaX & 0xFF;             // Cursor Delta X
    buf[2] = (input.mouse.deltaX >> 8) & 0xFF;      // Cursor Delta X
    buf[3] = input.mouse.deltaY& 0xFF;              // Cursor Delta Y
    buf[4] = (input.mouse.deltaY >> 8) & 0xFF;      // Cursor Delta Y
    buf[5] = input.mouse.wheel;                     // Mouse Wheel

    HidDev_createReport(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buf);
}

/*********************************************************************
 * @fn      HidEmuKbd_sendConsumerInput
 *
 * @brief   Build and send a HID consumer report.
 *
 * @param   input - HID input.
 *
 * @return  none
 */
static void HidEmuKbd_sendConsumerInput(hid_input input)
{
    uint8_t buf = input.keyboard.reserved;
    HidDev_createReport(HID_RPT_ID_CONSUMER_IN, HID_REPORT_TYPE_INPUT, HID_CONSUMER_IN_RPT_LEN, &buf);
}

/*********************************************************************
 * @fn      HidEmuKbd_receiveReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8_t HidEmuKbd_receiveReport(uint8_t len, uint8_t *pData)
{
    // Verify data length
    if (len == HID_LED_OUT_RPT_LEN)
    {
        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
    }
}

/*********************************************************************
 * @fn      HidEmuKbd_reportCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8_t HidEmuKbd_reportCB(uint8_t id, uint8_t type, uint16_t uuid,
                                  uint8_t oper, uint16_t *pLen, uint8_t *pData)
{
    uint8_t status = SUCCESS;

    // Write
    if (oper == HID_DEV_OPER_WRITE)
    {
        if (uuid == REPORT_UUID)
        {
            // Process write to LED output report; ignore others
            if (type == HID_REPORT_TYPE_OUTPUT)
            {
                status = HidEmuKbd_receiveReport(*pLen, pData);
            }
        }

        if (status == SUCCESS)
        {
            status = HidKbd_SetParameter(id, type, uuid, *pLen, pData);
        }
    }
    // Read
    else if (oper == HID_DEV_OPER_READ)
    {
        uint8_t len;

        status = HidKbd_GetParameter(id, type, uuid, &len, pData);
        if (status == SUCCESS)
        {
            *pLen = len;
        }
    }
    return status;
}

/*********************************************************************
 * @fn      HidEmuKbd_hidEventCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void HidEmuKbd_hidEventCB(uint8_t evt)
{
    // Process enter/exit suspend or enter/exit boot mode
    return;
}

/*********************************************************************
 * @fn      HidEmuKbd_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   state  - message state.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HidEmuKbd_enqueueMsg(uint8_t event, hid_input input)
{
    hidEmuKbdEvt_t *pMsg;

    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(hidEmuKbdEvt_t)))
    {
        pMsg->event = event;
        pMsg->input = input;

        // Enqueue the message.
        return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t*) pMsg);
    }

    return FALSE;
}

/*********************************************************************
 *********************************************************************/
