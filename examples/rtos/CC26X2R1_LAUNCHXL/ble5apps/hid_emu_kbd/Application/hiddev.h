/******************************************************************************

 @file  hiddev.h

 @brief This file contains the common HID Device profile.

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

#ifndef HIDDEV_H
#define HIDDEV_H

#include <icall_ble_api.h>

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Initial GAP Bond pairing state
#define HID_GAPBOND_PAIRING_STATE_NONE    0xFF  //!< Not connected/paired

// HID Device Parameters
#define HIDDEV_ERASE_ALLBONDS       0x00  // Erase all of the bonded devices.
                                          // Write Only. No Size.
#define HIDDEV_GAPROLE_STATE        0x01  // Reading this parameter will return
                                          // the HID Dev GAP Peripheral Role
                                          // State. Read Only. Size is uint8_t.
#define HIDDEV_GAPBOND_STATE        0x02  // Reading this parameter will return
                                          // the HID Dev GAP Bond Manager
                                          // Pairing State. Read Only.
                                          // Size is uint8_t.

#define HID_GAPSTATE_INIT                      0     //!< Waiting to be started
#define HID_GAPSTATE_STARTED                   1     //!< Started but not advertising
#define HID_GAPSTATE_ADVERTISING               2     //!< Currently Advertising
#define HID_GAPSTATE_ADVERTISING_NONCONN       3     //!< Currently using non-connectable Advertising
#define HID_GAPSTATE_WAITING                   4     //!< Device is started but not advertising, is in waiting period before advertising again
#define HID_GAPSTATE_WAITING_AFTER_TIMEOUT     5     //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again
#define HID_GAPSTATE_CONNECTED                 6     //!< In a connection
#define HID_GAPSTATE_CONNECTED_ADV             7     //!< In a connection + advertising
#define HID_GAPSTATE_ERROR                     8


// HID read/write operation
#define HID_DEV_OPER_WRITE          0  // Write operation
#define HID_DEV_OPER_READ           1  // Read operation
#define HID_DEV_OPER_ENABLE         2  // Notification enabled for report ID
#define HID_DEV_OPER_DISABLE        3  // Notifications disabled for report ID

// HID callback events
#define HID_DEV_SUSPEND_EVT               0  // HID suspend
#define HID_DEV_EXIT_SUSPEND_EVT          1  // HID exit suspend
#define HID_DEV_SET_BOOT_EVT              2  // HID set boot mode
#define HID_DEV_SET_REPORT_EVT            3  // HID set report mode
#define HID_DEV_GAPROLE_STATE_CHANGE_EVT  4  // HID GAP Role state change
#define HID_DEV_GAPBOND_STATE_CHANGE_EVT  5  // HID GAP Bond state change

// HID Report type
#define HID_REPORT_TYPE_INPUT       1
#define HID_REPORT_TYPE_OUTPUT      2
#define HID_REPORT_TYPE_FEATURE     3

// HID information flags
#define HID_FLAGS_REMOTE_WAKE           0x01 // RemoteWake
#define HID_FLAGS_NORMALLY_CONNECTABLE  0x02 // NormallyConnectable

// Control point commands
#define HID_CMD_SUSPEND             0x00 // Suspend
#define HID_CMD_EXIT_SUSPEND        0x01 // Exit Suspend

// HID protocol mode values
#define HID_PROTOCOL_MODE_BOOT      0x00 // Boot Protocol Mode
#define HID_PROTOCOL_MODE_REPORT    0x01 // Report Protocol Mode

// Attribute value lengths
#define HID_PROTOCOL_MODE_LEN       1    // HID Protocol Mode
#define HID_INFORMATION_LEN         4    // HID Information
#define HID_REPORT_REF_LEN          2    // HID Report Reference Descriptor
#define HID_EXT_REPORT_REF_LEN      2    // External Report Reference Descriptor

// HID Keyboard/Keypad Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
#define HID_KEYBOARD_RESERVED       0    // 0x00 - No event indicated
#define HID_KEYBOARD_A              4    // 0x04 - Keyboard a and A
#define HID_KEYBOARD_B              5    // 0x05 - Keyboard b and B
#define HID_KEYBOARD_C              6    // 0x06 - Keyboard c and C
#define HID_KEYBOARD_D              7    // 0x07 - Keyboard d and D
#define HID_KEYBOARD_E              8    // 0x08 - Keyboard e and E
#define HID_KEYBOARD_F              9    // 0x09 - Keyboard f and F
#define HID_KEYBOARD_G              10   // 0x0A - Keyboard g and G
#define HID_KEYBOARD_H              11   // 0x0B - Keyboard h and H
#define HID_KEYBOARD_I              12   // 0x0C - Keyboard i and I
#define HID_KEYBOARD_J              13   // 0x0D - Keyboard j and J
#define HID_KEYBOARD_K              14   // 0x0E - Keyboard k and K
#define HID_KEYBOARD_L              15   // 0x0F - Keyboard l and L
#define HID_KEYBOARD_M              16   // 0x10 - Keyboard m and M
#define HID_KEYBOARD_N              17   // 0x11 - Keyboard n and N
#define HID_KEYBOARD_O              18   // 0x12 - Keyboard o and O
#define HID_KEYBOARD_P              19   // 0x13 - Keyboard p and p
#define HID_KEYBOARD_Q              20   // 0x14 - Keyboard q and Q
#define HID_KEYBOARD_R              21   // 0x15 - Keyboard r and R
#define HID_KEYBOARD_S              22   // 0x16 - Keyboard s and S
#define HID_KEYBOARD_T              23   // 0x17 - Keyboard t and T
#define HID_KEYBOARD_U              24   // 0x18 - Keyboard u and U
#define HID_KEYBOARD_V              25   // 0x19 - Keyboard v and V
#define HID_KEYBOARD_W              26   // 0x1A - Keyboard w and W
#define HID_KEYBOARD_X              27   // 0x1B - Keyboard x and X
#define HID_KEYBOARD_Y              28   // 0x1C - Keyboard y and Y
#define HID_KEYBOARD_Z              29   // 0x1D - Keyboard z and Z
#define HID_KEYBOARD_1              30   // 0x1E - Keyboard 1 and !
#define HID_KEYBOARD_2              31   // 0x1F - Keyboard 2 and @
#define HID_KEYBOARD_3              32   // 0x20 - Keyboard 3 and #
#define HID_KEYBOARD_4              33   // 0x21 - Keyboard 4 and %
#define HID_KEYBOARD_5              34   // 0x22 - Keyboard 5 and %
#define HID_KEYBOARD_6              35   // 0x23 - Keyboard 6 and ^
#define HID_KEYBOARD_7              36   // 0x24 - Keyboard 7 and &
#define HID_KEYBOARD_8              37   // 0x25 - Keyboard 8 and *
#define HID_KEYBOARD_9              38   // 0x26 - Keyboard 9 and (
#define HID_KEYBOARD_0              39   // 0x27 - Keyboard 0 and)
#define HID_KEYBOARD_RETURN         40   // 0x28 - Keyboard Return (ENTER)
#define HID_KEYBOARD_ESCAPE         41   // 0x29 - Keyboard ESCAPE
#define HID_KEYBOARD_DELETE         42   // 0x2A - Keyboard DELETE (Backspace)
#define HID_KEYBOARD_TAB            43   // 0x2B - Keyboard Tab
#define HID_KEYBOARD_SPACEBAR       44   // 0x2C - Keyboard Spacebar
#define HID_KEYBOARD_MINUS          45   // 0x2D - Keyboard - and (underscore)
#define HID_KEYBOARD_EQUAL          46   // 0x2E - Keyboard = and +
#define HID_KEYBOARD_LEFT_BRKT      47   // 0x2F - Keyboard [ and {
#define HID_KEYBOARD_RIGHT_BRKT     48   // 0x30 - Keyboard ] and }
#define HID_KEYBOARD_BACK_SLASH     49   // 0x31 - Keyboard \ and |
#define HID_KEYBOARD_SEMI_COLON     51   // 0x33 - Keyboard ; and :
#define HID_KEYBOARD_SGL_QUOTE      52   // 0x34 - Keyboard ' and "
#define HID_KEYBOARD_GRV_ACCENT     53   // 0x35 - Keyboard Grave Accent and Tilde
#define HID_KEYBOARD_COMMA          54   // 0x36 - Keyboard , and <
#define HID_KEYBOARD_DOT            55   // 0x37 - Keyboard . and >
#define HID_KEYBOARD_FWD_SLASH      56   // 0x38 - Keyboard / and ?
#define HID_KEYBOARD_CAPS_LOCK      57   // 0x39 - Keyboard Caps Lock
#define HID_KEYBOARD_F1             58   // 0x3A - Keyboard F1
#define HID_KEYBOARD_F2             59   // 0x3B - Keyboard F2
#define HID_KEYBOARD_F3             60   // 0x3C - Keyboard F3
#define HID_KEYBOARD_F4             61   // 0x3D - Keyboard F4
#define HID_KEYBOARD_F5             62   // 0x3E - Keyboard F5
#define HID_KEYBOARD_F6             63   // 0x3F - Keyboard F6
#define HID_KEYBOARD_F7             64   // 0x40 - Keyboard F7
#define HID_KEYBOARD_F8             65   // 0x41 - Keyboard F8
#define HID_KEYBOARD_F9             66   // 0x42 - Keyboard F9
#define HID_KEYBOARD_F10            67   // 0x43 - Keyboard F10
#define HID_KEYBOARD_F11            68   // 0x44 - Keyboard F11
#define HID_KEYBOARD_F12            69   // 0x45 - Keyboard F12
#define HID_KEYBOARD_PRNT_SCREEN    70   // 0x46 - Keyboard Print Screen
#define HID_KEYBOARD_SCROLL_LOCK    71   // 0x47 - Keyboard Scroll Lock
#define HID_KEYBOARD_PAUSE          72   // 0x48 - Keyboard Pause
#define HID_KEYBOARD_INSERT         73   // 0x49 - Keyboard Insert
#define HID_KEYBOARD_HOME           74   // 0x4A - Keyboard Home
#define HID_KEYBOARD_PAGE_UP        75   // 0x4B - Keyboard PageUp
#define HID_KEYBOARD_DELETE_FWD     76   // 0x4C - Keyboard Delete Forward
#define HID_KEYBOARD_END            77   // 0x4D - Keyboard End
#define HID_KEYBOARD_PAGE_DOWN      78   // 0x4E - Keyboard PageDown
#define HID_KEYBOARD_RIGHT_ARROW    79   // 0x4F - Keyboard RightArrow
#define HID_KEYBOARD_LEFT_ARROW     80   // 0x50 - Keyboard LeftArrow
#define HID_KEYBOARD_DOWN_ARROW     81   // 0x51 - Keyboard DownArrow
#define HID_KEYBOARD_UP_ARROW       82   // 0x52 - Keyboard UpArrow
#define HID_KEYBPAD_NUM_LOCK        83   // 0x53 - Keypad Num Lock and Clear
#define HID_KEYBPAD_DIVIDE          84   // 0x54 - Keypad /
#define HID_KEYBOARD_MULTIPLY       85   // 0x55 - Keypad *
#define HID_KEYBOARD_SUBTRACT       86   // 0x56 - Keypad -
#define HID_KEYBPAD_ADD             87   // 0x57 - Keypad +
#define HID_KEYBPAD_ENTER           88   // 0x58 - Keypad ENTER
#define HID_KEYBPAD_1               89   // 0x59 - Keypad 1 and End
#define HID_KEYBPAD_2               90   // 0x5A - Keypad 2 and Down Arrow
#define HID_KEYBPAD_3               91   // 0x5B - Keypad 3 and PageDn
#define HID_KEYBPAD_4               92   // 0x5C - Keypad 4 and Left Arrow
#define HID_KEYBPAD_5               93   // 0x5D - Keypad 5
#define HID_KEYBPAD_6               94   // 0x5E - Keypad 6 and Right Arrow
#define HID_KEYBPAD_7               95   // 0x5F - Keypad 7 and Home
#define HID_KEYBPAD_8               96   // 0x60 - Keypad 8 and Up Arrow
#define HID_KEYBPAD_9               97   // 0x61 - Keypad 9 and PageUp
#define HID_KEYBPAD_0               98   // 0x62 - Keypad 0 and Insert
#define HID_KEYBPAD_DOT             99   // 0x63 - Keypad . and Delete
#define HID_KEYBOARD_MUTE           127  // 0x7F - Keyboard Mute
#define HID_KEYBOARD_VOLUME_UP      128  // 0x80 - Keyboard Volume up
#define HID_KEYBOARD_VOLUME_DOWN    129  // 0x81 - Keyboard Volume down
#define HID_KEYBOARD_LEFT_CTRL      224  // 0xE0 - Keyboard LeftContorl
#define HID_KEYBOARD_LEFT_SHIFT     225  // 0xE1 - Keyboard LeftShift
#define HID_KEYBOARD_LEFT_ALT       226  // 0xE2 - Keyboard LeftAlt
#define HID_KEYBOARD_LEFT_GUI       227  // 0xE3 - Keyboard LeftGUI
#define HID_KEYBOARD_RIGHT_CTRL     228  // 0xE4 - Keyboard LeftContorl
#define HID_KEYBOARD_RIGHT_SHIFT    229  // 0xE5 - Keyboard LeftShift
#define HID_KEYBOARD_RIGHT_ALT      230  // 0xE6 - Keyboard LeftAlt
#define HID_KEYBOARD_RIGHT_GUI      231  // 0xE7 - Keyboard RightGUI

#define HID_MOUSE_BUTTON_LEFT       253
#define HID_MOUSE_BUTTON_MIDDLE     254
#define HID_MOUSE_BUTTON_RIGHT      255

// HID Consumer Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
#define HID_CONSUMER_POWER          48  // 0x30 - Power
#define HID_CONSUMER_RESET          49  // 0x31 - Reset
#define HID_CONSUMER_SLEEP          50  // 0x32 - Sleep

#define HID_CONSUMER_MENU           64  // 0x40 - Menu
#define HID_CONSUMER_SELECTION      128 // 0x80 - Selection
#define HID_CONSUMER_ASSIGN_SEL     129 // 0x81 - Assign Selection
#define HID_CONSUMER_MODE_STEP      130 // 0x82 - Mode Step
#define HID_CONSUMER_RECALL_LAST    131 // 0x83 - Recall Last
#define HID_CONSUMER_QUIT           148 // 0x94 - Quit
#define HID_CONSUMER_HELP           149 // 0x95 - Help
#define HID_CONSUMER_CHANNEL_UP     156 // 0x9C - Channel Increment
#define HID_CONSUMER_CHANNEL_DOWN   157 // 0x9D - Channel Decrement

#define HID_CONSUMER_PLAY           176 // 0xB0 - Play
#define HID_CONSUMER_PAUSE          177 // 0xB1 - Pause
#define HID_CONSUMER_RECORD         178 // 0xB2 - Record
#define HID_CONSUMER_FAST_FORWARD   179 // 0xB3 - Fast Forward
#define HID_CONSUMER_REWIND         180 // 0xB4 - Rewind
#define HID_CONSUMER_SCAN_NEXT_TRK  181 // 0xB5 - Scan Next Track
#define HID_CONSUMER_SCAN_PREV_TRK  182 // 0xB6 - Scan Previous Track
#define HID_CONSUMER_STOP           183 // 0xB7 - Stop
#define HID_CONSUMER_EJECT          184 // 0xB8 - Eject
#define HID_CONSUMER_RANDOM_PLAY    185 // 0xB9 - Random Play
#define HID_CONSUMER_SELECT_DISC    186 // 0xBA - Select Disk
#define HID_CONSUMER_ENTER_DISC     187 // 0xBB - Enter Disc
#define HID_CONSUMER_REPEAT         188 // 0xBC - Repeat
#define HID_CONSUMER_STOP_EJECT     204 // 0xCC - Stop/Eject
#define HID_CONSUMER_PLAY_PAUSE     205 // 0xCD - Play/Pause
#define HID_CONSUMER_PLAY_SKIP      206 // 0xCE - Play/Skip

#define HID_CONSUMER_VOLUME         224 // 0xE0 - Volume
#define HID_CONSUMER_BALANCE        225 // 0xE1 - Balance
#define HID_CONSUMER_MUTE           226 // 0xE2 - Mute
#define HID_CONSUMER_BASS           227 // 0xE3 - Bass
#define HID_CONSUMER_VOLUME_UP      233 // 0xE9 - Volume Increment
#define HID_CONSUMER_VOLUME_DOWN    234 // 0xEA - Volume Decrement

/*********************************************************************
 * TYPEDEFS
 */

// HID report mapping table
typedef struct
{
  uint16_t        handle;       // Handle of report characteristic
  gattAttribute_t *pCccdAttr;   // Pointer to CCCD attribute for report characteristic
  uint8_t         id;           // Report ID
  uint8_t         type;         // Report type
  uint8_t         mode;         // Protocol mode (report or boot)
} hidRptMap_t;

// HID dev configuration structure
typedef struct
{
  uint32_t    idleTimeout;      // Idle timeout in milliseconds
  uint8_t     hidFlags;         // HID feature flags
} hidDevCfg_t;

/*********************************************************************
 * Global Variables
 */

// These variables are defined in the service source file that uses HID Dev

// HID report map length
extern uint8_t hidReportMapLen;

// HID protocol mode
extern uint8_t hidProtocolMode;

/*********************************************************************
 * Profile Callbacks
 */

// HID Report callback
typedef uint8_t (*hidDevReportCB_t)(uint8_t id, uint8_t type, uint16_t uuid,
                                   uint8_t oper, uint16_t *pLen, uint8_t *pData);

// HID event callback
typedef void (*hidDevEvtCB_t)(uint8_t evt);

// HID passcode callback
typedef void (*hidDevPasscodeCB_t)(uint8_t  *deviceAddr,
                                   uint16_t connectionHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs);

typedef struct
{
  hidDevReportCB_t    reportCB;
  hidDevEvtCB_t       evtCB;
  hidDevPasscodeCB_t  passcodeCB;
} hidDevCB_t;


/*********************************************************************
 * @fn      HidDev_createTask
 *
 * @brief   Task creation function for the HID service.
 *
 * @param   none
 *
 * @return  none
 */
extern void HidDev_createTask(void);

/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg - Parameter configuration.
 * @param   pCBs - Callback function.
 *
 * @return  None.
 */
extern void HidDev_Register(hidDevCfg_t *pCfg, hidDevCB_t *pCBs);

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
extern void HidDev_RegisterReports(uint8_t numReports, hidRptMap_t *pRpt);

/*********************************************************************
 * @fn      HidDev_Report
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
extern void HidDev_createReport(uint8_t id, uint8_t type, uint8_t len,
                          uint8_t *pData);

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @return  None.
 */
extern void HidDev_Close(void);

/*********************************************************************
 * @fn      HidDev_SetParameter
 *
 * @brief   Set a HID Dev parameter.
 *
 * @param   param  - Profile parameter ID
 * @param   len    - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *                   the parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be cast to
 *                   uint16_t pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t HidDev_SetParameter(uint8_t param, uint8_t len, void *pValue);

/*********************************************************************
 * @fn      HidDev_GetParameter
 *
 * @brief   Get a HID Dev parameter.
 *
 * @param   param  - Profile parameter ID
 * @param   pValue - pointer to data to get.  This is dependent on
 *                   the parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be cast to
 *                   uint16_t pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t HidDev_GetParameter(uint8_t param, void *pValue);

/*********************************************************************
 * @fn      HidDev_PasscodeRsp
 *
 * @brief   Respond to a passcode request.
 *
 * @param   status   - SUCCESS if passcode is available, otherwise
 *                     see @ref SMP_PAIRING_FAILED_DEFINES.
 * @param   passcode - integer value containing the passcode.
 *
 * @return  none
 */
extern void HidDev_PasscodeRsp(uint8_t status, uint32_t passcode);

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
extern bStatus_t HidDev_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen,
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);

/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute read callback.
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
extern bStatus_t HidDev_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * @fn      HidDev_StartIdleTimer
 *
 * @brief   Start the idle timer.
 *
 * @return  None.
 */
extern void HidDev_StartIdleTimer(void);

/*********************************************************************
 * @fn      HidDev_StopIdleTimer
 *
 * @brief   Stop the idle timer.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDev_StopIdleTimer(void);

/*********************************************************************
 * @fn      HidDev_StartAdvertising
 *
 * @brief   Start advertising.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void HidDev_startAdvertising(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HIDDEV_H */
