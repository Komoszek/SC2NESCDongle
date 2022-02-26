/******************************************************************************

 @file  SC2NESCDongle.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2541

 ******************************************************************************

 Copyright (c) 2010-2019, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com. */

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_uart.h"
#include "hal_key.h"
#include "hal_led.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"

#include "SC2NESCDongle.h"
#include "scbt.h"
#include "wiimote.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  6

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 3000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Application states
enum
{
    BLE_STATE_IDLE,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
};


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 SC2NESCDongleTaskId;

// GAP GATT Attributes
static const uint8 SC2NESCDongleDeviceName[GAP_DEVICE_NAME_LEN] = "SC Adapter";

// Number of scan results and scan result index
static uint8 SC2NESCScanRes;

// Scan result list
static gapDevRec_t SC2NESCDongleDevList[DEFAULT_MAX_SCAN_RES];

// Connection handle of current connection
static uint16 SC2NESCDongleConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 SC2NESCDongleState = BLE_STATE_IDLE;

// Notification enabled
static bool SCNotification = FALSE;

// Magic command that enables all of the SC functionality
const static uint8 LizardModeCode[] = { 0xc0, 0x87, 0x0f, 0x18, 0x00, 0x00, 0x31, 0x02, 0x00, 0x08, 0x07, 0x00, 0x07, 0x07, 0x00, 0x30, 0x00, 0x00, 0x00};

//  Length of steam controller messages

#define STEAM_CON_MSG_LEN             19

static struct SCByBtC SCData;

static struct wiicontrolls WiiControlls = {0};

void stream_callback(uint8 *buffer) {
  wiimote_write_buffer(buffer, WiiControlls.lx, WiiControlls.rx, WiiControlls.ly, WiiControlls.ry, WiiControlls.ltrig, WiiControlls.rtrig, WiiControlls.raw[0], WiiControlls.raw[1]);
}



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SC2NESCDongleProcessGATTMsg( gattMsgEvent_t *pMsg );
static uint8 SC2NESCDongleEventCB( gapCentralRoleEvent_t *pEvent );
static void SC2NESCDonglePairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void SC2NESCDongle_HandleKeys( uint8 shift, uint8 keys );
static void SC2NESCDongle_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void BLE_StartDeviceDiscovery( void );

char *bdAddr2Str ( uint8 *pAddr );
char *uuid2Str ( uint8 uuid);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t SC2NESCDongleRoleCB =
{
    NULL,       // RSSI callback
    SC2NESCDongleEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t SC2NESCDongleBondCB =
{
    NULL,
    SC2NESCDonglePairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void SerialInterface_Init( void )
{
    HalUARTInit();

    //Configure UART And register callback function
    halUARTCfg_t uartConfig;

    // configure UART
    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = HAL_UART_BR_115200;
    uartConfig.flowControl          = FALSE;
    uartConfig.flowControlThreshold = MT_UART_THRESHOLD;
    uartConfig.rx.maxBufSize = MT_UART_RX_BUFF_MAX;
    uartConfig.tx.maxBufSize = MT_UART_TX_BUFF_MAX;
    uartConfig.idleTimeout = MT_UART_IDLE_TIMEOUT;
    uartConfig.intEnable = TRUE;
    uartConfig.callBackFunc = (halUARTCBack_t)serial_parser;

    // start UART
    // Note: Assumes no issue opening UART port.
    HalUARTOpen( UART_PORT, &uartConfig );
}


/*********************************************************************
 * @fn      SC2NESCDongle_Init
 *
 * @brief   Initialization function for the SC 2 NESC App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void SC2NESCDongle_Init( uint8 task_id )
{
    HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
    SerialInterface_Init();
    WiiControlls.lx = 0x7F;
    WiiControlls.rx = 0x7F;
    WiiControlls.ly = 0x7F;
    WiiControlls.ry = 0x7F;
    
    wiimote_stream = stream_callback;
    wiimote_init();
    
    

    SC2NESCDongleTaskId = task_id;
    
    // Setup Central Profile
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );

    // Setup GAP
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) SC2NESCDongleDeviceName );

    // Setup the GAP Bond Manager
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
    

    // Initialize GATT Client
    VOID GATT_InitClient();

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd( SC2NESCDongleTaskId );

    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );         // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

    // Register for all key events - This app will handle all key events
    RegisterForKeys( SC2NESCDongleTaskId );

    P0SEL = 0; // Configure Port 0 as GPIO
    P0DIR = 0xFE; // Port 0 pins P0.0 and P0.1 as input (buttons),
                  // all others (P0.2-P0.7) as output
    P0 = 0x01; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)

    // makes sure LEDs are off
    HalLedSet( (HAL_LED_2), HAL_LED_MODE_OFF );
    //HalLedBlink(HAL_LED_1, 0, 50, 500);

    // Setup a delayed profile startup
    osal_set_event( SC2NESCDongleTaskId, START_DEVICE_EVT );


}

/*********************************************************************
 * @fn      SC2NESCDongle_ProcessEvent
 *
 * @brief   SC 2 NESC Dongle Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SC2NESCDongle_ProcessEvent( uint8 task_id, uint16 events )
{

    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG ){
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( SC2NESCDongleTaskId )) != NULL ){
          SC2NESCDongle_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

          // Release the OSAL message
          VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & TIMER_EVT ){
        // return unprocessed events
        return (events ^ TIMER_EVT);
    }

    if ( events & START_DEVICE_EVT ){
        // Start the Device
        VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &SC2NESCDongleRoleCB );

        // Register with bond manager after starting device
        GAPBondMgr_Register( (gapBondCBs_t *) &SC2NESCDongleBondCB );

        return ( events ^ START_DEVICE_EVT );
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      SC2NESCDongle_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SC2NESCDongle_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
        case KEY_CHANGE:
            SC2NESCDongle_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
            break;
        case GATT_MSG_EVENT:
            SC2NESCDongleProcessGATTMsg( (gattMsgEvent_t *) pMsg );
            break;
    }
}

/*********************************************************************
 * @fn      SC2NESCDongle_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
static void SC2NESCDongle_HandleKeys( uint8 shift, uint8 keys )
{
    (void)shift;  // Intentionally unreferenced parameter

    HalUARTWrite(UART_PORT, "KeyPressed: ", 5);
    HalUARTWrite(UART_PORT, (uint8 *)uuid2Str(keys), 4);

    if ( keys & HAL_KEY_SW_1 ){
        osal_start_timerEx( SC2NESCDongleTaskId, TIMER_EVT, 5000 );
        HalLedSet( (HAL_LED_1), HAL_LED_MODE_ON );
    } else {
        osal_stop_timerEx(SC2NESCDongleTaskId, TIMER_EVT);
        HalLedSet( (HAL_LED_1), HAL_LED_MODE_OFF );

    }

}

/*********************************************************************
 * @fn      SC2NESCDongleProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void SC2NESCDongleProcessGATTMsg( gattMsgEvent_t *pMsg )
{
    if ( SC2NESCDongleState != BLE_STATE_CONNECTED ){
      // In case a GATT message came after a connection has dropped,
      // ignore the message
      return;
    }
    if ( ( pMsg->method == ATT_WRITE_RSP ) ||
        ( ( pMsg->method == ATT_ERROR_RSP ) &&
        ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) ){
        if ( !(pMsg->method == ATT_ERROR_RSP) ){
            if(!SCNotification){
                uint8 status;
                attWriteReq_t req;

                req.pValue = GATT_bm_alloc( SC2NESCDongleConnHandle, ATT_WRITE_REQ, 2, NULL );
                if ( req.pValue != NULL ){
                    req.handle = 0x002A; //0x0029+1
                    req.len = 2;
                    req.pValue[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);
                    req.pValue[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);
                    req.sig = 0;
                    req.cmd = 1;

                    status = GATT_WriteNoRsp( SC2NESCDongleConnHandle, &req );
                    
                    if ( status != SUCCESS ){
                        GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
                    } else {
                        SCNotification = TRUE;
                    }
                }
                else{
                    status = bleMemAllocError;
                }
            }
        }
    } else if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ){
          if (read_input(&SCData, pMsg->msg.handleValueNoti.pValue)){
              translate_input(&SCData, &WiiControlls);
          }
    }

    GATT_bm_free( &pMsg->msg, pMsg->method );
}


/*********************************************************************
 * @fn      SC2NESCDongleEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8 SC2NESCDongleEventCB( gapCentralRoleEvent_t *pEvent )
{
    switch( pEvent->gap.opcode ){
        case GAP_DEVICE_INIT_DONE_EVENT:
            BLE_StartDeviceDiscovery();
            break;
        case GAP_DEVICE_DISCOVERY_EVENT:
            {
                bool SCFound = FALSE;
                uint8 i;
                
                // Copy results
                SC2NESCScanRes = pEvent->discCmpl.numDevs;
                osal_memcpy( SC2NESCDongleDevList, pEvent->discCmpl.pDevList,
                         (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
                
                if ( SC2NESCScanRes > 0 ){
                    uint8 *peerAddr;
                    uint8 addrType;
                    for(i = 0; i < SC2NESCScanRes; i++){
                        if (SC2NESCDongleDevList[i].addr[5] == 0xCF){
                            // connect to current device in scan result
                            peerAddr = SC2NESCDongleDevList[i].addr;
                            addrType = SC2NESCDongleDevList[i].addrType;
                            SC2NESCDongleState = BLE_STATE_CONNECTING;

                            GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                          DEFAULT_LINK_WHITE_LIST,
                                                          addrType, peerAddr );

                            HalUARTWrite(UART_PORT, "Connecting\n", 11);
                            SCFound = TRUE;
                            break;
                        } else if (i + 1 == SC2NESCScanRes){
                            HalUARTWrite(UART_PORT, "SC not found\n", 13);
                        }
                    }

                }

                if (!SCFound){
                    HalUARTWrite(UART_PORT, "Rediscovering Devices\n", 22);
                    BLE_StartDeviceDiscovery();
                }
            }
            break;
        case GAP_LINK_ESTABLISHED_EVENT:
            {
                if ( pEvent->gap.hdr.status == SUCCESS ){
                  uint8 status;
                  attWriteReq_t req;
                  
                  SC2NESCDongleState = BLE_STATE_CONNECTED;
                  SC2NESCDongleConnHandle = pEvent->linkCmpl.connectionHandle;

                  HalUARTWrite(UART_PORT, "Connected\n", 10);
                  

                  req.pValue = GATT_bm_alloc( SC2NESCDongleConnHandle, ATT_WRITE_REQ, STEAM_CON_MSG_LEN, NULL );
                  if ( req.pValue != NULL )
                  {
                      req.handle = 0x002C;
                      req.len = STEAM_CON_MSG_LEN;
                      req.sig = 0;
                      req.cmd = 0;
                      osal_memcpy(req.pValue, LizardModeCode, STEAM_CON_MSG_LEN);

                      status = GATT_WriteCharDesc( SC2NESCDongleConnHandle, &req, SC2NESCDongleTaskId );
                      if ( status != SUCCESS ){
                          GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
                      }
                  } else {
                      status = bleMemAllocError;
                  }
                } else {
                    SC2NESCDongleState = BLE_STATE_IDLE;
                    SC2NESCDongleConnHandle = GAP_CONNHANDLE_INIT;
                }
            }
            break;
        case GAP_LINK_TERMINATED_EVENT:
            {
              SC2NESCDongleState = BLE_STATE_IDLE;
              SC2NESCDongleConnHandle = GAP_CONNHANDLE_INIT;
              SCNotification = FALSE;
              
              BLE_StartDeviceDiscovery();
            }
            break;
        default:
            break;
    }

    return TRUE;
}

/*********************************************************************
 * @fn      SC2NESCDonglePairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SC2NESCDonglePairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
    if ( state == GAPBOND_PAIRING_STATE_STARTED ){
        HalUARTWrite(UART_PORT, "Pairing started\n", 16);
    } else if ( state == GAPBOND_PAIRING_STATE_COMPLETE ){
        if ( status == SUCCESS ){
            HalUARTWrite(UART_PORT, "Pairing success\n", 16);
        } else {
            HalUARTWrite(UART_PORT, "Pairing fail\n", 13);
        }
    } else if ( state == GAPBOND_PAIRING_STATE_BONDED ){
        if ( status == SUCCESS ){
            HalUARTWrite(UART_PORT, "Bonding success\n", 16);
        }
    }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}

/*********************************************************************
*********************************************************************/


/*********************************************************************
 * @fn      uuid2Str
 *
 * @brief   Convert uint8 number to hex string
 *
 * @return  none
 */



/*********************************************************************
*********************************************************************/

static void BLE_StartDeviceDiscovery( void ){
        SC2NESCScanRes = 0;

        HalUARTWrite(UART_PORT, "Discovering...\n", 15);

        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );
}
