/*!
 *		@file			  MB_Main.c
 *  	@author  		Dinh Le
 *		@copyright	Fiot Co.,Ltd
 *  	@version 		1.0
 *  	@date    		2016-04-12
 *		@brief			Manage on/off advertisement of BLE
 *							and control other layers
 */
/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/PIN/PINCC26XX.h>
#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "hal_mcu.h"
#include "MB_Service.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "osal_snv.h"
#include "icall_apimsg.h"
#include "util.h"
#include "MB_Uart.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC
#include "MB_Define.h"
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
//!Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
//!General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
//!Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
//!parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

//!Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
//!parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

#else //!FEATURE_OAD
//!Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
//!parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

//!Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
//!parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

//!Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

//!Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
//!update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

//!Whether to enable automatic parameter update request when a connection is
//!formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

//!Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

//!How often to perform periodic event (in msec)
#define MB_MAIN_PERIODIC_EVT_PERIOD               (1000)
#define MB_TIMEOUT_RESPONSE_CHAR                  (10000)
#define MB_TIMEOUT_INDICATION_CHAR                (10000)
#define MB_TIMEOUT_DATA1_CHAR                     (10000)
#define MB_TIMEOUT_DATA2_CHAR                     (10000)
#define MB_TIMEOUT_DATA3_CHAR                     (10000)
#define MB_TIMEOUT_DATA4_CHAR                     (10000)

//!Enable echo from app
//#define ENABLE_ECHO_APP
#ifdef FEATURE_OAD
//!The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

//!Task configuration
#define MB_MAIN_TASK_PRIORITY                     1
#define MB_MAIN_TASK_STACK_SIZE                   644

//!Internal Events for RTOS application
#define MB_MAIN_STATE_CHANGE_EVT                  0x0001
#define MB_MAIN_CHAR_CHANGE_EVT                   0x0002
#define MB_MAIN_UART_CHANGE_EVT                   0x0010
#define MB_MAIN_PERIODIC_EVT                      0x0004
#define MB_Main_CONN_EVT_END_EVT                  0x0008
   

/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/
//!App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  //!event header.
} sbpEvt_t;

typedef struct
{
  UART_Data_T bufferRes[PACKET_DATA_LENGTH];
  UART_Data_T bufferInd[PACKET_DATA_LENGTH];
  UART_Data_T bufferData1[PACKET_DATA_LENGTH];
  UART_Data_T bufferData2[PACKET_DATA_LENGTH];
  UART_Data_T bufferData3[PACKET_DATA_LENGTH];
  UART_Data_T bufferData4[PACKET_DATA_LENGTH];
} UART_Buffer_T;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************/
//!Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntityMain;

//!Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

//!Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct responseClock;
static Clock_Struct indicationClock;
static Clock_Struct data1Clock;
static Clock_Struct data2Clock;
static Clock_Struct data3Clock;
static Clock_Struct data4Clock;

//!Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
//!Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

//!events flag for internal application events.
static uint16_t events;

//!events when start device
static uint8_t startDevice;

//!Buffer to compare when receive UART
UART_Buffer_T bufferData;

//!Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[MB_MAIN_TASK_STACK_SIZE];

//!GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  //!complete name
  0x0B,   //!length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'M', 'o', 't', 'i', 'o', 'n', 'B', 'a', 'n', 'd',

  //!connection interval range
  0x05,   //!length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  //!Tx power level
  0x02,   //!length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       //!0dBm
};

//!GAP - Advertisement data (max size = 31 bytes, though this is
//!best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  //!Flags; this sets the device to use limited discoverable
  //!mode (advertises for 30 seconds at a time) instead of general
  //!discoverable mode (advertises indefinitely)
  0x02,   //!length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  //!service UUID, to notify central devices what services are included
  //!in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   //!length of this data
#else //OAD for external flash
  0x05,  //!lenght of this data
#endif //FEATURE_OAD

  GAP_ADTYPE_16BIT_MORE,      //!some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(MB_SERV_UUID),
  HI_UINT16(MB_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

//Set value for DevInfo
static const uint8_t devInfoModelNumber[] = "MotionBand";
static const uint8_t devInfoFirmwareRev[] = FW_VERSION_STR;
static const uint8_t devInfoHardwareRev[] = "Hardware 1.0";

//!GAP GATT Attributes
static const uint8_t *attDeviceName = devInfoModelNumber; //!GAP_DEVICE_NAME_LEN

//!Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/********************************************************************************
 * LOCAL FUNCTIONS
 ********************************************************************************/
static void MB_InitTask( void );
static void MB_FxnTask(UArg a0, UArg a1);

static uint8_t MB_processStackMsg(ICall_Hdr *pMsg);
static uint8_t MB_processGATTMsg(gattMsgEvent_t *pMsg);
static void MB_processAppMsg(sbpEvt_t *pMsg);
static void MB_processStateChangeEvt(gaprole_States_t newState);
static void MB_processCharValueChangeEvt(uint8_t paramID);
static void MB_performPeriodicTask(void);
static void MB_clockHandler(UArg arg);
static void MB_sendAttRsp(void);
static void MB_freeAttRsp(uint8_t status);
static void MB_stateChangeCB(gaprole_States_t newState);
void MB_enqueueMsg(uint8_t event, uint8_t state);
static void MB_handleUART(uint8_t paramID);
#ifdef FEATURE_OAD
void MB_processOadWriteCB(uint8_t event, uint16_t connHandle, uint8_t *pData);
#endif //FEATURE_OAD
static void MB_setDeviceInfo(void);
/********************************************************************************
 * PROFILE CALLBACKS
 ********************************************************************************/
//!GAP Role Callbacks
static gapRolesCBs_t MB_gapRoleCBs =
{
  MB_stateChangeCB     //!Profile State Change Callbacks
};

//!GAP Bond Manager Callbacks
static gapBondCBs_t MotionBand_BondMgrCBs =
{
  NULL, //!Passcode callback (not used by application)
  NULL  //!Pairing / Bonding state Callback (not used by application)
};
//!Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static motionBandCBs_t MB_simpleProfileCBs =
{
  MB_charValueChangeCB, //!Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t MotionBand_oadCBs =
{
  MB_processOadWriteCB //!Write Callback.
};
#endif //FEATURE_OAD
/********************************************************************************
 * FUNCTIONS - APIs
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      MB_CreateTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 ********************************************************************************/
void MB_CreateTask( void )
{
  Task_Params taskParams;

  //!Configure task
  Task_Params_init( &taskParams );
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = MB_MAIN_TASK_STACK_SIZE;
  taskParams.priority = MB_MAIN_TASK_PRIORITY;

  Task_construct( &sbpTask, MB_FxnTask, &taskParams, NULL );
}
/*!
 ********************************************************************************
 * @fn      MB_InitTask
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 ********************************************************************************/
static void MB_InitTask( void )
{
  //!******************************************************************
  //!N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  //!******************************************************************
  //!Register the current thread as an ICall dispatcher application
  //!so that the application can send and receive messages.
  ICall_registerApp( &selfEntityMain, &sem );

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  //!Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue( &appMsg );

  //!Create one-shot clocks for internal periodic events.
  Util_constructClock( &periodicClock, MB_clockHandler,
                      MB_MAIN_PERIODIC_EVT_PERIOD, 0, false, MB_MAIN_PERIODIC_EVT );

  //!Setup the GAP
  GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  //!Setup the GAP Peripheral Role Profile
  {
    //!For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    //!By setting this to zero, the device will go into the waiting state after
    //!being discoverable for 30.72 second, and will not being advertising again
    //!until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    //!Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof(advertData), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout );
  }

  //!Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(devInfoModelNumber)/sizeof(uint8_t), (void *)attDeviceName );

  //!Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  //!Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; //!passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),&passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, 		sizeof(uint8_t), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, 	sizeof(uint8_t), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, 	sizeof(uint8_t), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, 	sizeof(uint8_t), &bonding );
  }

  //!Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           //!GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   //!GATT attributes
  DevInfo_AddService();                        //!Device Information Service
  MB_AddService(GATT_ALL_SERVICES);
#ifdef FEATURE_OAD
  VOID OAD_addService();                 //!OAD Profile
  OAD_register((oadTargetCBs_t *)&MotionBand_oadCBs);
  hOadQ = Util_constructQueue( &oadQ );
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  //!Setup the MotionBand Characteristic Values
  {
    uint8_t charValue1[MB_REQUEST_CHAR_LEN] = {0x00};
    MB_SetParameter( MB_REQUEST_CHAR, MB_REQUEST_CHAR_LEN, charValue1 );
  }
  //!Setup start device
  startDevice = 0;
  
  //!Add application specific device information
  MB_setDeviceInfo();

  //!Register callback with MB_Board
  MB_RegisterAppCBs( &MB_simpleProfileCBs );
  //!Start the Device
  VOID GAPRole_StartDevice( &MB_gapRoleCBs );

  //!Start Bond Manager
  VOID GAPBondMgr_Register( &MotionBand_BondMgrCBs );

  //!Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs( selfEntityMain );

  HCI_LE_ReadMaxDataLenCmd();

  //!Setting Clock for 4 characteristic of GATT
  Util_constructClock( &responseClock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_RESPONSE_CHAR, 0, false, NULL );
  Util_constructClock( &indicationClock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_INDICATION_CHAR, 0, false, NULL );
  Util_constructClock( &data1Clock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_DATA1_CHAR, 0, false, NULL );
  Util_constructClock( &data2Clock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_DATA2_CHAR, 0, false, NULL );
  Util_constructClock( &data3Clock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_DATA3_CHAR, 0, false, NULL );
  Util_constructClock( &data4Clock, MB_PACKET_Write_Error_TO,
                      MB_TIMEOUT_DATA4_CHAR, 0, false, NULL );
  Util_startClock( &periodicClock );
}
/*!
 ********************************************************************************
 * @fn      MB_FxnTask
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
********************************************************************************/
static void MB_FxnTask( UArg a0, UArg a1 )
{
  //!Initialize application
  MB_InitTask();

  //!Application main loop
  for (;;)
  {
    //!Waits for a signal to the semaphore associated with the calling thread.
    //!Note that the semaphore associated with a thread is signaled when a
    //!message is queued to the message receive queue of the thread or when
    //!ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait( ICALL_TIMEOUT_FOREVER );

    if ( errno == ICALL_ERRNO_SUCCESS )
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if ( ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS )
      {
        uint8 safeToDealloc = TRUE;

        if ( (src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntityMain) )
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          //!Check for BLE stack events first
          if ( pEvt->signature == 0xffff )
          {
            if ( pEvt->event_flag & MB_Main_CONN_EVT_END_EVT )
            {
              //!Try to retransmit pending ATT Response (if any)
              MB_sendAttRsp();
            }
          }
          else
          {
            //!Process inter-task message
            safeToDealloc = MB_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if ( pMsg && safeToDealloc )
        {
          ICall_freeMsg( pMsg );
        }
      }

      //!If RTOS queue is not empty, process app message.
      while ( !Queue_empty(appMsgQueue) )
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg( appMsgQueue );
        if ( pMsg )
        {
          //!Process message.
          MB_processAppMsg( pMsg );

          //!Free the space from the message.
          ICall_free( pMsg );
        }
      }
    }

    if ( events & MB_MAIN_PERIODIC_EVT )
    {
      events &= ~MB_MAIN_PERIODIC_EVT;
      Util_startClock( &periodicClock );
      //!Perform periodic application task
      MB_performPeriodicTask();
    }
#ifdef FEATURE_OAD
    while ( !Queue_empty(hOadQ) )
    {
      oadTargetWrite_t *oadWriteEvt = Queue_get( hOadQ );

      //!Identify new image.
      if ( oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ )
      {
        OAD_imgIdentifyWrite( oadWriteEvt->connHandle, oadWriteEvt->pData );
      }
      //!Write a next block request.
      else if ( oadWriteEvt->event == OAD_WRITE_BLOCK_REQ )
      {
        OAD_imgBlockWrite( oadWriteEvt->connHandle, oadWriteEvt->pData );
      }

      //!Free buffer.
      ICall_free( oadWriteEvt );
    }
#endif //FEATURE_OAD
  }
}
/*!
 ********************************************************************************
 * @fn      MB_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
********************************************************************************/
static uint8_t MB_processStackMsg( ICall_Hdr *pMsg )
{
  uint8_t safeToDealloc = TRUE;

  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
    {
    //!Process GATT message
    safeToDealloc = MB_processGATTMsg((gattMsgEvent_t *)pMsg);
    break;
    }
    case HCI_GAP_EVENT_EVENT:
    {
      //!Process HCI message
      switch( pMsg->status )
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          //!Process HCI Command Complete Event
          break;

        default:
          break;
      }
    }
    break;

    default:
      //!do nothing
      break;
  }
  return ( safeToDealloc );
}
/*!
 ********************************************************************************
 * @fn      MB_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
********************************************************************************/
static uint8_t MB_processGATTMsg(gattMsgEvent_t *pMsg)
{
  //!See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    //!No HCI buffer was available. Let's try to retransmit the response
    //!on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntityMain,
                                   MB_Main_CONN_EVT_END_EVT) == SUCCESS)
    {
      //!First free any pending response
      MB_freeAttRsp(FAILURE);

      //!Hold on to the response message for retransmission
      pAttRsp = pMsg;

      //!Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    //!ATT request-response or indication-confirmation flow control is
    //!violated. All subsequent ATT requests or indications will be dropped.
    //!The app is informed in case it wants to drop the connection.

    //!Display the opcode of the message that caused the violation.
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    //!MTU size updated
  }

  //!Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  //!It's safe to free the incoming message
  return (TRUE);
}
/*!
 ********************************************************************************
 * @fn      MB_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
********************************************************************************/
static void MB_sendAttRsp(void)
{
  //!See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    //!Increment retransmission count
    rspTxRetry++;

    //!Try to retransmit ATT response till either we're successful or
    //!the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      //!Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntityMain, 0);

      //!We're done with the response message
      MB_freeAttRsp(status);
    }
    else
    {
      //!Continue retrying
    }
  }
}
/*!
 ********************************************************************************
 * @fn      MB_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
********************************************************************************/
static void MB_freeAttRsp(uint8_t status)
{
  //!See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    //!See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
      //!Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    //!Free response message
    ICall_freeMsg(pAttRsp);

    //!Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}
/*!
 ********************************************************************************
 * @fn      MB_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
********************************************************************************/
static void MB_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case MB_MAIN_STATE_CHANGE_EVT:
      MB_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case MB_MAIN_CHAR_CHANGE_EVT:
      MB_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    case MB_MAIN_UART_CHANGE_EVT:
      MB_handleUART(pMsg->hdr.state);
      break;
    default:
      //!Do nothing.
      break;
  }
}
/*!
 ********************************************************************************
 * @fn      MB_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
********************************************************************************/
static void MB_stateChangeCB(gaprole_States_t newState)
{
  MB_enqueueMsg(MB_MAIN_STATE_CHANGE_EVT, newState);
}
/*!
 ********************************************************************************
 * @fn      MB_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
********************************************************************************/
static void MB_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
    {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
        //!use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];
        //!set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;
        //!shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
    }
    break;

    case GAPROLE_ADVERTISING:
    {
      if(startDevice)
      {
        //!Send data to STM when BLE Peripheral advertise
        UART_Data_T data = PACKET_STATUS_DATA_NOTCONNECT;
        MB_PACKET_Write(PACKET_CMD_STATUS_BLE, PACKET_STATUS_RES_LENGTH, &data);
      }
      startDevice = 1;
    }
    break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
    {
        uint8_t advertEnabled = FALSE;
        //!Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
        
        advertEnabled = TRUE;
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        // Reset flag for next connection.
        firstConnFlag = false;
        MB_freeAttRsp(bleNotConnected);
    }
    break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
    {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        Util_startClock(&periodicClock);
        //!Send data to STM when BLE Peripheral Connected
        UART_Data_T data = PACKET_STATUS_DATA_CONNECTED;
        MB_PACKET_Write(PACKET_CMD_STATUS_BLE, PACKET_STATUS_RES_LENGTH, &data);
        //!Use numActive to determine the connection handle of the last
        //!connection
        numActive = linkDB_NumActive();
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];
          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        }

        #ifdef PLUS_BROADCASTER
          //!Only turn advertising on for this state when we first connect
          //!otherwise, when we go from connected_advertising back to this state
          //!we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; //!Turn on Advertising
            //!Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            //!Set to true for non-connectabel advertising.
            advertEnabled = TRUE;
            //!Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
    }
    break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      MB_freeAttRsp(bleNotConnected);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      MB_freeAttRsp(bleNotConnected);
      #ifdef PLUS_BROADCASTER
        //!Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      break;

    default:
      break;
  }
}
/*!
 *******************************************************************************
 * @fn      MB_charValueChangeCB
 *
 * @brief   Callback from Sensor Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
void MB_charValueChangeCB(uint8_t paramID)
{
  MB_enqueueMsg(MB_MAIN_CHAR_CHANGE_EVT, paramID);
}
/*!
 ********************************************************************************
 * @fn      MB_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
********************************************************************************/
static void MB_processCharValueChangeEvt(uint8_t paramID)
{
  UART_Data_T valuechar[PACKET_DATA_LENGTH] = {0x00};
  switch(paramID)
  {
    case MB_REQUEST_CHAR:
    {
      MB_GetParameter(MB_REQUEST_CHAR, valuechar);
      MB_PACKET_Write(PACKET_CMD_REQ, g_LengthDataReceiveApp, valuechar);
    }
    break;
#ifdef ENABLE_ECHO_APP
    case MB_RESPONSE_CHAR:
    {
      if (Util_isActive(&responseClock))
      {
        MB_GetParameter(MB_RESPONSE_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferRes, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_RES, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&responseClock);
      }
    }
    break;
    case MB_INDICATION_CHAR:
    {
      if (Util_isActive(&indicationClock))
      {
        MB_GetParameter(MB_INDICATION_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferInd, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_IND, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&indicationClock);
      }
    }
    break;
    case MB_DATA1_CHAR:
    {
      if (Util_isActive(&data1Clock))
      {
        MB_GetParameter(MB_DATA1_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferData1, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_DATA1, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&data1Clock);
      }
    }
    break;
    case MB_DATA2_CHAR:
    {
      if (Util_isActive(&data2Clock))
      {
        MB_GetParameter(MB_DATA2_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferData2, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_DATA2, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&data2Clock);
      }
    }
    break;
    case MB_DATA3_CHAR:
    {
      if (Util_isActive(&data3Clock))
      {
        MB_GetParameter(MB_DATA3_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferData3, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_DATA3, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&data3Clock);
      }
    }
    break;
    case MB_DATA4_CHAR:
    {
      if (Util_isActive(&data4Clock))
      {
        MB_GetParameter(MB_DATA4_CHAR, valuechar);
        if (!memcmp(valuechar, bufferData.bufferData4, PACKET_DATA_LENGTH))
        {
          MB_PACKET_Write(PACKET_CMD_DATA4, g_LengthDataReceiveApp, valuechar);
        }
        else
        {
          MB_PACKET_Write_Error_TO();
        }
        Util_stopClock(&data4Clock);
      }
    }
    break;
#endif /* ENABLE_ECHO_APP */
    default:
    break;
  }
}
/*!
 ********************************************************************************
 * @fn      MB_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (MB_MAIN_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
********************************************************************************/
static void MB_performPeriodicTask(void)
{
  return;
}
#ifdef FEATURE_OAD
/*!
 ********************************************************************************
 * @fn      MB_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
********************************************************************************/
void MB_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD
/*!
 ********************************************************************************
 * @fn      MB_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
********************************************************************************/
static void MB_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}
/*!
 ********************************************************************************
 * @fn      MB_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
********************************************************************************/
void MB_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}
/*!
 *******************************************************************************
 * @fn      MB_setDeviceInfo
 *
 * @brief   Set application specific Device Information
 *
 * @param   none
 *
 * @return  none
 ********************************************************************************/
static void MB_setDeviceInfo(void)
{
  DevInfo_SetParameter(DEVINFO_FIRMWARE_REV, sizeof(devInfoFirmwareRev),
                       (void*)devInfoFirmwareRev);
  DevInfo_SetParameter(DEVINFO_HARDWARE_REV, sizeof(devInfoHardwareRev),
                       (void*)devInfoHardwareRev);
}
/*!
 *******************************************************************************
 * @fn      MB_handleUART
 *
 * @brief   Set application specific when receive data from UART
 *
 * @param   paramID
 *
 * @return  none
 ********************************************************************************/
void MB_handleUART(uint8_t paramID)
{
  UART_Data_T pData[PACKET_DATA_LENGTH] = {0x00}; 
  UART_Data_T pLen = 0;
  //!Read packet
  MB_PACKET_Read(pData,&pLen);
  //!Read status of BLE connection
  gaprole_States_t status;
  GAPRole_GetParameter(GAPROLE_STATE, &status);
  //!Handle command
  switch(paramID)
  {
    //! Command check status of BLE
    case PACKET_CMD_STATUS_BLE:
    {
      //!Check value of datalen
      if (pLen != PACKET_STATUS_REQ_LENGTH)
      {
        MB_PACKET_Write_Error();
      }
      else
      {
        if (status == GAPROLE_CONNECTED)
        {
          UART_Data_T data = PACKET_STATUS_DATA_CONNECTED;
          MB_PACKET_Write(PACKET_CMD_STATUS_BLE, PACKET_STATUS_RES_LENGTH, &data);
        }
        else
        {
          UART_Data_T data = PACKET_STATUS_DATA_NOTCONNECT;
          MB_PACKET_Write(PACKET_CMD_STATUS_BLE, PACKET_STATUS_RES_LENGTH, &data);
        }
      }
    }
    break;
      
    //!Command disable BLE connection 
    case PACKET_CMD_DIS_BLE:
    {
      if (pLen != PACKET_DIS_REQ_LENGTH)
      {
        MB_PACKET_Write_Error();
      }
      else
      {
        if (status == GAPROLE_CONNECTED)
        {
          GAPRole_TerminateConnection();
          UART_Data_T data = PACKET_DIS_DATA_SUCCESS;
          MB_PACKET_Write(PACKET_CMD_DIS_BLE, PACKET_DIS_RES_LENGTH, &data);
        }
        else
        {
          UART_Data_T data = PACKET_DIS_DATA_SUCCESS;
          MB_PACKET_Write(PACKET_CMD_DIS_BLE, PACKET_DIS_RES_LENGTH, &data);
        }
      }
    }
    break;
    
    case PACKET_CMD_ENA_AD:
    { 
      if (pLen != PACKET_ENA_DIS_AD_REQ_LENGTH)
      {
        MB_PACKET_Write_Error();
      }
      else
      {
        if (pData[PACKET_DATA_FIRSTBYTE] == PACKET_ENA_AD_DATA)
        {
            uint8_t advertenable = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertenable);
            UART_Data_T data = PACKET_ENA_AD_DATA;
            MB_PACKET_Write(PACKET_CMD_ENA_AD, PACKET_ENA_DIS_AD_RES_LENGTH, &data);
        }
        else if (pData[PACKET_DATA_FIRSTBYTE] == PACKET_DIS_AD_DATA)
        {
            uint8_t advertdisable = FALSE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertdisable);
            UART_Data_T data = PACKET_DIS_AD_DATA;
            MB_PACKET_Write(PACKET_CMD_ENA_AD, PACKET_ENA_DIS_AD_RES_LENGTH, &data);
            //! Do not response Status of BLE automatically when not advertising -> advertising 
            startDevice = 0;
        }
        else 
        { 
            MB_PACKET_Write_Error();
        }
      }
    }
    break;

    case PACKET_CMD_RES:
    {      
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_RESPONSE_CHAR, pLen, pData);
        VOID memset(bufferData.bufferRes, 0x00, sizeof(bufferData.bufferRes));
        memcpy(bufferData.bufferRes, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&responseClock);
#else
        MB_PACKET_Write(PACKET_CMD_RES, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);
      }
    }
    break;
    
    case PACKET_CMD_IND:
    {
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_INDICATION_CHAR, pLen, pData);
        VOID memset(bufferData.bufferInd, 0x00, sizeof(bufferData.bufferInd));
        memcpy(bufferData.bufferInd, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&indicationClock);
#else
        MB_PACKET_Write(PACKET_CMD_IND, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);
      } 
    }
    break;
    
    case PACKET_CMD_DATA1:
    {
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_DATA1_CHAR, pLen, pData);
        VOID memset(bufferData.bufferData1, 0x00, sizeof(bufferData.bufferData1));
        memcpy(bufferData.bufferData1, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&data1Clock);
#else
        MB_PACKET_Write(PACKET_CMD_DATA1, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);      
      }
    }
    break;
    
    case PACKET_CMD_DATA2:
    {
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_DATA2_CHAR, pLen, pData);
        VOID memset(bufferData.bufferData2, 0x00, sizeof(bufferData.bufferData2));
        memcpy(bufferData.bufferData2, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&data2Clock);
#else
        MB_PACKET_Write(PACKET_CMD_DATA2, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);         
      }
    }
    break;
    
    case PACKET_CMD_DATA3:
    {
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_DATA3_CHAR, pLen, pData);
        VOID memset(bufferData.bufferData3, 0x00, sizeof(bufferData.bufferData3));
        memcpy(bufferData.bufferData3, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&data3Clock);
#else
        MB_PACKET_Write(PACKET_CMD_DATA3, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);         
      }
    }
    break;
    
    case PACKET_CMD_DATA4:
    {
      if (status == GAPROLE_CONNECTED)
      {
        MB_SetParameter(MB_DATA4_CHAR, pLen, pData);
        VOID memset(bufferData.bufferData4, 0x00, sizeof(bufferData.bufferData4));
        memcpy(bufferData.bufferData4, pData, pLen);
#ifdef  ENABLE_ECHO_APP
        Util_startClock(&data4Clock);
#else
        MB_PACKET_Write(PACKET_CMD_DATA4, pLen, pData);
#endif  /* ENABLE_ECHO_APP */
      }
      else
      {
        UART_Data_T data = PACKET_ERROR_BLE_DATA;
        MB_PACKET_Write(PACKET_CMD_ERROR_BLE, PACKET_ERROR_BLE_LENGTH, &data);         
      }
    }
    break;
    
    default:
      break;
  }
}
/********************************************************************************
 * END
 ********************************************************************************/
