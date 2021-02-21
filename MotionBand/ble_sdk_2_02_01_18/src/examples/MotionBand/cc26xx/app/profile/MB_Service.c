/*! 
 *  @file	      MB_Service.c
 *  @author  	  Dinh Le
 *  @copyright	Fiot Co.,Ltd
 *  @version 	  1.0
 *  @date    	  2017-4-12
 *  @brief	    Service of MotionBand
 *
 */
/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include <string.h>
#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "MB_Service.h"
/********************************************************************************
 * MACROS
 ********************************************************************************/
#if (defined USING_UUID_128BIT)
#define UUID_SIZE 16
#else
#define UUID_SIZE 2
#endif
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
#define SERVAPP_NUM_ATTR_SUPPORTED        28
/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * EXTERNAL VARIABLES
 ********************************************************************************/
uint8_t g_LengthDataReceiveApp = 0;
/********************************************************************************
 * EXTERNAL FUNCTIONS
 ********************************************************************************/

/********************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************/
static motionBandCBs_t *motionBand_AppCBs = NULL;

#if (defined USING_UUID_128BIT)
//!Using UUID 128bit
//!MotionBand Service UUID 128bit: 0xFF00-7AA9-18E911E7-93AE-92361F002671
CONST uint8 motionBandServUUID[ATT_UUID_SIZE] =
{
  MB_UUID(MB_SERV_UUID)
};
//!Request Characteristic UUID 128bit: 0xFF01-7AA9-18E911E7-93AE-92361F002671
CONST uint8 requestCharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_REQUEST_CHAR_UUID)
};
//!Response Characteristic UUID 128bit: 0xFF02-7AA9-18E911E7-93AE-92361F002671
CONST uint8 responseCharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_RESPONSE_CHAR_UUID)
};
//!Indication Characteristic UUID 128bit: 0xFF03-7AA9-18E911E7-93AE-92361F002671 
CONST uint8 indicationCharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_INDICATION_CHAR_UUID)
};
//!Data1 Characteristic UUID 128bit: 0xFF04-7AA9-18E911E7-93AE-92361F002671
CONST uint8 data1CharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_DATA1_CHAR_UUID)
};
//!Data2 Characteristic UUID 128bit: 0xFF05-7AA9-18E911E7-93AE-92361F002671
CONST uint8 data2CharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_DATA2_CHAR_UUID)
};
//!Data3 Characteristic UUID 128bit: 0xFF06-7AA9-18E911E7-93AE-92361F002671
CONST uint8 data3CharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_DATA3_CHAR_UUID)
};
//!Data4 Characteristic UUID 128bit: 0xFF07-7AA9-18E911E7-93AE-92361F002671
CONST uint8 data4CharUUID[ATT_UUID_SIZE] =
{ 
  MB_UUID(MB_DATA4_CHAR_UUID)
};
#else
//!Using UUID 16bit
//!MotionBand Service UUID 16bit: 0xFF00
CONST uint8 motionBandServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_SERV_UUID), HI_UINT16(MB_SERV_UUID)
};
//!Request Characteristic UUID 16bit: 0xFF01
CONST uint8 requestCharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_REQUEST_CHAR_UUID), HI_UINT16(MB_REQUEST_CHAR_UUID)
};
//!Request Characteristic UUID 16bit: 0xFF02
CONST uint8 responseCharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_RESPONSE_CHAR_UUID), HI_UINT16(MB_RESPONSE_CHAR_UUID)
};
//!Indication Characteristic UUID 16bit: 0xFF03
CONST uint8 indicationCharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_INDICATION_CHAR_UUID), HI_UINT16(MB_INDICATION_CHAR_UUID)
};
//!Data1 Characteristic UUID 16bit: 0xFF04
CONST uint8 data1CharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_DATA1_CHAR_UUID), HI_UINT16(MB_DATA1_CHAR_UUID)
};
//!Data2 Characteristic UUID 16bit: 0xFF05
CONST uint8 data2CharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_DATA2_CHAR_UUID), HI_UINT16(MB_DATA2_CHAR_UUID)
};
//!Data3 Characteristic UUID 16bit: 0xFF06
CONST uint8 data3CharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_DATA3_CHAR_UUID), HI_UINT16(MB_DATA3_CHAR_UUID)
};
//!Data4 Characteristic UUID 16bit: 0xFF07
CONST uint8 data4CharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(MB_DATA4_CHAR_UUID), HI_UINT16(MB_DATA4_CHAR_UUID)
};
#endif
/********************************************************************************
 * MotionBand Attributes - variables
 ********************************************************************************/
//!MotionBand Service attribute
static CONST gattAttrType_t motionBandService = {UUID_SIZE, motionBandServUUID};

//!Request Characteristic Properties
static uint8 requestCharProps = GATT_PROP_READ | GATT_PROP_WRITE;
//!Request Characteristic Value
static uint8 requestChar[MB_REQUEST_CHAR_LEN] = {0x00};
//!RequestCharacteristic 1 User Description
static uint8 requestCharUserDesp[8] = "REQUEST";

//!Response Characteristic Properties
static uint8 responseCharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Response Characteristic Value
static uint8 responseChar[MB_RESPONSE_CHAR_LEN] = {0x00};
//!Response Characteristic User Description
static uint8 responseCharUserDesp[9] = "RESPONSE";
//!Response Characteristic Configuration
static gattCharCfg_t *responseCharConfig;      

//!Indication Characteristic Properties
static uint8 indicationCharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Indication Characteristic Value
static uint8 indicationChar[MB_INDICATION_CHAR_LEN] = {0x00};
//!Indication Characteristic User Description
static uint8 indicationCharUserDesp[11] = "INDICATION";
//!Indication Characteristic Configuration
static gattCharCfg_t *indicationCharConfig;   

//!Data1 Characteristic Properties
static uint8 data1CharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Data1 Characteristic Value
static uint8 data1Char[MB_DATA1_CHAR_LEN] = {0x00};                                     
//!Data1 Characteristic User Description
static uint8 data1CharUserDesp[6] = "DATA1";
//!Data1 Characteristic Configuration 
static gattCharCfg_t *data1CharConfig;   

//!Data2 Characteristic Properties
static uint8 data2CharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Data2 Characteristic Value
static uint8 data2Char[MB_DATA2_CHAR_LEN] = {0x00};
//!Data2 Characteristic User Description
static uint8 data2CharUserDesp[6] = "DATA2";
//!Data2 Characteristic Configuration 
static gattCharCfg_t *data2CharConfig; 

//!Data3 Characteristic Properties
static uint8 data3CharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Data3 Characteristic Value
static uint8 data3Char[MB_DATA3_CHAR_LEN] = {0x00};
//!Data3 Characteristic User Description
static uint8 data3CharUserDesp[6] = "DATA3";
//!Data3 Characteristic Configuration 
static gattCharCfg_t *data3CharConfig; 

//!Data5 Characteristic Properties
static uint8 data4CharProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE;
//!Data5 Characteristic Value
static uint8 data4Char[MB_DATA4_CHAR_LEN] = {0x00};
//!Data5 Characteristic User Description
static uint8 data4CharUserDesp[6] = "DATA4";
//!Data5 Characteristic Configuration 
static gattCharCfg_t *data4CharConfig; 
/********************************************************************************
 * MotionBand Attributes - Table
 ********************************************************************************/
static gattAttribute_t motionBandAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  //!MotionBand Service
  { 
    {ATT_BT_UUID_SIZE, primaryServiceUUID},   /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&motionBandService               /* pValue */
  },
    //!Request Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &requestCharProps 
    },
      //!Request Characteristic Value
      { 
        { UUID_SIZE, requestCharUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        requestChar 
      },
      //!Request Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        requestCharUserDesp 
      },      
    //!Response Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &responseCharProps 
    },
      //!Response Characteristic Value
      { 
        { UUID_SIZE, responseCharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        responseChar
      },
      //!Response Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&responseCharConfig 
      },
      //!Response Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        responseCharUserDesp 
      },           
    //!Indication Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &indicationCharProps 
    },
      //!Indication Characteristic Value 
      { 
        { UUID_SIZE, indicationCharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        indicationChar
      },
      //!Indication Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&indicationCharConfig 
      },
      //!Indication Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        indicationCharUserDesp 
      },
    //!Data1 Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &data1CharProps 
    },
      //!Data1 Characteristic Value
      { 
        { UUID_SIZE, data1CharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        data1Char
      },
      //!Data1 Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&data1CharConfig 
      },
      //!Data1 Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        data1CharUserDesp 
      },
    //!Data2 Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &data2CharProps 
    },
      //!Data2 Characteristic Value
      { 
        { UUID_SIZE, data2CharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        data2Char
      },
      //!Data2 Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&data2CharConfig 
      },
      //!Data2 Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        data2CharUserDesp 
      },
    //!Data3 Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &data3CharProps 
    },
      //!Data3 Characteristic Value
      { 
        { UUID_SIZE, data3CharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        data3Char
      },
      //!Data3 Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&data3CharConfig 
      },
      //!Data3 Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        data3CharUserDesp 
      },
    //!Data4 Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &data4CharProps 
    },
      //!Data4 Characteristic Value
      { 
        { UUID_SIZE, data4CharUUID },
        GATT_PROP_NOTIFY | GATT_PROP_WRITE, 
        0, 
        data4Char
      },
      //!Data4 Characteristic Configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&data4CharConfig 
      },
      //!Data4 Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        data4CharUserDesp 
      },
};
/********************************************************************************
 * LOCAL FUNCTIONS
 ********************************************************************************/
static bStatus_t MB_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr, 
                                uint8_t *pValue, uint16_t *pLen, uint16_t offset, 
                                uint16_t maxLen, uint8_t method );
static bStatus_t MB_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                 uint8_t *pValue, uint16_t len, uint16_t offset, 
                                 uint8_t method );
/********************************************************************************
 * MOTIONBAND CALLBACKS
 ********************************************************************************/
CONST gattServiceCBs_t MB_CallBacks =
{
  MB_ReadAttrCB,  // Read callback function pointer
  MB_WriteAttrCB, // Write callback function pointer
  NULL            // Authorization callback function pointer
};
/********************************************************************************
 * PUBLIC FUNCTIONS
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      MB_AddService
 *
 * @brief   Initializes the MotionBand service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 ********************************************************************************/ 
bStatus_t MB_AddService( uint32 services )
{
  uint8 status;
  //!Allocate Client Characteristic Configuration table
  responseCharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( responseCharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  indicationCharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( indicationCharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  data1CharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( data1CharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  data2CharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( data2CharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  data3CharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( data3CharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  data4CharConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                            linkDBNumConns);
  if ( data4CharConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  //!Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, responseCharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, indicationCharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, data1CharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, data2CharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, data3CharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, data4CharConfig );
  
  if ( services & MB_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( motionBandAttrTbl, 
                                         GATT_NUM_ATTRS( motionBandAttrTbl ),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &MB_CallBacks );
  }
  else
  {
    status = SUCCESS;
  }
  return ( status );
}
/*!
 ********************************************************************************
 * @fn      MB_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 ********************************************************************************/ 
bStatus_t MB_RegisterAppCBs( motionBandCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    motionBand_AppCBs = appCallbacks;
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
/*!
 ********************************************************************************
 * @fn      MB_SetParameter
 *
 * @brief   Set a MotionBand parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 ********************************************************************************/ 
bStatus_t MB_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MB_REQUEST_CHAR:
      if ( len <= MB_REQUEST_CHAR_LEN )
      {
        VOID memset( requestChar, 0x00, MB_REQUEST_CHAR_LEN );
        VOID memcpy( requestChar, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case MB_RESPONSE_CHAR:
      if ( len <= MB_RESPONSE_CHAR_LEN ) 
      {
        VOID memset( responseChar, 0x00, MB_RESPONSE_CHAR_LEN );
        VOID memcpy( responseChar, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( responseCharConfig, responseChar, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case MB_INDICATION_CHAR:
      if ( len <= MB_INDICATION_CHAR_LEN ) 
      {
        VOID memset( indicationChar, 0x00, MB_INDICATION_CHAR_LEN );
        VOID memcpy( indicationChar, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( indicationCharConfig, indicationChar, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MB_DATA1_CHAR:
      if ( len <= MB_DATA1_CHAR_LEN )
      {
        VOID memset( data1Char, 0x00, MB_DATA1_CHAR_LEN );
        VOID memcpy( data1Char, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( data1CharConfig, data1Char, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MB_DATA2_CHAR:
      if ( len <= MB_DATA2_CHAR_LEN ) 
      {
        VOID memset( data2Char, 0x00, MB_DATA2_CHAR_LEN );
        VOID memcpy( data2Char, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( data2CharConfig, data2Char, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case MB_DATA3_CHAR:
      if ( len <= MB_DATA3_CHAR_LEN ) 
      {
        VOID memset( data3Char, 0x00, MB_DATA3_CHAR_LEN );
        VOID memcpy( data3Char, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( data3CharConfig, data3Char, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case MB_DATA4_CHAR:
      if ( len <= MB_DATA4_CHAR_LEN ) 
      {
        VOID memset( data4Char, 0x00, MB_DATA4_CHAR_LEN );
        VOID memcpy( data4Char, value, len );
        //!See if Notification has been enabled
        GATTServApp_ProcessCharCfg( data4CharConfig, data4Char, FALSE, motionBandAttrTbl, 
                                    GATT_NUM_ATTRS( motionBandAttrTbl ), INVALID_TASK_ID, 
                                    MB_ReadAttrCB );
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
  return ( ret );
}
/*!
 ********************************************************************************
 * @fn      MB_GetParameter
 *
 * @brief   Get a MotionBand parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 ********************************************************************************/ 
bStatus_t MB_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MB_REQUEST_CHAR:
      VOID memcpy( value, requestChar, MB_REQUEST_CHAR_LEN );
      break;

    case MB_RESPONSE_CHAR:
      VOID memcpy( value, responseChar, MB_RESPONSE_CHAR_LEN );
      break;      

    case MB_INDICATION_CHAR:
      VOID memcpy( value, indicationChar, MB_INDICATION_CHAR_LEN );
      break;  

    case MB_DATA1_CHAR:
      VOID memcpy( value, data1Char, MB_DATA1_CHAR_LEN );
      break;

    case MB_DATA2_CHAR:
      VOID memcpy( value, data2Char, MB_DATA2_CHAR_LEN );
      break;      
      
    case MB_DATA3_CHAR:
      VOID memcpy( value, data3Char, MB_DATA3_CHAR_LEN );
      break;
      
    case MB_DATA4_CHAR:
      VOID memcpy( value, data4Char, MB_DATA4_CHAR_LEN );
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}
/*!
 ********************************************************************************
 * @fn          MB_ReadAttrCB
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
 ********************************************************************************/ 
static bStatus_t MB_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                               uint8_t *pValue, uint16_t *pLen, uint16_t offset, 
                               uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;
  //!Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    //!Using 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case MB_REQUEST_CHAR_UUID:
      {
        *pLen = MB_REQUEST_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_REQUEST_CHAR_LEN );
      }
      break;
      
      case MB_RESPONSE_CHAR_UUID:
      {
        *pLen = MB_RESPONSE_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_RESPONSE_CHAR_LEN );
      }
      break;
      
      case MB_INDICATION_CHAR_UUID:
      {
        *pLen = MB_INDICATION_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_INDICATION_CHAR_LEN );
      }
      break;
      
      case MB_DATA1_CHAR_UUID:
      {
        *pLen = MB_DATA1_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA1_CHAR_LEN );
      }
      break;
      
      case MB_DATA2_CHAR_UUID:
      {
        *pLen = MB_DATA2_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA2_CHAR_LEN );      
      }
      break;  
      
      case MB_DATA3_CHAR_UUID:
      {
        *pLen = MB_DATA3_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA3_CHAR_LEN );      
      }
      break;  
      
      case MB_DATA4_CHAR_UUID:
      {
        *pLen = MB_DATA4_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA4_CHAR_LEN );      
      }
      break;  
      
      default:
      {
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
      }
    }
  }
  else if (pAttr->type.len == ATT_UUID_SIZE)
  {
    //!Using 128bit UUID
    if (!memcmp( pAttr->type.uuid, requestCharUUID,  pAttr->type.len))
    {
        *pLen = MB_REQUEST_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_REQUEST_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, responseCharUUID,  pAttr->type.len))
    {
        *pLen = MB_RESPONSE_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_RESPONSE_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, indicationCharUUID,  pAttr->type.len))
    {
        *pLen = MB_INDICATION_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_INDICATION_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, data1CharUUID,  pAttr->type.len))
    {
        *pLen = MB_DATA1_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA1_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, data2CharUUID,  pAttr->type.len))
    {
        *pLen = MB_DATA2_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA2_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, data3CharUUID,  pAttr->type.len))
    {
        *pLen = MB_DATA3_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA3_CHAR_LEN );
    }
    else if (!memcmp( pAttr->type.uuid, data4CharUUID,  pAttr->type.len))
    {
        *pLen = MB_DATA4_CHAR_LEN;
        VOID memcpy( pValue, pAttr->pValue, MB_DATA4_CHAR_LEN );
    }
    else
    {
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    //!Another UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}
/*!
 ******************************************************************************
 * @fn      MB_WriteAttrCB
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
 ******************************************************************************/ 
static bStatus_t MB_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                uint8_t *pValue, uint16_t len, uint16_t offset, 
                                uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    //!Using 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case MB_REQUEST_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_REQUEST_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        //!Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID memcpy( pCurValue, pValue, len );

          if( !memcmp(pAttr->pValue, requestChar, len) )
          {
            notifyApp = MB_REQUEST_CHAR;        
          }
        }
      }
      break;
      
      case MB_RESPONSE_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_RESPONSE_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //!Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_RESPONSE_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, responseChar, len) )
          {
            notifyApp = MB_RESPONSE_CHAR;        
          }
        }          
      }
      break;
      
      case MB_INDICATION_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_INDICATION_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_INDICATION_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, indicationChar, len) )
          {
            notifyApp = MB_INDICATION_CHAR;        
          }
        }      
      }
      break;
      
      case MB_DATA1_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_DATA1_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA1_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, data1Char, len) )
          {
            notifyApp = MB_DATA1_CHAR;        
          }
        }      
      }
      break;
      
      case MB_DATA2_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_DATA2_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA2_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, data2Char, len) )
          {
            notifyApp = MB_DATA2_CHAR;        
          }
        }      
      }
      break;
      
      case MB_DATA3_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_DATA3_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA3_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, data3Char, len) )
          {
            notifyApp = MB_DATA3_CHAR;        
          }
        }      
      }
      break;
      
      case MB_DATA4_CHAR_UUID:
      {
        if ( offset == 0 )
        {
          if ( len > MB_DATA4_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA4_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, data4Char, len) )
          {
            notifyApp = MB_DATA4_CHAR;        
          }
        }      
      }
      break;
      
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    //!Using 128-bit UUID
    if ( !memcmp( pAttr->type.uuid, requestCharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_REQUEST_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //!Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_REQUEST_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, requestChar, len) )
          {
            notifyApp = MB_REQUEST_CHAR;        
          }
        }
    }
    else if ( !memcmp( pAttr->type.uuid, responseCharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_RESPONSE_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_RESPONSE_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, responseChar, len) )
          {
            notifyApp = MB_RESPONSE_CHAR;        
          }
        }      
    }
    else if ( !memcmp( pAttr->type.uuid, indicationCharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_INDICATION_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_INDICATION_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if( !memcmp(pAttr->pValue, indicationChar, len) )
          {
            notifyApp = MB_INDICATION_CHAR;        
          }
        }
    }
    else if ( !memcmp( pAttr->type.uuid, data1CharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_DATA1_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA1_CHAR_LEN );
          VOID memcpy( pCurValue, pValue, len );
          if(!memcmp(pAttr->pValue, data1Char, len))
          {
            notifyApp = MB_DATA1_CHAR;        
          }
        }
    }
    else if ( !memcmp( pAttr->type.uuid, data2CharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_DATA2_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA2_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if(!memcmp(pAttr->pValue, data2Char, len))
          {
            notifyApp = MB_DATA2_CHAR;        
          }
        }
    }
    else if ( !memcmp( pAttr->type.uuid, data3CharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_DATA3_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA3_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if(!memcmp(pAttr->pValue, data3Char, len))
          {
            notifyApp = MB_DATA3_CHAR;        
          }
        }
    }
    else if ( !memcmp( pAttr->type.uuid, data4CharUUID,  pAttr->type.len) )
    {
        if ( offset == 0 )
        {
          if ( len > MB_DATA4_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          VOID memset( pCurValue, 0x00, MB_DATA4_CHAR_LEN);
          VOID memcpy( pCurValue, pValue, len );
          if(!memcmp(pAttr->pValue, data4Char, len))
          {
            notifyApp = MB_DATA4_CHAR;        
          }
        }
    }
    else
    {
    // Another UUID
    status = ATT_ERR_INVALID_HANDLE;
    }
  }
  
  //!If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && motionBand_AppCBs && motionBand_AppCBs->pfnMotionBandChange )
  {
    motionBand_AppCBs->pfnMotionBandChange( notifyApp );  
  }
  //!Write data from characteristic from BLE Central
  g_LengthDataReceiveApp = len;
  return ( status );
}
/********************************************************************************
 * END
 ********************************************************************************/