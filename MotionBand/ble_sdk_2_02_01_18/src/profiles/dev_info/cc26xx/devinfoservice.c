/*!
 *	@file				devinfoservice.h
 *  @author  		Dinh Le
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2016-04-12
 *	@brief			This file contains the Device Information service of MotionBand
 *
 */
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
#include <string.h>
#include "bcomdef.h"
#include "osal.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "devinfoservice.h"
/********************************************************************************
 * MACROS
 ********************************************************************************/

/********************************************************************************
 * CONSTANTS
 ********************************************************************************/

/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/
//!Device information service UUID 16 bit
CONST uint8 devInfoServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEVINFO_SERV_UUID), HI_UINT16(DEVINFO_SERV_UUID)
};
//!System ID
CONST uint8 devInfoSystemIdUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SYSTEM_ID_UUID), HI_UINT16(SYSTEM_ID_UUID)
};
//!Firmware Revision
CONST uint8 devInfoFirmwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(FIRMWARE_REV_UUID), HI_UINT16(FIRMWARE_REV_UUID)
};
//!Hardware Revision
CONST uint8 devInfoHardwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HARDWARE_REV_UUID), HI_UINT16(HARDWARE_REV_UUID)
};
/********************************************************************************
 * EXTERNAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * EXTERNAL FUNCTIONS
 ********************************************************************************/
extern void* memcpy(void *dest, const void *src, size_t len);
/********************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * Profile Attributes - variables
 ********************************************************************************/
//!Device Information Service attribute
static CONST gattAttrType_t devInfoService = { ATT_BT_UUID_SIZE, devInfoServUUID };
//!System ID characteristic
static uint8 devInfoSystemIdProps = GATT_PROP_READ;
static uint8 devInfoSystemId[DEVINFO_SYSTEM_ID_LEN] = {0x00};
static uint8 devInfoSystemIdUserDesp[10] = "System ID";
//!Firmware Revision String characteristic
static uint8 devInfoFirmwareRevProps = GATT_PROP_READ;
static uint8 devInfoFirmwareRev[DEVINFO_STR_ATTR_LEN+1] = "Firmware v1.0";
static uint8 devInfoFirmwareRevUserDesp[18] = "Firmware Revision";
//!Hardware Revision String characteristic
static uint8 devInfoHardwareRevProps = GATT_PROP_READ;
static uint8 devInfoHardwareRev[DEVINFO_STR_ATTR_LEN+1] = "Hardware v1.0";
static uint8 devInfoHardwareRevUserDesp[18] = "Hardware Revision";
/********************************************************************************
 * Profile Attributes - Table
 ********************************************************************************/
static gattAttribute_t devInfoAttrTbl[] =
{
  //!Device Information Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&devInfoService                  /* pValue */
  },
    //!System ID Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSystemIdProps
    },
      // System ID Value
      {
        { ATT_BT_UUID_SIZE, devInfoSystemIdUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSystemId
      },
      //!System ID User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        devInfoSystemIdUserDesp 
      },  
    //!Firmware Revision Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoFirmwareRevProps
    },
      //!Firmware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoFirmwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoFirmwareRev
      },
      //!Firmware Revision User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        devInfoFirmwareRevUserDesp 
      },           
    // Hardware Revision Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoHardwareRevProps
    },
      // Hardware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoHardwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoHardwareRev
      },
      //!Hardware Revision User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        devInfoHardwareRevUserDesp 
      },   
};


/********************************************************************************
 * LOCAL FUNCTIONS
 ********************************************************************************/
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method );

/********************************************************************************
 * PROFILE CALLBACKS
 ********************************************************************************/
CONST gattServiceCBs_t devInfoCBs =
{
  devInfo_ReadAttrCB, // Read callback function pointer
  NULL,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};
/********************************************************************************
 * NETWORK LAYER CALLBACKS
 ********************************************************************************/

/********************************************************************************
 * API FUNCTIONS
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      DevInfo_AddService
 *
 * @brief   Initializes the Device Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
********************************************************************************/
bStatus_t DevInfo_AddService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( devInfoAttrTbl,
                                      GATT_NUM_ATTRS( devInfoAttrTbl ),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &devInfoCBs );
}
/*!
 ********************************************************************************
 * @fn      DevInfo_SetParameter
 *
 * @brief   Set a Device Information parameter.
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
bStatus_t DevInfo_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
     case DEVINFO_SYSTEM_ID:
      // verify length
      if (len == sizeof(devInfoSystemId))
      {
        memcpy(devInfoSystemId, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_FIRMWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoFirmwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoFirmwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_HARDWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoHardwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoHardwareRev, value, len);
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
 * @fn      DevInfo_GetParameter
 *
 * @brief   Get a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
********************************************************************************/
bStatus_t DevInfo_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_SYSTEM_ID:
      memcpy(value, devInfoSystemId, sizeof(devInfoSystemId));
      break;

    case DEVINFO_FIRMWARE_REV:
      memcpy(value, devInfoFirmwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_HARDWARE_REV:
      memcpy(value, devInfoHardwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}
/*!
 ********************************************************************************
 * @fn          devInfo_ReadAttrCB
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
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // If the value offset of the Read Blob Request is greater than the
  // length of the attribute value, an Error Response shall be sent with
  // the error code Invalid Offset.
  switch (uuid)
  {
    case SYSTEM_ID_UUID:
      // verify offset
      if (offset > sizeof(devInfoSystemId))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoSystemId) - offset));

        // copy data
        memcpy(pValue, &devInfoSystemId[offset], *pLen);
      }
      break;

    case FIRMWARE_REV_UUID:
    case HARDWARE_REV_UUID:
      {
        uint16 len = strlen((char *)(pAttr->pValue));

        // verify offset
        if (offset > len)
        {
          status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
          // determine read length (exclude null terminating character)
          *pLen = MIN(maxLen, (len - offset));

          // copy data
          memcpy(pValue, &(pAttr->pValue[offset]), *pLen);
        }
      }
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  return ( status );
}
/********************************************************************************
 * END
 ********************************************************************************/