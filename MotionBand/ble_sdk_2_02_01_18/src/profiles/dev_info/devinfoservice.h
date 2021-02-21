/*!
 *	@file				devinfoservice.h
 *  @author  		Dinh Le
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2016-04-12
 *	@brief			This file contains the Device Information service of MotionBand
 *
 */
#ifndef DEVINFOSERVICE_H
#define DEVINFOSERVICE_H
#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/

/********************************************************************************
 * MACROS
 ********************************************************************************/
  
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
//!Device Information Service Parameters
#define DEVINFO_SYSTEM_ID                 (0)
#define DEVINFO_FIRMWARE_REV              (1)
#define DEVINFO_HARDWARE_REV              (2)
//!System ID length
#define DEVINFO_SYSTEM_ID_LEN             (8)
//!String attribute length
#ifndef DEVINFO_STR_ATTR_LEN
  #define DEVINFO_STR_ATTR_LEN            (20)
#endif
/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/

/********************************************************************************
 * Profile Callbacks
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
extern bStatus_t DevInfo_AddService( void );
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
bStatus_t DevInfo_SetParameter( uint8 param, uint8 len, void *value );
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
extern bStatus_t DevInfo_GetParameter( uint8 param, void *value );
/********************************************************************************
 * END
 ********************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* DEVINFOSERVICE_H */
