/*! 
 *  @file	      MB_Service.h
 *  @author  	  Dinh Le
 *  @copyright	Fiot Co.,Ltd
 *  @version 	  1.0
 *  @date    	  2017-4-12
 *  @brief	    Service of MotionBand
 *
 */
#ifndef __MB_SERVICE_H__
#define __MB_SERVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 * INCLUDES
 ********************************************************************************/
#include "stdint.h"
#include "bcomdef.h"
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
//!MotionBand 's Service UUID
#define MB_SERV_UUID                            (0xFF00)
  
//!MotionBand 's Services bit fields
#define MB_SERVICE                              (0x00000001)
  
//!MotionBand 's characteristic
#define MB_REQUEST_CHAR                         (0)  
#define MB_RESPONSE_CHAR                        (1)  
#define MB_INDICATION_CHAR                      (2)  
#define MB_DATA1_CHAR                           (3)  
#define MB_DATA2_CHAR                           (4)  
#define MB_DATA3_CHAR                           (5)  
#define MB_DATA4_CHAR                           (6) 
  
//!MotionBand 's characteristic UUID
#define MB_REQUEST_CHAR_UUID                    (0xFF01)
#define MB_RESPONSE_CHAR_UUID                   (0xFF02)
#define MB_INDICATION_CHAR_UUID                 (0xFF03)
#define MB_DATA1_CHAR_UUID                      (0xFF04)
#define MB_DATA2_CHAR_UUID                      (0xFF05)
#define MB_DATA3_CHAR_UUID                      (0xFF06)
#define MB_DATA4_CHAR_UUID                      (0xFF07)
  
//!MotionBand 's characteristic length in bytes 
#define MB_REQUEST_CHAR_LEN                     (20)
#define MB_RESPONSE_CHAR_LEN                    (20)
#define MB_INDICATION_CHAR_LEN                  (20)      
#define MB_DATA1_CHAR_LEN                       (20)
#define MB_DATA2_CHAR_LEN                       (20)  
#define MB_DATA3_CHAR_LEN                       (20)
#define MB_DATA4_CHAR_LEN                       (20)   

//!MotionBand 's UUID 128bit: 0xXXXX-7AA9-18E911E7-93AE-92361F002671
#define MB_UUID(uuid_16bit)                     0x71, 0x26, 0x00, 0x1F, 0x36, 0x92, 0xAE, 0x93, 0xE7, 0x11, 0xE9, 0x18, 0xA9, 0x7A, LO_UINT16(uuid_16bit), HI_UINT16(uuid_16bit)
 
/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/
  
/********************************************************************************
 * GLOBAL VARIABLE
 ********************************************************************************/
extern uint8_t g_LengthDataReceiveApp; 
/********************************************************************************
 * MACROS
 ********************************************************************************/
#define USING_UUID_128BIT
/********************************************************************************
 * Profile Callbacks
 ********************************************************************************/
//!Callback when a characteristic value has changed
typedef void (*motionBandChange_t)( uint8 paramID );
typedef struct
{
  motionBandChange_t        pfnMotionBandChange;  //!Called when characteristic value changes
} motionBandCBs_t;   
/********************************************************************************
 * API FUNCTIONS 
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
extern bStatus_t MB_AddService( uint32 services );
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
extern bStatus_t MB_RegisterAppCBs( motionBandCBs_t *appCallbacks );
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
extern bStatus_t MB_SetParameter( uint8 param, uint8 len, void *value );
/*!
 *******************************************************************************
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
extern bStatus_t MB_GetParameter( uint8 param, void *value );
/********************************************************************************
 * END
 ********************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* MB_SERVICE_H */
