/*! 
 *  @file			  Sys_Service.h
 *  @author  		
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2017-02-08
 *	@brief			service to update data of system
 */
#ifndef _SYS_SERVICE_H
#define _SYS_SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"

/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define SYS_SERV_UUID         0xFFF0
#define SYS_PERI_UUID         0xFFF1
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Sys_addService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Sys_addService(void);

/*
 * Sys_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Sys_registerAppCBs(sensorCBs_t *appCallbacks);

/*
 * Sys_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Sys_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * Sys_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Sys_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SYS_SERVICE_H */

