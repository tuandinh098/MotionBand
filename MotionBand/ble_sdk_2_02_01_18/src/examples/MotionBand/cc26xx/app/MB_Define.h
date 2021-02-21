/*!
 *		@file			  MB_Define.h
 *  	@author  		Vy Luu
 *		@copyright	Fiot Co.,Ltd
 *  	@version 		1.0
 *  	@date    		2017-02-09
 *		@brief			Define common information of system
 */
#ifndef __MB_DEFINE_H__
#define __MB_DEFINE_H__

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
//!Common
#include <stdint.h>
#include <string.h>

//!System
#include "MB_Board.h"
#include "MB_Main.h"
//!Fiot's drivers
#include "fiot_hal_gpio.h"
#include "fiot_hal_serial.h"
/********************************************************************************
 * EXTERNS
 ********************************************************************************/

/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
#define FW_VERSION	          1.0
#define HW_VERSION            1.0
/********************************************************************************
 * MACROS
 ********************************************************************************/
// Utility string macros
#define TOSTRING(x)     #x
#define FW_VERSION_STR  TOSTRING(FW_VERSION)" ("__DATE__")"
/********************************************************************************
 * END
 ********************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __MB_DEFINE_H__ */
