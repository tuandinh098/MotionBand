/********************************************************************************
 *	@Filename:	fiot_hal_define.h
 *  @Author  		Fiot Co.,Ltd
 *  @Version 		1.0
 *  @Date    		2016-09-31
 *	@Description:
 ********************************************************************************/

#ifndef __FIOT_HAL_DEFINE_H__
#define __FIOT_HAL_DEFINE_H__

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 * CONFIG FIOT HAL
 ********************************************************************************/
//!Configurate to choose MCU
// #define __FIOT_SAM3X8E_
// #define __FIOT_STM32F407_
// #define	__FIOT_STM32F411_
// #define __FIOT_ATMEGA328P_
#define __FIOT_CC2650_

//!General purpose C library
#include "stdint.h"
#include "string.h"

/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef volatile uint8_t * const  HAL_Register8_T;
typedef volatile uint16_t * const  HAL_Register16_T;
typedef volatile uint32_t * const  HAL_Register32_T;

//!Include ASF libs
#ifdef __FIOT_SAM3X8E_
	#include <asf.h>
  #include "delay.h"
  #define __DELAY_MS(_TIME_)  delay_ms(_TIME_)
#endif

//!Include STM32F407 HAL libs
#ifdef  __FIOT_STM32F407_
  #include "stm32f4xx_hal.h"
  #define __DELAY_MS(_TIME_)  HAL_Delay(_TIME_)
#endif

#ifdef __FIOT_STM32F411_
	#include "stm32f4xx_hal.h"
  #define __DELAY_MS(_TIME_)  HAL_Delay(_TIME_)
#endif

#ifdef __FIOT_ATMEGA328P_
	#include <asf.h>
  #include <util/delay.h>
  #define __DELAY_MS(_TIME_)  _delay_ms(_TIME_)
#endif

#ifdef __FIOT_ATMEGA328PB_
	#include <asf.h>
  #include <util/delay.h>
  #define __DELAY_MS(_TIME_)  _delay_ms(_TIME_)
#endif

#ifdef __FIOT_CC2650_

#endif

#ifdef __cplusplus
}
#endif

#endif /* __FIOT_HAL_DEFINE_H__ */
