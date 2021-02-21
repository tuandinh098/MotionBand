/********************************************************************************
 *  @Filename:  hal_gpio.h
 *  @Author:    Fiot Co.,Ltd
 *  @Version:   1.5
 *  @Date:      2016-11-22
 *  @Description:  Driver of GPIO 
 *    -------------------------------------------------
 *    |                   hal_gpio.h                  |
 *    -------------------------------------------------
 *
 *  @Example 1: Set PA07 of Port A is Output with Pullup
 *    //!***************************  
 *     //!Generate Pointer of Pin  
 *    //!*************************** 
 *     HAL_GPIO_T           mGPIO;
 *    HAL_GPIO_Status_T    mStatus;
 *    //!***************************
 *    //!Configure Pin 
 *    //!***************************    
 *     mGPIO.Pin_Info.Port_ID = PIOA;   //Port A
 *     mGPIO.Pin_Info.Pin_ID  = PIO_PA7;  //Pin A7
 *     
 *     mGPIO.Pin_Mode.Pin_Type    = GPIO_TYPE_OUTPUT_HIGH; 
 *     mGPIO.Pin_Mode.Pin_Mode_TI   = GPIO_MODE_IT_OFF; 
 *     mGPIO.Pin_Mode.Pin_Mode_PullUp  = GPIO_MODE_PULLUP_ON; 
 *     
 *     mStatus = HAL_GPIO_Init(&mGPIO,
 *                              mGPIO.Pin_Info,
 *                              mGPIO.Pin_Mode,
 *                              NULL);
 *    //!***************************
 *    //!Set value of Output
 *    //!*************************** 
 *    mStatus = HAL_GPIO_Set(&mGPIO,GPIO_VALUE_LOW);
 *
 *  @Example 2: Set PA07 of Port A is Input Interrupt with Pullup
 *    //!***************************  
 *     //!Callback functions  
 *    //!*************************** 
 *    void GPIO_PA07_Callback(void)
 *    {
 *         //!Process
 *     }
 *    //!***************************  
 *     //!Generate Pointer of Pin  
 *    //!*************************** 
 *    HAL_GPIO_T		mGPIO;
 *    HAL_GPIO_Status_T    mStatus;
 *    //!***************************
 *    //!Configure Pin 
 *    //!***************************    
 *     mGPIO.Pin_Info.Port_ID  = PIOA;   //Port A
 *     mGPIO.Pin_Info.Pin_ID    = PIO_PA7;  //Pin A7
 *     
 *     mGPIO.Pin_Mode.Pin_Type          = GPIO_TYPE_INPUT; 
 *     mGPIO.Pin_Mode.Pin_Mode_TI      = GPIO_MODE_IT_FALL_EDGE; 
 *     mGPIO.Pin_Mode.Pin_Mode_PullUp  = GPIO_MODE_PULLUP_ON; 
 *     
 *     mStatus = FIOT_HAL_GPIO_Init(&mGPIO,
 *                               		mGPIO.Pin_Info,
 *                               		mGPIO.Pin_Mode,
 *                              		&GPIO_PA07_Callback);
 *
 * @Note
 * STM : 
 *  Ports on STM32F407: GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI
 *  Pins on STM32F407: GPIO_PIN_0 --> GPIO_PIN_15 and GPIO_PIN_All
 *  IMPORTANT: You can not config two pins to be External Interrupt 
 *       on one line simultaneously.
 *       For example: PA0 and PB0 can't be External Interrupt at the same time. 
 * 
 * SAM3X8E :
 *  Ports on SAM3X8E: PIOA, PIOB, PIOC, PIOD
 *  Pins on SAM3X8E: 
 *    +PIO_PA0 --> PIO_PA29 
 *    +PIO_PB0 --> PIO_PB31 
 *    +PIO_PC0 --> PIO_PC30 
 *    +PIO_PD0 --> PIO_PD10 
 ********************************************************************************/

#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 *  INCLUDES
 ********************************************************************************/
#include "fiot_hal_define.h"
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
#ifdef __FIOT_STM32F407_
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_0  2
//!Priority of PIN_1
#define IRQn_Priority_GPIO_PIN_1  2
//!Priority of PIN_2   
#define IRQn_Priority_GPIO_PIN_2  2
//!Priority of PIN_3   
#define IRQn_Priority_GPIO_PIN_3  2
//!Priority of PIN_4   
#define IRQn_Priority_GPIO_PIN_4  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_5  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_6  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_7  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_8  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_9  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_10  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_11  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_12  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_13  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_14  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_15  2
#endif

#ifdef	__FIOT_STM32F411_
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_0  2
//!Priority of PIN_1
#define IRQn_Priority_GPIO_PIN_1  2
//!Priority of PIN_2   
#define IRQn_Priority_GPIO_PIN_2  2
//!Priority of PIN_3   
#define IRQn_Priority_GPIO_PIN_3  2
//!Priority of PIN_4   
#define IRQn_Priority_GPIO_PIN_4  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_5  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_6  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_7  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_8  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_9  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_10  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_11  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_12  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_13  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_14  2
//!Priority of PIN_0   
#define IRQn_Priority_GPIO_PIN_15  2
#endif

#ifdef __FIOT_SAM3X8E_
#define  IRQn_Priority_GPIO_PIOA    2
#define  IRQn_Priority_GPIO_PIOB    2
#define  IRQn_Priority_GPIO_PIOC    2
#define  IRQn_Priority_GPIO_PIOD    2
#endif

#ifdef __FIOT_ATMEGA328P_
#define  IRQn_Priority_GPIO_PIOB    2
#define  IRQn_Priority_GPIO_PIOC    2
#define  IRQn_Priority_GPIO_PIOD    2
#endif
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef enum
{
  GPIO_STATUS_OK,           //!Success
  GPIO_STATUS_ERROR_OPTION,  //!Error format of functions  
  GPIO_STATUS_ERROR_NULL_PTR
}
HAL_GPIO_Status_T;

typedef enum
{
  GPIO_VALUE_LOW,    //!Value at pin is Low
  GPIO_VALUE_HIGH    //!Value at pin is High
}
HAL_GPIO_Value_T;
 
//!Type of GPIO information
#ifdef __FIOT_STM32F407_
//!Ports on STM32F4: GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI
//!Pins on STM32F4: GPIO_PIN_0 --> GPIO_PIN_15 and GPIO_PIN_All
#endif

#ifdef __FIOT_STM32F411_
//!Ports on STM32F4: GPIOA, GPIOB, GPIOC, GPIOD, GPIOH
//!Pins on STM32F4: GPIO_PIN_0 --> GPIO_PIN_15 and GPIO_PIN_All
//! GPIOA:  GPIO_PIN_0 --> GPIO_PIN_15
//! GPIOB:  GPIO_PIN_0 --> GPIO_PIN_15 (Except GPIO_PIN_11)
//! GPIOC:  GPIO_PIN_0 --> GPIO_PIN_15
//! GPIOD:  GPIO_PIN_2 (1 pin only)
//! GPIOH:  GPIO_PIN_0 --> GPIO_PIN_1 (2 pins only)
#endif

#ifdef __FIOT_SAM3X8E_
//!Ports on SAM3X8E: PIOA, PIOB, PIOC, PIOD
//!Pins on SAM3X8E: 
//! +PIO_PA0 --> PIO_PA29 
//! +PIO_PB0 --> PIO_PB31 
//! +PIO_PC0 --> PIO_PC30 
//! +PIO_PD0 --> PIO_PD10 
#endif

#ifdef __FIOT_ATMEGA328P_
//!Ports on ATMEGA328P: PORTB, PORTC, PORTD
//!Pins on ATMEGA328P:
//! +IOPORT_PORTB: PINB0 --> PINB7
//! +IOPORT_PORTC: PINC0 --> PINC5 (PC6: RESET)
//! +IOPORT_PORTD: PIND0 --> PIND7
#endif

#ifdef __FIOT_ATMEGA328PB_
//!Ports on ATMEGA328P: PORTB, PORTC, PORTD, PORTE
//!Pins on ATMEGA328P:
//! +IOPORT_PORTB: PINB0 --> PINB7
//! +IOPORT_PORTC: PINC0 --> PINC5 (PC6: RESET)
//! +IOPORT_PORTD: PIND0 --> PIND7
//! +IOPORT_PORTE: PINE0 --> PINE3
#endif

#ifdef __FIOT_CC2650_
//!IOID0 --> IOID31
#endif


typedef uint32_t  HAL_GPIO_Port_T;
typedef uint32_t  HAL_GPIO_Pin_T; 
 
typedef struct
{
  #ifndef __FIOT_CC2650_  
    HAL_GPIO_Port_T  Port_ID;  //!Port ID
  #endif
  HAL_GPIO_Pin_T  Pin_ID;    //!Pin ID on Port  
}
HAL_GPIO_Pin_Info_T; 
 
//!Options of GPIO Type
typedef enum
{
  GPIO_TYPE_INPUT,        //!Pin is Input and use EXT Interrupt
  GPIO_TYPE_OUTPUT_HIGH,  //!Pin is Output and default value is HIGH
  GPIO_TYPE_OUTPUT_LOW,   //!Pin is Output and default value is LOW
}
HAL_GPIO_Type_T; 
 
typedef enum
{
  GPIO_MODE_PULLUP_OFF,    //!The internal pin pull-up is disabled.
  GPIO_MODE_PULLUP_ON      //!The internal pin pull-up is enabled.
}
HAL_GPIO_Mode_PullUp_T; 
 
//!Options of Interrupt in Mode of GPIO 
typedef enum
{
  GPIO_MODE_IT_OFF,               //!Interrupt is disabled
  GPIO_MODE_IT_FALL_EDGE,         //!Falling edge interrupt is enabled
  GPIO_MODE_IT_RISE_EDGE,         //!Rising edge interrupt is enabled
  GPIO_MODE_IT_RISE_OR_FALL_EDGE  //!Rising or falling edge interrupt is enabled
}
HAL_GPIO_Mode_IT_T;

typedef struct
{
  HAL_GPIO_Type_T  Pin_Type;
  HAL_GPIO_Mode_IT_T  Pin_Mode_IT;
  HAL_GPIO_Mode_PullUp_T	Pin_Mode_PullUp;
}
HAL_GPIO_Pin_Mode_T; 
 
typedef void  (*HAL_GPIO_Callback_T)(void);
typedef HAL_GPIO_Callback_T  HAL_GPIO_Pin_Callback_T; 
 
typedef struct
{  
  HAL_GPIO_Pin_Info_T  Pin_Info;  
  HAL_GPIO_Pin_Mode_T  Pin_Mode;
  HAL_GPIO_Pin_Callback_T  pPin_Cb;
}
HAL_GPIO_T; 
/********************************************************************************
 * FUNCTIONS - API
 ********************************************************************************/
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Init
 *
 * @brief   Config Parameter of GPIO 
 *
 * @param   iGPIO     -  pointer to GPIO
 *          iPin_Info -  Type of GPIO Information
 *          iPin_Mode -  Mode of GPIO Pin
 *          iCallback -  Callback function(TYPE_INPUT only)
 *
 * @return  GPIO_STATUS_OK        - Success
 *      GPIO_STATUS_ERROR_OPTION  - Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T   FIOT_HAL_GPIO_Init(HAL_GPIO_T  *iGPIO,
                                       HAL_GPIO_Pin_Info_T  iPin_Info,
                                       HAL_GPIO_Pin_Mode_T  iPin_Mode,
                                       HAL_GPIO_Pin_Callback_T  iPin_Callback); 
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Set
 *
 * @brief    Setting the value of the Pin in self pointer
 *
 * @param   iGPIO   -  pointer to GPIO
 *          iPin_Value  -  Value of GPIO Pin(GPIO_HIGH or GPIO_LOW)
 *
 * @return  GPIO_STATUS_OK    - Success
 *          GPIO_STATUS_ERROR_OPTION  - Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T   FIOT_HAL_GPIO_Set(HAL_GPIO_T  *iGPIO,
                                      HAL_GPIO_Value_T  iPin_Value);
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Get
 *
 * @brief   Get the value from the Pin
 *
 * @param   iGPIO   -  pointer to GPIO
 *          oPin_Value  -  Value of GPIO Pin(GPIO_HIGH or GPIO_LOW)
 *
 * @return  GPIO_STATUS_OK  -  Success
 *          GPIO_STATUS_ERROR_OPTION  -   Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T  FIOT_HAL_GPIO_Get(HAL_GPIO_T  *iGPIO, 
                                     HAL_GPIO_Value_T  *oPin_Value);
/********************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __HAL_GPIO_H__ */