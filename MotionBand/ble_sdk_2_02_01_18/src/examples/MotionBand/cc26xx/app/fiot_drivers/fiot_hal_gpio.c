/********************************************************************************
 *	@Filename:	hal_gpio.c
 *  @Author  		Fiot Co.,Ltd
 *  @Version 		1.0
 *  @Date    		2016-03-08
 *	@Description:	Driver of GPIO on TI CC26xx
 *		-------------------------------------------------
 *		|					        hal_gpio.c				            |
 *		-------------------------------------------------
 ********************************************************************************/
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
#include "fiot_hal_gpio.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/PIN/PINCC26XX.h>
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef uint16_t  GPIO_PinID_T;
typedef uint8_t   GPIO_Handle_Status_T;
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
#define HAL_GPIO_MAX_PIN						(32)
#define	HAL_GPIO_MODE_PULLUP_OFF
#define HAL_GPIO_MODE_PULLUP_ON
#define HAL_GPIO_HANDLE_OPEN        (1)
#define HAL_GPIO_HANDLE_CLOSE       (0)
/********************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************/
static PIN_State GPIO_Pins_State;
static PIN_Handle GPIO_Pins_Handle;
static GPIO_Handle_Status_T  GPIO_Current_Handle_Status = HAL_GPIO_HANDLE_CLOSE;
static const PIN_Config GPIO_Init_Table[] = {
    PIN_TERMINATE /* Terminate list */
};

static HAL_GPIO_Pin_Callback_T  HAL_GPIO_EXT_IT_Table[HAL_GPIO_MAX_PIN];
/********************************************************************************
 * MACROS
 ********************************************************************************/

/********************************************************************************
 * LOCAL FUNCTIONS
 ********************************************************************************/
static HAL_GPIO_Status_T  FIOT_HAL_GPIO_Convert_Settings(HAL_GPIO_T	*iGPIO,
																												 PIN_Config  *iGPIO_Configs);
static HAL_GPIO_Status_T	FIOT_HAL_GPIO_Init_IT(HAL_GPIO_T	*iGPIO);
/********************************************************************************
 * FUNCTIONS - CALLBACK
 ********************************************************************************/
static void  HAL_GPIO_IT_CB(PIN_Handle iHandle, PIN_Id iPinID);

static void  HAL_GPIO_IT_CB(PIN_Handle iHandle, PIN_Id iPinID)
{
	if((iHandle != NULL) && (HAL_GPIO_EXT_IT_Table[PIN_ID(iPinID)] != NULL))
	{
		//!Jump into user's cb function
		HAL_GPIO_EXT_IT_Table[PIN_ID(iPinID)]();
	}
}
/********************************************************************************
 * FUNCTIONS - LOCAL
 ********************************************************************************/
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Init_IT
 *
 * @brief   Config Interrupt of GPIO
 *
 * @param   self  - pointer to GPIO
 *
 * @return  GPIO_STATUS_OK    - Success
 *					GPIO_STATUS_ERROR_OPTION	-	Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T FIOT_HAL_GPIO_Init_IT(HAL_GPIO_T	*iGPIO)
{
	//!Checking NULL Pointer
  if(iGPIO == NULL)
		return GPIO_STATUS_ERROR_NULL_PTR;

	HAL_GPIO_EXT_IT_Table[PIN_ID(iGPIO->Pin_Info.Pin_ID)] = iGPIO->pPin_Cb;
	//!Convert from FIOT settings to TI interrupt settings
	PIN_Config  interruptConfig = iGPIO->Pin_Info.Pin_ID;
	switch(iGPIO->Pin_Mode.Pin_Mode_IT)
	{
		case GPIO_MODE_IT_OFF:
		{
			interruptConfig |= PIN_IRQ_DIS;
			break;
		}
		case GPIO_MODE_IT_FALL_EDGE:
		{
			interruptConfig |= PIN_IRQ_NEGEDGE;
			break;
		}
		case GPIO_MODE_IT_RISE_EDGE:
		{
			interruptConfig |= PIN_IRQ_POSEDGE;
			break;
		}
		case GPIO_MODE_IT_RISE_OR_FALL_EDGE:
		{
			interruptConfig |= PIN_IRQ_BOTHEDGES;
			break;
		}

		default:
		{
			return GPIO_STATUS_ERROR_OPTION;
		}
	}
	//!Set interrupt config for the input pin
	if(PIN_setInterrupt(GPIO_Pins_Handle, interruptConfig) != PIN_SUCCESS)
		return GPIO_STATUS_ERROR_OPTION;
	if(PIN_registerIntCb(GPIO_Pins_Handle, &HAL_GPIO_IT_CB) != PIN_SUCCESS)
		return GPIO_STATUS_ERROR_OPTION;

	return	GPIO_STATUS_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Convert_Settings
 *
 * @brief   Convert Parameters of GPIO to TI library parameters
 *
 * @param   iGPIO         - Pointer of self
 *          iGPIO_Configs - Setting of Pin
 *
 * @return  GPIO_STATUS_OK    - Success
 *					GPIO_STATUS_ERROR_OPTION	-	Option is Error
 ********************************************************************************/
static HAL_GPIO_Status_T  FIOT_HAL_GPIO_Convert_Settings(HAL_GPIO_T	 *iGPIO,
																												 PIN_Config  *iGPIO_Configs)
{
	if(iGPIO == NULL || iGPIO_Configs == NULL)
		return GPIO_STATUS_ERROR_NULL_PTR;

	switch(iGPIO->Pin_Mode.Pin_Type)
	{
		case GPIO_TYPE_INPUT:
		{
			*iGPIO_Configs |= (PIN_INPUT_EN | PIN_HYSTERESIS);
			switch(iGPIO->Pin_Mode.Pin_Mode_PullUp)
			{
				case GPIO_MODE_PULLUP_OFF:
				{
					*iGPIO_Configs |= PIN_NOPULL;
					break;
				}
				case GPIO_MODE_PULLUP_ON:
				{
					*iGPIO_Configs |= PIN_PULLUP;
					break;
				}
			}
			break;
		}
		case GPIO_TYPE_OUTPUT_HIGH:
		{
			*iGPIO_Configs |= (PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX);
			break;
		}
		case GPIO_TYPE_OUTPUT_LOW:
		{
			*iGPIO_Configs |= (PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX);
			break;
		}

		default:
		{
			return GPIO_STATUS_ERROR_OPTION;
		}
	}

  return	GPIO_STATUS_OK;
}
/********************************************************************************
 * FUNCTIONS - API
 ********************************************************************************/
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Init
 *
 * @brief   Configure Parameters of GPIO
 *
 * @param   iGPIO 				-	pointer to GPIO
 *				  iPin_Info 		-	Type of GPIO Information
 *				  iPin_Mode			-	Mode of GPIO Pin
 *				  iPin_Callback	-	Callback function(TYPE_INPUT only)
 *
 * @return  GPIO_STATUS_OK    - Success
 *					GPIO_STATUS_ERROR_OPTION	-	Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T	 FIOT_HAL_GPIO_Init(HAL_GPIO_T  *iGPIO,
										                  HAL_GPIO_Pin_Info_T	 iPin_Info,
										                  HAL_GPIO_Pin_Mode_T	 iPin_Mode,
										                  HAL_GPIO_Pin_Callback_T  iPin_Callback)
{
	if(iGPIO == NULL)
    return GPIO_STATUS_ERROR_NULL_PTR;
	//!Variable that indicate current config configStatus
	HAL_GPIO_Status_T  configStatus = GPIO_STATUS_OK;

	//!Copy input data to GPIO instance
	{
		iGPIO->Pin_Info.Pin_ID = iPin_Info.Pin_ID;
		iGPIO->Pin_Mode.Pin_Type = iPin_Mode.Pin_Type;
		iGPIO->Pin_Mode.Pin_Mode_IT = iPin_Mode.Pin_Mode_IT;
		iGPIO->Pin_Mode.Pin_Mode_PullUp = iPin_Mode.Pin_Mode_PullUp;
		iGPIO->pPin_Cb = iPin_Callback;
	}
	//!Convert FIOT GPIO settings to TI Library settings
	PIN_Config  TI_PIN_Configs = iPin_Info.Pin_ID;
	{
		configStatus = FIOT_HAL_GPIO_Convert_Settings(iGPIO, &TI_PIN_Configs);
		if(configStatus != GPIO_STATUS_OK)
			return configStatus;
	}
  if(GPIO_Current_Handle_Status == HAL_GPIO_HANDLE_CLOSE)
  {
    //!Initialize Pin handle
    GPIO_Pins_Handle = PIN_open(&GPIO_Pins_State, GPIO_Init_Table);
    if(GPIO_Pins_Handle == NULL)
      return GPIO_STATUS_ERROR_NULL_PTR;
    else
      GPIO_Current_Handle_Status = HAL_GPIO_HANDLE_OPEN;
  }
	//!Adding pin to handle
	if(PIN_add(GPIO_Pins_Handle, TI_PIN_Configs) != PIN_SUCCESS)
		return GPIO_STATUS_ERROR_OPTION;
	//!Config interrupt settings
	if(iGPIO->Pin_Mode.Pin_Type == GPIO_TYPE_INPUT)
	{
		configStatus = FIOT_HAL_GPIO_Init_IT(iGPIO);
		if(configStatus != GPIO_STATUS_OK)
			return configStatus;
	}

	return	GPIO_STATUS_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Set
 *
 * @brief	  Setting the value of the Pin in self pointer
 *
 * @param   iGPIO 		-	pointer to GPIO
 *			    iPin_Value 	-	Value of GPIO Pin(GPIO_VALUE_HIGH or GPIO_VALUE_LOW)
 *
 * @return  GPIO_STATUS_OK    - Success
 *			    GPIO_STATUS_ERROR_OPTION	-	Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T	 FIOT_HAL_GPIO_Set(HAL_GPIO_T  *iGPIO,
																		 HAL_GPIO_Value_T  iPin_Value)
{
	if(iGPIO == NULL)
    return GPIO_STATUS_ERROR_NULL_PTR;

	switch(iPin_Value)
	{
		case GPIO_VALUE_HIGH:
			PIN_setOutputValue(GPIO_Pins_Handle, iGPIO->Pin_Info.Pin_ID, GPIO_VALUE_HIGH);
			break;
		case GPIO_VALUE_LOW:
			PIN_setOutputValue(GPIO_Pins_Handle, iGPIO->Pin_Info.Pin_ID, GPIO_VALUE_LOW);
			break;

		default:
			return GPIO_STATUS_ERROR_OPTION;
	}

	return GPIO_STATUS_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_GPIO_Get
 *
 * @brief   Get the value from the Pin
 *
 * @param   iGPIO	 -  pointer to GPIO
 *			    oPin_Value  -  Value of GPIO Pin(GPIO_VALUE_HIGH or GPIO_VALUE_LOW)
 *
 * @return  GPIO_STATUS_OK  -  Success
 *			    GPIO_STATUS_ERROR_OPTION  -	 Option is Error
 ********************************************************************************/
HAL_GPIO_Status_T  FIOT_HAL_GPIO_Get(HAL_GPIO_T  *iGPIO,
                                     HAL_GPIO_Value_T  *oPin_Value)
{
  if(iGPIO == NULL)
    return GPIO_STATUS_ERROR_NULL_PTR;

  if(PIN_getOutputValue(iGPIO->Pin_Info.Pin_ID))
    *oPin_Value = GPIO_VALUE_HIGH;
  else
    *oPin_Value = GPIO_VALUE_LOW;

  return GPIO_STATUS_OK;
}
/********************************************************************************
 * End
 ********************************************************************************/