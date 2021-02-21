/********************************************************************************
 *  @Filename:     fiot_hal_adc.h
 *  @Author        Fiot Co.,Ltd
 *  @Version       1.1
 *  @Date          April 06, 2017
 *  @Description:  Driver of ADC
 *    -------------------------------------------------
 *    |                 fiot_hal_adc.h                |
 *    -------------------------------------------------
 *
 *  @Example 1: Read data on Channel 1
 *    //!***************************
 *    //!Generate Pointer of ADC channel 1
 *    //!***************************
 *    HAL_ADC_T       mADC_Channel1;
 *    HAL_ADC_Data_T   ADC_Raw_Data = 0;
 *    HAL_ADC_Volt_T   ADC_Volt_Data = 0;
 *    //!***************************
 *    //!Enable ADC of MCU
 *    //!***************************
 *    FIOT_HAL_ADC_Init(ADC_REF_VDD);
 *    //!***************************
 *    //!Enable Channel 1 of ADC
 *    //!***************************
 *    FIOT_HAL_ADC_Open(&mADC_Channel1,ADC_CHANNEL_1);
 *    //!***************************
 *    //!Read Raw Data on Channel 1 of ADC
 *    //!***************************
 *    FIOT_HAL_ADC_Read(&mADC_Channel1,&ADC_Raw_Data);
 *    //!***************************
 *    //!Read Convert Data on Channel 1 of ADC
 *    //!***************************
 *    FIOT_HAL_ADC_ReadConvert(&mADC_Channel1,&ADC_Volt_Data);
 *
 ********************************************************************************/
#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

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

/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef enum
{
  ADC_STATUS_OK,             //!Success
  ADC_STATUS_ERROR_NULL_PTR, //!Error input null pointer as function parameter
  ADC_STATUS_ERROR_OPTION,   //!Error format of functions
  ADC_STATUS_ERROR_TIMEOUT   //!Error ADC timeout
}
HAL_ADC_Status_T;

typedef uint16_t  HAL_ADC_Data_T;
typedef int16_t   HAL_ADC_Volt_T;
//!Channel
typedef enum
{
#ifdef __FIOT_CC2650_
  ADC_CHANNEL_0,
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_5,
  ADC_CHANNEL_6,
  ADC_CHANNEL_7,
  ADC_CHANNEL_COUNT
#endif
}
HAL_ADC_Channel_T;

//!Vref of ADC
typedef enum
{
  ADC_REF_VDD
}
HAL_ADC_Ref_T;

typedef struct
{
  HAL_ADC_Channel_T  ADC_Channel;
}
HAL_ADC_T;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * FUNCTIONS - API
 ********************************************************************************/
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_Init(HAL_ADC_Ref_T  iADC_Ref)
 *
 * @brief   + Enable ADC of MCU
 *          + Set Vref of ADC
 *
 * @param   iADC_Ref        - Vref of ADC
 *
 * @return  ADC_STATUS_OK
 * @return  ADC_STATUS_ERROR_OPTION
 * @return  ADC_STATUS_ERROR_TIMEOUT
 ********************************************************************************/
extern HAL_ADC_Status_T  \
FIOT_HAL_ADC_Init(HAL_ADC_Ref_T  iADC_Ref);
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_Open(HAL_ADC_T    			*iADC,
 *                                              HAL_ADC_Channel_T  iADC_Channel)
 *
 * @brief   Init channel of ADC
 *
 * @param   iADC            - Pointer to manage information of ADC
 * @param   iADC_Channel    - Channnel of ADC
 *
 * @return  ADC_STATUS_OK
 * @return  ADC_STATUS_ERROR_OPTION
 * @return  ADC_STATUS_ERROR_TIMEOUT
 ********************************************************************************/
extern HAL_ADC_Status_T  \
FIOT_HAL_ADC_Open(HAL_ADC_T  *iADC,
                  HAL_ADC_Channel_T  iADC_Channel);
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_Read(HAL_ADC_T    			*iADC,
 *                                              HAL_ADC_Data_T    *oADC_Data)
 *
 * @brief   Read raw data of ADC on Channel
 *
 * @param   iADC        - Pointer to manage information of ADC
 * @param   oADC_Data   - the Raw output data of ADC
 *
 * @return  ADC_STATUS_OK
 * @return  ADC_STATUS_ERROR_OPTION
 * @return  ADC_STATUS_ERROR_TIMEOUT
 ********************************************************************************/
extern HAL_ADC_Status_T  \
FIOT_HAL_ADC_Read(HAL_ADC_T  *iADC,
                  HAL_ADC_Data_T  *oADC_Data);
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_ReadConvert(HAL_ADC_T    			*iADC,
 *                                                     HAL_ADC_Volt_T    *oADC_Data)
 *
 * @brief   Read the converted data to Voltage based on Vref.
 *          Unit is mV
 *
 * @param   iADC        - Pointer to manage information of ADC
 * @param   oADC_Data   - the output data of ADC
 *
 * @return  ADC_STATUS_OK
 * @return  ADC_STATUS_ERROR_OPTION
 * @return  ADC_STATUS_ERROR_TIMEOUT
 ********************************************************************************/
extern HAL_ADC_Status_T  \
FIOT_HAL_ADC_ReadConvert(HAL_ADC_T  *iADC,
                         HAL_ADC_Volt_T   *oADC_Data);
/********************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __HAL_ADC_H__ */
