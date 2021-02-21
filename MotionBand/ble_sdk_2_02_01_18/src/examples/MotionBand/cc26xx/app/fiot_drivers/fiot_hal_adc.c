/********************************************************************************
 *  @Filename:     fiot_hal_adc.c
 *  @Author        Fiot Co.,Ltd
 *  @Version       1.1
 *  @Date          2017-04-06
 *  @Description:  Driver of ADC
 *    -------------------------------------------------
 *    |            hal_adc.c                          |
 *    -------------------------------------------------
 ********************************************************************************/

 /********************************************************************************
 *  INCLUDES
 ********************************************************************************/
#include "fiot_hal_adc.h"

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>
/********************************************************************************
* CONSTANTS
********************************************************************************/
#define Board_DIO23_ANALOG      PIN_UNASSIGNED
#define Board_DIO24_ANALOG      PIN_UNASSIGNED
#define Board_DIO25_ANALOG      PIN_UNASSIGNED
#define Board_DIO26_ANALOG      PIN_UNASSIGNED
#define Board_DIO27_ANALOG      PIN_UNASSIGNED
#define Board_DIO28_ANALOG      PIN_UNASSIGNED
#define Board_DIO29_ANALOG      PIN_UNASSIGNED
#define Board_DIO30_ANALOG      PIN_UNASSIGNED
/*
 *  ========================== ADC begin =========================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(ADC_config, ".const:ADC_config")
#pragma DATA_SECTION(adcCC26xxHWAttrs, ".const:adcCC26xxHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

/* ADC objects */
ADCCC26XX_Object  adcCC26xxObjects[ADC_CHANNEL_COUNT];

const ADCCC26XX_HWAttrs  adcCC26xxHWAttrs[ADC_CHANNEL_COUNT] = {
  {
    .adcDIO = Board_DIO23_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO7,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO24_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO6,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO25_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO5,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO26_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO4,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO27_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO3,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO28_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO2,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO29_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO1,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  },
  {
    .adcDIO = Board_DIO30_ANALOG,
    .adcCompBInput = ADC_COMPB_IN_AUXIO0,
    .refSource = ADCCC26XX_FIXED_REFERENCE,
    .samplingDuration = ADCCC26XX_SAMPLING_DURATION_10P9_MS,
    .inputScalingEnabled = true,
    .triggerSource = ADCCC26XX_TRIGGER_MANUAL
  }
};

const ADC_Config  ADC_config[] = {
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[0], &adcCC26xxHWAttrs[0]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[1], &adcCC26xxHWAttrs[1]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[2], &adcCC26xxHWAttrs[2]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[3], &adcCC26xxHWAttrs[3]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[4], &adcCC26xxHWAttrs[4]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[5], &adcCC26xxHWAttrs[5]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[6], &adcCC26xxHWAttrs[6]},
  {&ADCCC26XX_fxnTable, &adcCC26xxObjects[7], &adcCC26xxHWAttrs[7]},
  {NULL, NULL, NULL},
};
/*
 *  ========================== ADC end =========================================
 */
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/

/********************************************************************************
* LOCAL VARIABLES
********************************************************************************/
static ADC_Params  HAL_ADC_Params;
static ADC_Handle  HAL_ADC_Handle[ADC_CHANNEL_COUNT];
/********************************************************************************
* FUNCTIONS - LOCAL
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
FIOT_HAL_ADC_Init(HAL_ADC_Ref_T  iADC_Ref)
{
  //!Initializes the ADC driver
  ADC_init();

  return ADC_STATUS_OK;
}
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_Open(HAL_ADC_T  *iADC,
 *                                              HAL_ADC_Channel_T  iADC_Channel)
 *
 * @brief   Init channel of ADC
 *
 * @param   iADC          - Pointer to manage information of ADC
 * @param   iADC_Channel  - Channnel of ADC
 *
 * @return  ADC_STATUS_OK
 * @return  ADC_STATUS_ERROR_OPTION
 * @return  ADC_STATUS_ERROR_TIMEOUT
 ********************************************************************************/
extern HAL_ADC_Status_T  \
FIOT_HAL_ADC_Open(HAL_ADC_T  *iADC,
                  HAL_ADC_Channel_T  iADC_Channel)
{
  if(iADC == NULL)
    return ADC_STATUS_ERROR_NULL_PTR;

  iADC->ADC_Channel = iADC_Channel;

  ADC_Params_init(&HAL_ADC_Params);
  HAL_ADC_Handle[iADC_Channel] = ADC_open(iADC_Channel, &HAL_ADC_Params);
  if(HAL_ADC_Handle[iADC_Channel] == NULL)
    return ADC_STATUS_ERROR_NULL_PTR;
  else
    return ADC_STATUS_OK;
}
/*
 ********************************************************************************
 * @fn      HAL_ADC_Status_T  FIOT_HAL_ADC_Read(HAL_ADC_T  *iADC,
 *                                              HAL_ADC_Data_T  *oADC_Data)
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
                  HAL_ADC_Data_T  *oADC_Data)
{
  if((iADC == NULL) || (oADC_Data == NULL))
    return ADC_STATUS_ERROR_NULL_PTR;

  if(ADC_convert(HAL_ADC_Handle[iADC->ADC_Channel], oADC_Data) == ADC_STATUS_SUCCESS)
    return ADC_STATUS_OK;
  else
    return ADC_STATUS_ERROR_TIMEOUT;
}
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
                         HAL_ADC_Volt_T  *oADC_Data)
{
  if((iADC == NULL) || (oADC_Data == NULL))
    return ADC_STATUS_ERROR_NULL_PTR;

  uint16_t  ADC_rawValue = 0;
  if(ADC_convert(HAL_ADC_Handle[iADC->ADC_Channel], &ADC_rawValue) == ADC_STATUS_SUCCESS)
  {
    *oADC_Data = ADC_convertRawToMicroVolts(HAL_ADC_Handle[iADC->ADC_Channel], ADC_rawValue) / 1000;
    return ADC_STATUS_OK;
  }
  else
  {
    return ADC_STATUS_ERROR_TIMEOUT;
  }
}
/********************************************************************************
* END of fiot_hal_adc.c
********************************************************************************/