/********************************************************************************
 *  @Filename:    fiot_hal_serial_it.c
 *  @Author       Khang Vo
 *  @Version      1.0
 *  @Date         April 05, 2017
 *  @Description: Driver of Serial
 *    -------------------------------------------------
 *    |              fiot_hal_serial_it.c             |
 *    |                  TI CC2650                    |
 *    -------------------------------------------------
 *  @Note:
 *  _ fiot_hal_serial_it.c does not support auto_receive data for TI UARTCC26XX.
 *  _ 
 ********************************************************************************/
/********************************************************************************
 *  INCLUDES
 ********************************************************************************/
#include "fiot_hal_serial.h"
#include "cbuffer.h"

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
//!Pin Definition for USART modules
//!USART0 Pin
#define USART0_PIN_RX      IOID_2
#define USART0_PIN_TX      IOID_3
#define USART0_PIN_CTS     PIN_UNASSIGNED
#define USART0_PIN_RTS     PIN_UNASSIGNED

#define SERIAL_BUFFER_MAX_SIZE        (32)
/********************************************************************************
 *  ============================= UART begin ===================================
 ********************************************************************************/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartCC26XXHWAttrs, ".const:uartCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* UART objects */
UARTCC26XX_Object uartCC26XXObjects[SERIAL_COUNT];

/* UART hardware parameter structure, also used to assign UART pins */
const UARTCC26XX_HWAttrsV1  uartCC26XXHWAttrs[SERIAL_COUNT] = {
  {
    .baseAddr     = UART0_BASE,
    .powerMngrId  = PowerCC26XX_PERIPH_UART0,
    .intNum       = INT_UART0_COMB,
    .intPriority  = ~0,
    .swiPriority  = 0,
    .txPin        = USART0_PIN_TX,
    .rxPin        = USART0_PIN_RX,
    .ctsPin       = USART0_PIN_CTS,
    .rtsPin       = USART0_PIN_RTS
  }
};

/* UART configuration structure */
const UART_Config UART_config[] = {
  {
    .fxnTablePtr = &UARTCC26XX_fxnTable,
    .object      = &uartCC26XXObjects[0],
    .hwAttrs     = &uartCC26XXHWAttrs[0]
  },
  {NULL, NULL, NULL}
};

// const PIN_Config  UART_PinConfig[] = {
//     USART0_PIN_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
//     USART0_PIN_RX | PIN_INPUT_EN | PIN_HYSTERESIS | PIN_PULLDOWN,
//     USART0_PIN_CTS,
//     USART0_PIN_RTS
//     PIN_TERMINATE
// };
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef UART_Handle HAL_SERIAL_Handler_T;
typedef struct
{
  HAL_SERIAL_Data_T  Data[SERIAL_BUFFER_MAX_SIZE];
}
HAL_SERIAL_Callback_Buffer_T;
/********************************************************************************
 * VARIABLES
 ********************************************************************************/
static UART_Params  Serial_Params;
static HAL_SERIAL_Handler_T  Serial_Handler[SERIAL_COUNT];

static HAL_SERIAL_Callback_T  Serial_RxCB[SERIAL_COUNT] = { 0 };
static HAL_SERIAL_Callback_T  Serial_TxCB[SERIAL_COUNT] = { 0 };
/********************************************************************************
 * HANDLERS
 ********************************************************************************/
static void USART0_Rx_IRQHandler(UART_Handle iHandler,
                                 void *Serial_RxData, size_t Rx_DataCount);
static void USART0_Tx_IRQHandler(UART_Handle iHandler,
                                 void *Serial_TxData, size_t Tx_DataCount);

static void USART0_Rx_IRQHandler(UART_Handle iHandler,
                                 void *Serial_RxData, size_t Rx_DataCount)
{
  if((iHandler == Serial_Handler[0]) && (Rx_DataCount != 0))
  {
    //!Jump into Rx Callback function
    Serial_RxCB[0](SERIAL_OK);
  }
}

static void USART0_Tx_IRQHandler(UART_Handle iHandler,
                                 void *Serial_TxData, size_t Tx_DataCount)
{
  if ((iHandler == Serial_Handler[0]) && (Tx_DataCount != 0))
  {
    Serial_TxCB[0](SERIAL_OK);
  }
}
/********************************************************************************
* LOCAL FUNCTIONS
********************************************************************************/
static HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Convert_Settings(HAL_SERIAL_T  *iSerial,
                                 UART_Params   *iSerial_Params,
                                 HAL_SERIAL_Setting_T  iSerial_Settings);
static HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_IT_Configure(HAL_SERIAL_T  *iSerial,
                             HAL_SERIAL_Length_T  iSerial_RxCallback_Length,
                             HAL_SERIAL_Callback_T  iSerial_RxCallback,
                             HAL_SERIAL_Callback_T  iSerial_TxCallback);
/********************************************************************************
 * FUNCTIONS - API
 ********************************************************************************/

/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Init
 *
 * @brief   Config Parameter of Serial
 *          Update value for USART Callback Pointer
 *
 * @param   iSerial                   - Pointer of self
 *          iSerial_Port              - Serial port
 *          iSerial_Setting           - Setting of Serial
 *          iSerial_RxCallback_Length - Data Length of Rx Callback
 *          iSerial_RxCallback        - Address of Rx Callback
 *          iSerial_TxCallback        - Address of Tx Callback
 *
 * @return  SERIAL_OK
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Init(HAL_SERIAL_T  *iSerial,
                     HAL_SERIAL_Usart_T     iSerial_Port,
                     HAL_SERIAL_Setting_T   iSerial_Settings,
                     HAL_SERIAL_Length_T    iSerial_RxCallback_Length,
                     HAL_SERIAL_Callback_T  iSerial_RxCallback,
                     HAL_SERIAL_Callback_T  iSerial_TxCallback)
{
  HAL_SERIAL_Status_T  retStt;
  //!Checking address of iSerial
  if(iSerial == NULL)
    return SERIAL_ERROR_NULL_PTR;
  if(iSerial_Port >= SERIAL_COUNT)
    return SERIAL_ERROR_OPTION;
  //! Init UART module
  UART_init();
  //! Init UART Params to default settings
  UART_Params_init(&Serial_Params);
  //!Copy input settings to UART instance
  {
    iSerial->Serial_Port = iSerial_Port;
    iSerial->Serial_Settings.Serial_Baudrate = iSerial_Settings.Serial_Baudrate;
    iSerial->Serial_Settings.Serial_Data_Size = iSerial_Settings.Serial_Data_Size;
    iSerial->Serial_Settings.Serial_Parity = iSerial_Settings.Serial_Parity;
    iSerial->Serial_Settings.Serial_StopBit = iSerial_Settings.Serial_StopBit;
    iSerial->Serial_Settings.Serial_Mode = iSerial_Settings.Serial_Mode;
    iSerial->Serial_Settings.Serial_HFC = iSerial_Settings.Serial_HFC;
    iSerial->Serial_RxCallback = iSerial_RxCallback;
    iSerial->Serial_TxCallback = iSerial_TxCallback;
  }
  //!Convert Parameters of Serial to TI library parameters
  retStt = FIOT_HAL_SERIAL_Convert_Settings(iSerial,
                                            &Serial_Params,
                                            iSerial_Settings);
  if(retStt != SERIAL_OK)
    return retStt;

  //!Serial Interrupt Configuration
  retStt = FIOT_HAL_SERIAL_IT_Configure(iSerial,
                                        iSerial_RxCallback_Length,
                                        iSerial_RxCallback,
                                        iSerial_TxCallback);
  if(retStt != SERIAL_OK)
    return retStt;

  Serial_Handler[iSerial_Port] = UART_open(iSerial_Port, &Serial_Params);
  if(Serial_Handler[iSerial_Port] == NULL)
    return SERIAL_ERROR_NULL_PTR;

  //!Return Status
  return SERIAL_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Convert_Settings
 *
 * @brief   Convert Parameters of Serial to STM library parameters
 *
 * @param   iSerial         - Pointer of self
 *          iSerial_Setting - Setting of Serial
 *
 * @return  SERIAL_OK
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
static HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Convert_Settings(HAL_SERIAL_T  *iSerial,
                                 UART_Params   *iSerial_Params,
                                 HAL_SERIAL_Setting_T  iSerial_Settings)
{
  //!Set Baudrate for Serial Port
  iSerial_Params->baudRate = iSerial_Settings.Serial_Baudrate;
  //!Set data size to be 8-bit by default
  iSerial_Params->dataLength = UART_LEN_8;
  //!Set Stop-bit for Serial Port
  switch(iSerial_Settings.Serial_StopBit)
  {
    case STOPBIT_1_BIT:
      iSerial_Params->stopBits = UART_STOP_ONE;
      break;
    case STOPBIT_2_BIT:
      iSerial_Params->stopBits = UART_STOP_TWO;
      break;

    default:
      return SERIAL_ERROR_OPTION;
  }
  //!Set Parity mode
  switch(iSerial_Settings.Serial_Parity)
  {
    case PARITY_NONE:
      iSerial_Params->parityType = UART_PAR_NONE;
      break;
    case PARITY_EVEN:
      iSerial_Params->parityType = UART_PAR_EVEN;
      break;
    case PARITY_ODD:
      iSerial_Params->parityType = UART_PAR_ODD;
      break;

    default:
      return SERIAL_ERROR_OPTION;
  }
  //!Set transfer timeout
  iSerial_Params->readTimeout = UART_WAIT_FOREVER;
  iSerial_Params->readTimeout = UART_WAIT_FOREVER;
  //!This mode only functions when in UART_DATA_TEXT mode.
  iSerial_Params->readReturnMode = UART_RETURN_FULL;
  //!This mode only functions when in UART_DATA_TEXT mode.
  iSerial_Params->readEcho = UART_ECHO_OFF;
  //!Set Uart data mode
  iSerial_Params->readDataMode = UART_DATA_BINARY;
  iSerial_Params->writeDataMode = UART_DATA_BINARY;
  //!No HFC settings in TI's UART driver

  return SERIAL_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_IT_Configure
 *
 * @brief   Convert Parameters of Serial to STM library parameters
 *
 * @param   iSerial                   - Pointer of self
 *          iSerial_RxCallback_Length - Length of Rx Callback
 *          iSerial_RxCallback        - address of Rx Callback
 *          iSerial_TxCallback        - address of Tx Callback
 *
 * @return  SERIAL_OK
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
static HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_IT_Configure(HAL_SERIAL_T  *iSerial,
                             HAL_SERIAL_Length_T    iSerial_RxCallback_Length,
                             HAL_SERIAL_Callback_T  iSerial_RxCallback,
                             HAL_SERIAL_Callback_T  iSerial_TxCallback)
{
  if (iSerial_RxCallback_Length > SERIAL_BUFFER_MAX_SIZE)
    return SERIAL_ERROR_OPTION;
  //!Set User's Callback
  Serial_TxCB[iSerial->Serial_Port] = iSerial_TxCallback;
  Serial_RxCB[iSerial->Serial_Port] = iSerial_RxCallback;
  //!Set transfer mode to Callback
  Serial_Params.readMode = UART_MODE_CALLBACK;
  Serial_Params.writeMode = UART_MODE_CALLBACK;
  //!Set interrupt handler
  Serial_Params.readCallback = USART0_Rx_IRQHandler;
  Serial_Params.writeCallback = USART0_Tx_IRQHandler;

  return SERIAL_OK;
}
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Read
 *
 * @brief   Config total of bytes that need be read on serial
 *
 * @param   iSerial       - Pointer of self
 *          Serial_RxData  - Pointer of Receive Buffer
 *          Serial_nBytes  - Total of Receive byte.
 *
 * @return  SERIAL_OK
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Read(HAL_SERIAL_T  *iSerial,
                     HAL_SERIAL_Data_T   *oSerial_RxData,
                     HAL_SERIAL_Length_T  iSerial_nBytes)
{
  //!Checking address of iSerial
  if(iSerial == NULL)
    return SERIAL_ERROR_NULL_PTR;
  if(iSerial->Serial_Port >= SERIAL_COUNT)
    return SERIAL_ERROR_OPTION;

  if (iSerial_nBytes != 0)
  {
    //!Start reading
    UART_read(Serial_Handler[iSerial->Serial_Port], oSerial_RxData, iSerial_nBytes);
  }

  //!Return Status
  return SERIAL_RX_WAIT;
}
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Write
 *
 * @brief   Config total of bytes that need be write on serial
 *
 * @param   iSerial       - Pointer of self
 *          Serial_TxData  - Pointer of Transmit Buffer
 *          Serial_nBytes  - Total of Transmit byte.
 *
 * @return  SERIAL_OK
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Write(HAL_SERIAL_T  *iSerial,
                      HAL_SERIAL_Data_T  *iSerial_TxData,
                      HAL_SERIAL_Length_T  iSerial_nBytes)
{
  //!Checking address of iSerial
  if(iSerial == NULL)
    return SERIAL_ERROR_NULL_PTR;
  if(iSerial->Serial_Port >= SERIAL_COUNT)
    return SERIAL_ERROR_OPTION;

  UART_write(Serial_Handler[iSerial->Serial_Port], iSerial_TxData, iSerial_nBytes);

  return SERIAL_TX_WAIT;
}

extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Refresh(HAL_SERIAL_T  *iSerial)
{
  if(iSerial == NULL)
  {
    return SERIAL_ERROR_NULL_PTR;
  }
  
  UART_close(Serial_Handler[0]);
  Serial_Handler[0] = UART_open(iSerial->Serial_Port, &Serial_Params);
  return SERIAL_OK;
}
/********************************************************************************/