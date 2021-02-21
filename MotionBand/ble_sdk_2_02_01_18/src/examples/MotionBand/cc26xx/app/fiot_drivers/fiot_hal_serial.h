/********************************************************************************
 *  @Filename:    fiot_hal_serial.h
 *  @Author       Khang Vo
 *  @Version      1.4
 *  @Date         April 05, 2017
 *  @Description: Driver of Serial 
 *    -------------------------------------------------
 *    |                 fiot_hal_serial.h             |        
 *    -------------------------------------------------
 *  @Notes:
 *  _ fiot_hal_serial v1.4 does not support auto-receive data for CC26xx
 *
 *
 *  @Example: Configure USART2 and USART3 to receive data from USART2 and
 *              transmit to USART3
 *    //!CONSTANTS USED:
 *    #define   UART_BUFFER_LENGTH  64
 *    #define   UART_CB_LENGTH      1
 *
 *    //!TYPEDEF AND VARIABLES USED:
 *    typedef struct
 *    {
 *      HAL_SERIAL_T          Serial;
 *      HAL_SERIAL_Status_T   Serial_RxStatus;
 *      HAL_SERIAL_Status_T   Serial_TxStatus;
 *    }
 *    HAL_SERIAL_Manager_T;
 *
 *    HAL_SERIAL_Manager_T    mUSART2; 
 *    HAL_SERIAL_Manager_T    mUSART3;
 *    
 *    //!BUFFERS:
 *    uint8_t uart_receiveBuffer[UART_BUFFER_LENGTH] = { 0 };
 *    uint8_t data_length = 0;
 *    CBuffer_T   mCBuffer;
 *
 *    //!CALLBACKS PROTOTYPES AND IMPLEMENTATION:
 *    void Serial2_CB_Receive(HAL_SERIAL_Status_T oStatus);
 *    void Serial2_CB_Transmit(HAL_SERIAL_Status_T oStatus);
 *    void Serial3_CB_Receive(HAL_SERIAL_Status_T oStatus);
 *    void Serial3_CB_Transmit(HAL_SERIAL_Status_T oStatus);
 *    
 *    //! Callback function:
 *    void Serial2_CB_Receive(HAL_SERIAL_Status_T oStatus)
 *    {
 *      uint8_t rxBuffer[UART_BUFFER_LENGTH] = { 0 };
 *    
 *      mUSART2.Serial_RxStatus = oStatus;
 *    
 *      FIOT_HAL_SERIAL_Read(&mUSART2.Serial, rxBuffer, UART_CB_LENGTH);
 *    
 *      CB_Write(&mCBuffer,rxBuffer,UART_CB_LENGTH);  
 *    }
 *    
 *    void Serial2_CB_Transmit(HAL_SERIAL_Status_T oStatus)
 *    {
 *      mUSART2.Serial_TxStatus = oStatus;
 *    }
 *    
 *    void Serial3_CB_Receive(HAL_SERIAL_Status_T oStatus)
 *    {
 *      mUSART3.Serial_RxStatus = oStatus;
 *    }
 *    
 *    void Serial3_CB_Transmit(HAL_SERIAL_Status_T oStatus)
 *    {
 *      mUSART3.Serial_TxStatus = oStatus;
 *    }
 *    
 *    //!MAIN FUNCTIONS: (HAL_Init, SystemClock_Config and GPIO should be configured before)
 *
 *    //!USART2 Init:
 *    mUSART2.Serial_RxStatus = SERIAL_OK;
 *    mUSART2.Serial_TxStatus = SERIAL_OK;
 *    //!Select USART2:
 *    mUSART2.Serial.Serial_Port = SERIAL_2;
 *    //!Setting baudrate to be 57600:
 *    mUSART2.Serial.Serial_Settings.Serial_Baudrate = BAUDRATE_57600;
 *    //!Enabling CTS/RTS flow control:
 *    mUSART2.Serial.Serial_Settings.Serial_HFC = HFC_CTS_RTS;
 *    //!Using no parity bit:
 *    mUSART2.Serial.Serial_Settings.Serial_Parity = PARITY_NONE;
 *    //!Using 1 stop bit:
 *    mUSART2.Serial.Serial_Settings.Serial_StopBit = STOPBIT_1_BIT;
 *    
 *    //!Setting USART2 using above settings:
 *    mUSART2.Serial_RxStatus = FIOT_HAL_SERIAL_Configure(&mUSART2.Serial, 
 *                                                       mUSART2.Serial.Serial_Port, 
 *                                                       mUSART2.Serial.Serial_Settings, 
 *                                                       UART_CB_LENGTH, 
 *                                                       Serial2_CB_Receive,
 *                                                       Serial2_CB_Transmit);
 *    
 *    //!USART3 Init:
 *    mUSART3.Serial_RxStatus = SERIAL_OK;
 *    mUSART3.Serial_TxStatus = SERIAL_OK;
 *    mUSART3.Serial.Serial_Port = SERIAL_3;
 *    mUSART3.Serial.Serial_Settings.Serial_Baudrate = BAUDRATE_115200;
 *    mUSART3.Serial.Serial_Settings.Serial_HFC = HFC_DISABLE;
 *    mUSART3.Serial.Serial_Settings.Serial_Parity = PARITY_NONE;
 *    mUSART3.Serial.Serial_Settings.Serial_StopBit = STOPBIT_1_BIT;
 *    
 *    mUSART3.Serial_RxStatus = FIOT_HAL_SERIAL_Configure(&mUSART3.Serial, 
 *                                                       mUSART3.Serial.Serial_Port, 
 *                                                       mUSART3.Serial.Serial_Settings, 
 *                                                       UART_CB_LENGTH, 
 *                                                       Serial3_CB_Receive,
 *                                                       Serial3_CB_Transmit);
 *    //!Init Circuler Buffer:
 *    CB_Init(&mCBuffer, uart_receiveBuffer, UART_BUFFER_LENGTH);
 *
 *    //!Infinite Loop to transfer received data from USART2 to USART3:
 *    while (1)
 *    {
 *      uint32_t  DataLength = CB_DataCount(&mCBuffer);
 *      if(DataLength > 0)
 *      {
 *        //!Checking Transmit status:
 *        if(mUSART3.Serial_TxStatus == SERIAL_OK)
 *        {
 *          //!Transmit buffer:
 *          uint8_t txBuffer[UART_BUFFER_LENGTH] = { 0 };
 *          if(DataLength > UART_BUFFER_LENGTH)
 *          {
 *            DataLength = UART_BUFFER_LENGTH;
 *          }
 *          //!Copying data from circular buffer to txBuffer to transmit:
 *          CB_Read(&mCBuffer, txBuffer, DataLength);
 *          //!Transmit data using USART3:      
 *          mUSART3.Serial_TxStatus = FIOT_HAL_SERIAL_Write(&mUSART3.Serial, txBuffer, DataLength);               
 *        }
 *      }
 *    }
 *
 ********************************************************************************/

#ifndef __HAL_SERIAL_H__
#define __HAL_SERIAL_H__

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
//!Priority of SERIAL1
#define IRQn_Priority_SERIAL1      6
//!Priority of SERIAL2
#define IRQn_Priority_SERIAL2      6
//!Priority of SERIAL3
#define IRQn_Priority_SERIAL3      6
//!Priority of SERIAL4
#define IRQn_Priority_SERIAL4      6
//!Priority of SERIAL5
#define IRQn_Priority_SERIAL5      6
//!Priority of SERIAL6
#define IRQn_Priority_SERIAL6      6
#endif

#ifdef __FIOT_STM32F411_
//!Priority of USART1
#define IRQn_Priority_SERIAL1      6
//!Priority of USART2
#define IRQn_Priority_SERIAL2      6
//!Priority of USART6
#define IRQn_Priority_SERIAL6      6
#endif

#ifdef __FIOT_SAM3X8E_
#define IRQn_Priority_SERIAL0      6
#define IRQn_Priority_SERIAL1      6
#define IRQn_Priority_SERIAL2      6
#define IRQn_Priority_SERIAL3      6
#endif
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
//!Options of Status
typedef enum
{
  SERIAL_OK,          //!Receive and Transmit Flag is disabled
  SERIAL_RX_WAIT,     //!Receive Flag is enabled to wait for a new Data
  SERIAL_TX_WAIT,     //!Transmit Flag is enabled to wait for transmit is completed
  SERIAL_ERROR_OPTION, //!Error
  SERIAL_ERROR_NULL_PTR, //!Error
}
HAL_SERIAL_Status_T;
//!Options of Baudrate
typedef enum
{
#ifdef __FIOT_ATMEGA328P_
  BAUDRATE_1200   =  1200,
  BAUDRATE_2400   =  2400,
  BAUDRATE_4800   =  4800,
  BAUDRATE_9600   =  9600,
  BAUDRATE_19200  =  19200,
  BAUDRATE_38400  =  38400,
  BAUDRATE_57600  =  57600
#else
  BAUDRATE_9600   =  9600,
  BAUDRATE_14400  =  14400,
  BAUDRATE_19200  =  19200,
  BAUDRATE_38400  =  38400,
  BAUDRATE_57600  =  57600,
  BAUDRATE_115200 =  115200
#endif
}
HAL_SERIAL_Baudrate_T;

//!Options of Data Size
typedef enum
{
  DATA_SIZE_8_BIT
}
HAL_SERIAL_Data_Size_T;
//!Options of Parity
typedef enum
{
  PARITY_NONE,  //!No parity,
  PARITY_EVEN,  //!Even parity 
  PARITY_ODD    //!Odd parity
}
HAL_SERIAL_Parity_T;
//!Options of StopBit
typedef enum
{
  STOPBIT_1_BIT,  //!1 stop bit
  STOPBIT_2_BIT   //!2 stop bits
}
HAL_SERIAL_StopBit_T;
//!Options of Serial Mode 
typedef enum
{
  MODE_ASYNCHRONOUS,  //!Normal mode
  MODE_SYNCHRONOUS    //!Not supported
}
HAL_SERIAL_Mode_T;
//!Options of Hardware Flow Control
#ifndef __FIOT_ATMEGA328P_  //! No Hardware Flow Control in ATMega328P
typedef enum
{
  HFC_DISABLE,
  HFC_CTS,
  HFC_RTS,
  HFC_CTS_RTS
}
HAL_SERIAL_HFC_T;
#endif

//!Options of Supported Serial
typedef enum
{
#ifdef __FIOT_STM32F407_
  SERIAL_1,
  SERIAL_2,
  SERIAL_3,
  SERIAL_4,
  SERIAL_5,
  SERIAL_6,
  SERIAL_COUNT
#endif

#ifdef __FIOT_STM32F411_
  SERIAL_1,
  SERIAL_2,
  SERIAL_6,
  SERIAL_COUNT
#endif

#ifdef __FIOT_SAM3X8E_
  SERIAL_0,
  SERIAL_1,
  SERIAL_2,
  SERIAL_3,
  SERIAL_COUNT
#endif

#ifdef __FIOT_ATMEGA328P_
  SERIAL_0,
  SERIAL_COUNT
#endif

#ifdef __FIOT_CC2650_
  SERIAL_0,
  SERIAL_COUNT
#endif
}
HAL_SERIAL_Usart_T;

//!Type of Serial Setting
typedef struct
{
  HAL_SERIAL_Baudrate_T   Serial_Baudrate;
  HAL_SERIAL_Data_Size_T  Serial_Data_Size;
  HAL_SERIAL_Parity_T     Serial_Parity;
  HAL_SERIAL_StopBit_T    Serial_StopBit;
  HAL_SERIAL_Mode_T    Serial_Mode;
  #ifndef __FIOT_ATMEGA328P_
  HAL_SERIAL_HFC_T  Serial_HFC;
  #endif
}
HAL_SERIAL_Setting_T;
//!Type of Serial Data
typedef uint8_t  HAL_SERIAL_Data_T;
//!Type of Serial Data Length
typedef uint16_t  HAL_SERIAL_Length_T;
//!Type of Serial Callback
typedef void (*HAL_SERIAL_Callback_T)(HAL_SERIAL_Status_T oStatus);
//!Type of Serial information
typedef struct
{
  HAL_SERIAL_Usart_T  Serial_Port;
  HAL_SERIAL_Setting_T  Serial_Settings;
  HAL_SERIAL_Callback_T  Serial_TxCallback;
  HAL_SERIAL_Callback_T  Serial_RxCallback; 
}
HAL_SERIAL_T;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

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
                     HAL_SERIAL_Callback_T  iSerial_TxCallback);
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Read
 *
 * @brief   Config total of bytes that need be read on serial
 *
 * @param   iSerial       - Pointer of self
 *          Serial_RxData - Pointer of Receive Buffer
 *          Serial_nBytes - Total of Receive byte. 
 *
 * @return  SERIAL_RX_WAIT
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Read(HAL_SERIAL_T  *iSerial,
                     HAL_SERIAL_Data_T  *oSerial_RxData,
                     HAL_SERIAL_Length_T  iSerial_nBytes);
/********************************************************************************
 * @fn      FIOT_HAL_SERIAL_Write
 *
 * @brief   Config total of bytes that need be write on serial
 *
 * @param   iSerial       - Pointer of self
 *          Serial_TxData  - Pointer of Transmit Buffer
 *          Serial_nBytes  - Total of Transmit byte. 
 *
 * @return  SERIAL_TX_WAIT
 *          SERIAL_ERROR_OPTION
 ********************************************************************************/
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Write(HAL_SERIAL_T  *iSerial,
                      HAL_SERIAL_Data_T  *iSerial_TxData,
                      HAL_SERIAL_Length_T  iSerial_nBytes);
 
extern HAL_SERIAL_Status_T  \
FIOT_HAL_SERIAL_Refresh(HAL_SERIAL_T  *iSerial);
/********************************************************************************/
#ifdef __cplusplus
}
#endif

#endif  /* __HAL_SERIAL_H__ */