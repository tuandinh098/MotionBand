/*! 
 *	@file				MB_Uart.c
 *  @author  		Dinh Le
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2017-04-12
 *	@brief			Protocol packet between STM, CC2650
 *
 */
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
#include "MB_Uart.h"
#include "Board.h"
#include "string.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/PIN/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <bcomdef.h>
#include "peripheral.h"
#include "hal_mcu.h"
#include "MB_Main.h"
#include "fiot_hal_serial.h"
/********************************************************************************
 *	PACKET STRUCTURE
 ********************************************************************************/
#define SBP_UART_CHANGE_EVT           (0x0010)

#define UART_HEADER1_INDEX						(0)
#define UART_HEADER2_INDEX 						(1)
#define UART_FRAME_LENGTH_INDEX 			(2)
#define UART_COMMAND_INDEX						(3)
#define UART_FIRST_ADDRESS_L_INDEX		(4)
#define UART_FIRST_ADDRESS_H_INDEX		(5)
#define UART_START_DATA_INDEX					(6)
/********************************************************************************
 *	CONSTANTS
 ********************************************************************************/
#define UART_BUFFER_SIZE           100
#define UART_TASK_STACK_SIZE       644
#define UART_TASK_PRIORITY         1
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef enum
{
	MB_UART_STATUS_OK,
	MB_UART_STATUS_ERROR
}UART_Status_T; 
typedef struct
{
  UART_Data_T Header;
  UART_Data_T Cmd;
  UART_Data_T DataLen;
  UART_Data_T Data[PACKET_DATA_LENGTH]; 
}MB_Packet_T;
typedef struct
{
  HAL_SERIAL_T          Serial;
  HAL_SERIAL_Status_T   Serial_RxStatus;
  HAL_SERIAL_Status_T   Serial_TxStatus;
}
HAL_SERIAL_Manager_T;
/********************************************************************************
 * VARIABLES
 ********************************************************************************/
uint8_t     			        UART_TaskStack[UART_TASK_STACK_SIZE];
Task_Struct 			        UART_TaskStruct;
Semaphore_Struct 	        UART_sSem;
Semaphore_Handle 	        UART_hSem;
UART_Data_T 			        UART_Rx[PACKET_LENGTH] = {0x00};
UART_Data_T 			        UART_rBuffer_Data[UART_BUFFER_SIZE]={0x00};
UART_Length_T 		        UART_rBuffer_Length = 0;
UART_Data_T               PACKET_Buffer[23];
Packet_Status_T           PACKET_Status;  
MB_Packet_T               MB_Packet;
HAL_SERIAL_Manager_T      mUSART; 
/********************************************************************************
 * FUNCTIONS - LOCAL
 ********************************************************************************/
static void FIOT_UART_InitTask(); 
static void FIOT_UART_TaskFxn(UArg a0, UArg a1);
static void MB_UART_Clear_rBuffer(void);
static void MB_UART_Serial_Write(UART_Data_T	*iData, UART_Length_T nBytes);
static void MB_UART_Serial_Read(UART_Data_T	*iData, UART_Length_T nBytes);
static Packet_Status_T MB_PACKET_AnalyzeData(UART_Data_T *iData);
/********************************************************************************
 * HANDLER
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      void FIOT_UART_Serial_Rx_Cb(UART_Handle iHandle, void *oData, size_t count)
 * 
 * @brief   Callback function when there is the data on serial
 * 
 * @param   iHandle -	handle of callback serial
 * @param   oData   -	pointer of receive buffer
 * @param   count   -	total of callback bytes  
 * 
 * @return  None
 *
 ********************************************************************************/ 
void Serial_CB_Receive(HAL_SERIAL_Status_T oStatus)
{
  //!Wake task to process
  Semaphore_post(UART_hSem);
  mUSART.Serial_RxStatus = oStatus;
}
 
void Serial_CB_Transmit(HAL_SERIAL_Status_T oStatus)
{
  if(mUSART.Serial_TxStatus == SERIAL_TX_WAIT)
  {
    Semaphore_post(UART_hSem);
  }
  mUSART.Serial_TxStatus = oStatus;
} 
/********************************************************************************
 * FUNCTIONS - TASK
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      void FIOT_UART_CreateTask(void)
 * 
 * @brief   Task creation function for the protocol
 * 
 * @param   None
 * 
 * @return  None
 *
 ********************************************************************************/ 
void FIOT_UART_CreateTask(void)
{
  Task_Params TaskParams;
	//!Configure task
  Task_Params_init(&TaskParams);
  TaskParams.stack 			= UART_TaskStack;
  TaskParams.stackSize 	= UART_TASK_STACK_SIZE;
  TaskParams.priority 	= UART_TASK_PRIORITY;
  Task_construct(&UART_TaskStruct, FIOT_UART_TaskFxn, &TaskParams, NULL);
}
/*!
 ********************************************************************************
 * @fn      void FIOT_UART_InitTask(void)
 * 
 * @brief   Initilize UART layer
 * 
 * @param   None
 * 
 * @return  None
 *
 ********************************************************************************/
void FIOT_UART_InitTask(void)
{
	//!Initilize Value of Packet
  MB_Packet.Header      = 0x00;
  MB_Packet.Cmd         = 0x00;
  MB_Packet.DataLen     = 0x00;
  memset(MB_Packet.Data,0x00,sizeof(MB_Packet.Data));
	//!Initilize Buffer
	memset(UART_rBuffer_Data,0x00,UART_BUFFER_SIZE);
	UART_rBuffer_Length = 0;

  //!Initilize UART	
  mUSART.Serial_RxStatus = SERIAL_OK;
  mUSART.Serial_TxStatus = SERIAL_OK;
  mUSART.Serial.Serial_Port = SERIAL_0;
  mUSART.Serial.Serial_Settings.Serial_Baudrate = BAUDRATE_115200;
  mUSART.Serial.Serial_Settings.Serial_HFC = HFC_DISABLE;
  mUSART.Serial.Serial_Settings.Serial_Parity = PARITY_NONE;  
  mUSART.Serial.Serial_Settings.Serial_StopBit = STOPBIT_1_BIT;

  mUSART.Serial_RxStatus = FIOT_HAL_SERIAL_Init(&mUSART.Serial, 
                                                 mUSART.Serial.Serial_Port, 
                                                 mUSART.Serial.Serial_Settings, 
                                                 NULL, 
                                                 Serial_CB_Receive,
                                                 Serial_CB_Transmit);  
	//!Initilize Semaphore
  Semaphore_Params sParams;
  Semaphore_Params_init(&sParams);
  sParams.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&UART_sSem, 0, &sParams);
  UART_hSem = Semaphore_handle(&UART_sSem); 
  
}
/*!
 ********************************************************************************
 * @fn      void FIOT_UART_TaskFxn(UArg a0, UArg a1)
 * 
 * @brief   Application task entry point for the protocol.
 * 
 * @param   a0, a1 - not used.
 * 
 * @return  None
 *
 ********************************************************************************/
void FIOT_UART_TaskFxn(UArg a0, UArg a1)
{		
  //!Initilize uart layer
  FIOT_UART_InitTask();    
	//!Loop of uart layer	
  for (;;)
  {
    MB_UART_Serial_Read(UART_Rx, PACKET_LENGTH); 
    Semaphore_pend(UART_hSem, BIOS_WAIT_FOREVER);
    memcpy(&UART_rBuffer_Data[UART_rBuffer_Length], UART_Rx, PACKET_LENGTH);
    UART_rBuffer_Length+=PACKET_LENGTH;
    if(UART_rBuffer_Length == PACKET_LENGTH)
    {
      //!Analyze data
      PACKET_Status = MB_PACKET_AnalyzeData(UART_rBuffer_Data);
      //!Clear buffer
      MB_UART_Clear_rBuffer();
      if (PACKET_Status == PACKET_TRUE)
      {     
         //!Check PACKET_CMD_REFRESH_UART, should not enqueue
         if(MB_Packet.Cmd == PACKET_CMD_REFRESH_UART)
         {
           if (MB_Packet.DataLen != PACKET_REFRESH_LENGTH)
           {
             MB_PACKET_Write_Error();
           }
           else
           {
             FIOT_HAL_SERIAL_Refresh(&mUSART.Serial);
             UART_Data_T data = PACKET_REFRESH_DATA;
             MB_PACKET_Write(PACKET_CMD_REFRESH_UART, PACKET_REFRESH_LENGTH, &data);
           }
         }
         else if (MB_Packet.Cmd == PACKET_CMD_RESET)
         {
           if (MB_Packet.DataLen != PACKET_RESET_LENGTH)
           {
             MB_PACKET_Write_Error();
           }
           else
           {
              UART_Data_T data = PACKET_RESET_DATA;
              MB_PACKET_Write(PACKET_CMD_RESET, PACKET_RESET_LENGTH, &data);
              mUSART.Serial_TxStatus = SERIAL_TX_WAIT; 
              Semaphore_pend(UART_hSem, BIOS_WAIT_FOREVER);
              HAL_SYSTEM_RESET();
           }
         }
         else
         {
           MB_enqueueMsg(SBP_UART_CHANGE_EVT, MB_Packet.Cmd);
         }    
      }
      else 
      {
         MB_PACKET_Write_Error();
      }
    }
  }
}
/********************************************************************************
 * FUNCTIONS - API
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      MB_PACKET_Write
 * 
 * @brief   CC2650 sends packet to STM
 * 
 * @param   iCmd: command in packet
 * @param   iLen: length of data in packet
 * @param   iData: =data in packet
 *
 * @return  None
 *
 ********************************************************************************/
void MB_PACKET_Write(UART_Data_T iCmd, UART_Data_T iLen, UART_Data_T *iData)
{
  UART_Data_T   packet[PACKET_LENGTH] = {0x00};
  //! Copy to buffer
  packet[PACKET_HEADER_OFFSET] = PACKET_HEADER_VALUE_DEFAULT;
  packet[PACKET_CMD_OFFSET] = iCmd;
  packet[PACKET_DATALENGTH_OFFSET] = iLen;
  VOID memcpy(&packet[PACKET_DATA_OFFSET], iData, iLen);
  //! Send data
  MB_UART_Serial_Write(packet, PACKET_LENGTH);
}
/*!
 ********************************************************************************
 * @fn      MB_PACKET_Write_Error
 * 
 * @brief   CC2650 sends ERROR packet to STM
 * 
 * @param   None
 *
 * @return  None
 *
 ********************************************************************************/
void MB_PACKET_Write_Error()
{
  UART_Data_T   packet[PACKET_LENGTH] = {0x00};
  //! Copy to buffer
  packet[PACKET_HEADER_OFFSET] = PACKET_HEADER_VALUE_DEFAULT;
  packet[PACKET_CMD_OFFSET] = PACKET_CMD_ERROR;
  packet[PACKET_DATALENGTH_OFFSET] = PACKET_ERROR_LENGTH;
  //! Send data
  MB_UART_Serial_Write(packet, PACKET_LENGTH);
}
/*!
 ********************************************************************************
 * @fn      MB_PACKET_Write_Error_TO
 * 
 * @brief   CC2650 sends ERROR TIMEOUT packet to STM
 * 
 * @param   None
 *
 * @return  None
 *
 ********************************************************************************/
void MB_PACKET_Write_Error_TO()
{
  UART_Data_T   packet[PACKET_LENGTH] = {0x00};
  //! Copy to buffer
  packet[PACKET_HEADER_OFFSET] = PACKET_HEADER_VALUE_DEFAULT;
  packet[PACKET_CMD_OFFSET] = PACKET_CMD_ERROR_TO;
  packet[PACKET_DATALENGTH_OFFSET] = PACKET_ERROR_TO_LENGTH;
  //! Send data
  MB_UART_Serial_Write(packet, PACKET_LENGTH);  
}
/*!
 ********************************************************************************
 * @fn      MB_PACKET_Read
 * 
 * @brief   Read packet is sent from STM
 * 
 * @param   None
 *
 * @return  None
 *
 ********************************************************************************/
void MB_PACKET_Read(uint8_t *oData,uint8_t *oLen)
{
  memcpy(oData,MB_Packet.Data,MB_Packet.DataLen);
  *oLen = MB_Packet.DataLen;
}
/********************************************************************************
 * LOCAL FUNCTIONS
 ********************************************************************************/
/*!
 ********************************************************************************
 * @fn      MB_UART_Clear_rBuffer
 * 
 * @brief   Clear Buffer Rx Buffer
 * 
 * @param   None
 * 
 * @return  None
 *
 ********************************************************************************/ 
void MB_UART_Clear_rBuffer(void)
{
	memset(UART_rBuffer_Data,0x00,UART_BUFFER_SIZE);
	UART_rBuffer_Length = 0;	
}
/*!
 ********************************************************************************
 * @fn      MB_UART_Serial_Write
 * 
 * @brief   Write data to STM
 * 
 * @param   iData 	-	input data 
 * @param   nBytes -	length of input data
 * 
 * @return  None
 *
 ********************************************************************************/
void MB_UART_Serial_Write(UART_Data_T	*iData, UART_Length_T nBytes)
{
  FIOT_HAL_SERIAL_Write(&mUSART.Serial, iData, nBytes);
}
/*!
 ********************************************************************************
 * @fn      MB_UART_Serial_Read
 * 
 * @brief   Read data from STM
 * 
 * @param   iData 	-	input data 
 * @param   nBytes -	length of input data
 * 
 * @return  None
 *
 ********************************************************************************/
void MB_UART_Serial_Read(UART_Data_T	*iData, UART_Length_T nBytes)
{
  FIOT_HAL_SERIAL_Read(&mUSART.Serial, iData, nBytes); 
}
/*!
 ********************************************************************************
 * @fn      MB_PACKET_AnalyzeData
 * 
 * @brief   Check packet is sent from STM
 * 
 * @param   iData 	-	input data 
 * @param   iLength -	length of input data
 * 
 * @return  None
 *
 ********************************************************************************/
Packet_Status_T MB_PACKET_AnalyzeData(UART_Data_T *iData)
{
  //!Checking header
  if (iData[PACKET_HEADER_OFFSET] != PACKET_HEADER_VALUE_DEFAULT)
  {
    return PACKET_FALSE;
  }
  
  //!Checking command
  if ((iData[PACKET_CMD_OFFSET] != PACKET_CMD_STATUS_BLE) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_DIS_BLE) &&
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_ENA_AD) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_RESET) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_REFRESH_UART) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_RES) &&
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_IND) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_DATA1) &&
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_DATA2) && 
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_DATA3) &&
      (iData[PACKET_CMD_OFFSET] != PACKET_CMD_DATA4))
  {
    return PACKET_FALSE;
  }
  //!Checking length of data
  if(iData[PACKET_DATALENGTH_OFFSET] > (UART_Data_T)PACKET_DATA_LENGTH)
  {
    return PACKET_FALSE;
  }
  //!Checking data, if padding of packet is not 0, packet is not correct
  UART_Length_T numberdummybyte = PACKET_DATA_LENGTH - iData[PACKET_DATALENGTH_OFFSET];
  UART_Length_T dummybte[PACKET_DATA_LENGTH] = {0x00}; 
  if (memcmp(dummybte, &iData[PACKET_DATA_OFFSET + iData[PACKET_DATALENGTH_OFFSET]], numberdummybyte))
  {
    return PACKET_FALSE;
  }
  
  //!If check data is all right, analizing packet
  MB_Packet.Header  = iData[PACKET_HEADER_OFFSET];
  MB_Packet.Cmd     = iData[PACKET_CMD_OFFSET];
  MB_Packet.DataLen = iData[PACKET_DATALENGTH_OFFSET];
  memset(MB_Packet.Data, 0x00, sizeof(MB_Packet.Data));
  memcpy(MB_Packet.Data, &iData[PACKET_DATA_OFFSET], MB_Packet.DataLen);
  //!If packet correct
  return PACKET_TRUE;   
}
/********************************************************************************
 * END
 ********************************************************************************/