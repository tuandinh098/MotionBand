/*! 
 *	@file				MB_Uart.h
 *  @author  		Dinh Le
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2017-04-12
 *	@brief			Protocol packet between STM, CC2650
 *
 */
#ifndef __MB_UART_H__
#define __MB_UART_H__

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
#include "stdint.h"
/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
//!Packet receive from STM 's structure
#define PACKET_LENGTH                            (23)
//!Header
#define PACKET_HEADER_VALUE_DEFAULT              (0x55)
#define PACKET_HEADER_LENGTH                     (1)
#define PACKET_HEADER_OFFSET                     (0)
   
//!Command   
#define PACKET_CMD_LENGTH                       (1)
#define PACKET_CMD_OFFSET                       (1)
#define PACKET_CMD_NUMBER                       (12)

//!Length of data
#define PACKET_DATALENGTH_LENGTH                (1)
#define PACKET_DATALENGTH_OFFSET                (2)
#define PACKET_DATALENGTH_VALUE_DEFAULT         (0x01)

//!Data
#define PACKET_DATA_OFFSET                      (3)
#define PACKET_DATA_LENGTH                      (20)
#define PACKET_DATA_VALUE_DEFAULT               (0x00)
#define PACKET_DATA_FIRSTBYTE                   (0)

//!PACKET Type
//!PACKET ERROR Structure
#define PACKET_ERROR_LENGTH                     (0x00)
#define PACKET_ERROR_DATA                       (0x00)

//!PACKET ERROR TO Structure
#define PACKET_ERROR_TO_LENGTH                  (0x00)
#define PACKET_ERROR_TO_DATA                    (0x00)
   
//!PACKET ERROR BLE Structure
#define PACKET_ERROR_BLE_LENGTH                 (0x00)
#define PACKET_ERROR_BLE_DATA                   (0x00)

// PACKET Status BLE Connection
#define PACKET_STATUS_REQ_LENGTH                (0x00)
#define PACKET_STATUS_RES_LENGTH                (0x01)
#define PACKET_STATUS_DATA_CONNECTED            (0x01)    
#define PACKET_STATUS_DATA_NOTCONNECT           (0x00)

// PACKET Disconnect BLE Connection
#define PACKET_DIS_REQ_LENGTH                   (0x00)
#define PACKET_DIS_RES_LENGTH                   (0x01)
#define PACKET_DIS_DATA_SUCCESS                 (0x01)
#define PACKET_DIS_DATA_FAIL                    (0x00)

// PACKET Ena/Dis BLE Advertisement
#define PACKET_ENA_DIS_AD_REQ_LENGTH            (0x01)
#define PACKET_ENA_DIS_AD_RES_LENGTH            (0x01)
#define PACKET_ENA_AD_DATA                      (0x01)
#define PACKET_DIS_AD_DATA                      (0x00)      

// PACKET Reset MCU
#define PACKET_RESET_LENGTH                     (0x00)
#define PACKET_RESET_DATA                       (0x00)

// PACKET Refresh UART
#define PACKET_REFRESH_LENGTH                   (0x00)
#define PACKET_REFRESH_DATA                     (0x00)
/********************************************************************************
 * TYPEDEF
 ********************************************************************************/
typedef enum
{
  PACKET_TRUE,
  PACKET_FALSE
}Packet_Status_T;
typedef enum
{
  PACKET_CMD_STATUS_BLE = 0x01,
  PACKET_CMD_DIS_BLE,
  PACKET_CMD_ENA_AD,
  PACKET_CMD_RESET,
  PACKET_CMD_REFRESH_UART,
  PACKET_CMD_REQ = 0x10,
  PACKET_CMD_RES,
  PACKET_CMD_IND,
  PACKET_CMD_DATA1,
  PACKET_CMD_DATA2,
  PACKET_CMD_DATA3,
  PACKET_CMD_DATA4,
  PACKET_CMD_ERROR_BLE = 0xFC,
  PACKET_CMD_ERROR_TO = 0xFD,
  PACKET_CMD_ERROR
}PACKET_CMD_VALUE;
typedef uint8_t 	UART_Data_T;
typedef uint16_t	UART_Length_T;
/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * FUNCTIONS - API
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
extern void FIOT_UART_CreateTask(void);
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
extern void MB_PACKET_Write(UART_Data_T iCmd, UART_Data_T iLen, UART_Data_T *iData);
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
extern void MB_PACKET_Write_Error();
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
extern void MB_PACKET_Write_Error_TO();
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
extern void MB_PACKET_Read(uint8_t *oData,uint8_t *oLen);
/********************************************************************************/
#ifdef __cplusplus
}
#endif											

#endif /* __MB_UART_H__ */