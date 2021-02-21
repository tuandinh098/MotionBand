/**
******************************************************************************
* @file			cbuffer.h	
* @author  	1.0 - Triet Luu - Fiot Co.,Ltd
*						1.1 - Vy Luu - Fiot Co.,Ltd
* @version 	1.1
* @date    	2016-11-14
* @brief
******************************************************************************
* @attention: This Circular Buffer is safe to use in IRQ with single Reader, 
*							single Writer. No need to disable any IRQ.
*							
*							CBuffer capacity = <size> - 1 
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CBUFFER_H
#define __CBUFFER_H
#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ------------------------------------------------------------------*/
#include "stdint.h" 

/* Public defines ------------------------------------------------------------*/
#define CB_MAX_SIZE	(0x00800000)

/* Public enumerate/structure ------------------------------------------------*/
typedef uint8_t		CBuffer_Data_T;
typedef uint16_t	CBuffer_Length_T;

typedef struct {
	CBuffer_Data_T 	  *Data;
	CBuffer_Length_T	Size;
	CBuffer_Length_T	Writer;
	CBuffer_Length_T	Reader;
	CBuffer_Length_T	Overflow;
} CBuffer_T;

/* Public macros -------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/
void							FIOT_CBuffer_Init(CBuffer_T *cb, void *buf, CBuffer_Length_T size);
void							FIOT_CBuffer_Clear(CBuffer_T *cb);
CBuffer_Length_T	FIOT_CBuffer_Read(CBuffer_T *cb, void *buf, CBuffer_Length_T nBytes);
CBuffer_Length_T	FIOT_CBuffer_Write(CBuffer_T *cb,const void *buf, CBuffer_Length_T nBytes);
CBuffer_Length_T	FIOT_CBuffer_DataCount(CBuffer_T *cb);
CBuffer_Length_T	FIOT_CBuffer_SizeCount(CBuffer_T *cb);

#endif // __CBUFFER_H
