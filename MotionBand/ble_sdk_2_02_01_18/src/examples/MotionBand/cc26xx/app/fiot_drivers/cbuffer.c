/**
******************************************************************************
* @file		cbuffer.c
* @author  	Triet Luu - Fiot Co.,Ltd
* @version 	1.0
* @date    	2015-10-10
* @brief
******************************************************************************
* @attention
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "cbuffer.h"

/* Private defines -----------------------------------------------------------*/
#define USE_EX_LOAD_STORE	(1)

/* Private enumerate/structure -----------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function definitions ------------------------------------------------------*/
/*
Initializes the CB structure with the given buffer and size
*/
void FIOT_CBuffer_Init(CBuffer_T *cb, void *buf, CBuffer_Length_T size)
{
	cb->Data = (CBuffer_Data_T *)buf;
	cb->Size = size;
	cb->Reader = 0;
	cb->Writer = 0;
	cb->Overflow = 0;
}

/*
Reset the CB structure
*/
void FIOT_CBuffer_Clear(CBuffer_T *cb)
{
	cb->Reader = 0;
	cb->Writer = 0;
	cb->Overflow = 0;
}

/*
Return number of unread bytes in the CB.
*/
CBuffer_Length_T FIOT_CBuffer_DataCount(CBuffer_T *cb)
{
	CBuffer_Length_T tmpReader, tmpWriter;
	tmpReader = cb->Reader;
	tmpWriter = cb->Writer;

	if (tmpReader <= tmpWriter)
		return (tmpWriter - tmpReader);
	else
		return (tmpWriter + cb->Size - tmpReader);
}

/*
Return number of free bytes in the CB.
*/
CBuffer_Length_T FIOT_CBuffer_SizeCount(CBuffer_T *cb)
{
	return (cb->Size - 1 - FIOT_CBuffer_DataCount(cb));
}

/*
Read upto <nBytes> from the CB. Actual number of read bytes is returned.
*/
CBuffer_Length_T FIOT_CBuffer_Read(CBuffer_T *cb, void *buf, CBuffer_Length_T nBytes)
{
	CBuffer_Length_T i;

	for (i = 0; i < nBytes; i++)
	{
		// See if any data is available
		if (cb->Reader != cb->Writer)
		{
			// Grab a byte from the internal buffer
			*((CBuffer_Data_T *)buf) = cb->Data[cb->Reader]; 
			buf = (CBuffer_Data_T *)buf + 1;
			
			// Check for wrap-around
			if (++cb->Reader == cb->Size)
				cb->Reader = 0;
		}
		else 
		{
			break;
		}
	}

	return i; // Number of bytes read
}

/*
Write upto <nBytes> to the CB. Actual number of written byte is returned.
*/
CBuffer_Length_T FIOT_CBuffer_Write(CBuffer_T *cb, const void *buf, CBuffer_Length_T nBytes)
{
	CBuffer_Length_T i;

	for (i = 0; i < nBytes; i++) 
	{
		// First check to see if there is space in the buffer
		if ((cb->Writer + 1 == cb->Reader) || ((cb->Writer + 1 == cb->Size) && (cb->Reader == 0)))
		{
			cb->Overflow += (nBytes - i);
			break;
		}
		else 
		{
			// Write a byte to the internal buffer
			cb->Data[cb->Writer] = *((CBuffer_Data_T *)buf);
			buf = (CBuffer_Data_T *)buf + 1;

			// Check for wrap-around
			if (++cb->Writer == cb->Size)
				cb->Writer = 0;
		}
	}

	return i; // Number of bytes write
}

/*****************************END OF FILE****/
