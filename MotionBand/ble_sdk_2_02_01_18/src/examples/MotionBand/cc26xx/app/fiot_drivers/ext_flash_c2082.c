/**************** STFL-I based Serial Flash Memory Driver **********************

	Filename:	c2082.c
	Description: Library routines for the M25P05A, M25P10A, M25P20, M25P40, M25P80
				M25P16, M25P32, M25P64 Serial Flash Memories
********************************************************************************

	This source file provides library C code for M25P05A, M25P10A, M25P20,
	M25P40, M25P80, M25P16, M25P32, M25P64 serial flash devices.

	The following functions are available in this library(some memories may only support
	a subset of the list, refer to the specific product datasheet for details):

		Flash(WriteEnable, 0)						 to disable Write Protect in the Flash memory
		Flash(WriteDisable, 0)						to enable Write Protect in the Flash memory
		Flash(ReadDeviceIdentification, ParameterType)  to get the Device Identification from the device
		Flash(ReadManufacturerIdentification, ParameterType)(if available in the memory)	to get the manufacturer Identification from the device
		Flash(ReadStatusRegister, ParameterType)		to get the value in the Status Register from the device
		Flash(WriteStatusRegister, ParameterType)		to set the value in the Status Register from the device
		Flash(Read, ParameterType)					to read from the Flash device
		Flash(FastRead, ParameterType)				to read from the Flash device in a faster way
		Flash(PageProgram, ParameterType)			 to write an array of elements within one page
		Flash(SectorErase, ParameterType)			 to erase a whole sector
		Flash(BulkErase, ParameterType)				to erase the whole memory
		Flash(DeepPowerDown, 0)						to set the memory into the low power consumption mode
		Flash(ReleaseFromDeepPowerDown, 0)			to wake up the memory from the low power consumption mode
		Flash(Program, ParameterType)				 to program an array of elements
		Flash_ErrorStr()								to return an error description (define VERBOSE)

	Note that data Bytes will be referred to as elements throughout the document unless otherwise specified.

	For further information consult the related Datasheets and Application Note.
	The Application Note gives information about how to modify this code for
	a specific application.

	The hardware specific functions which may need to be modified by the user are:

		FlashWrite() used to write an element (uCPUBusType) to the Flash memory
		Flash_Read()  used to read an element (uCPUBusType) from the Flash memory
		Flash_TimeOut() to return after the function has timed out

	A list of the error conditions can be found at the end of the header file.

*******************************************************************************/

/******************************************************************************
	Includes
*******************************************************************************/

#include "ext_flash_c2082.h" /* Header file with global prototypes */
#include "MB_Board.h"
#include <ti/drivers/PIN/PINCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>

#ifdef TIME_H_EXISTS
  #include <time.h>
#endif

#ifdef SYNCHRONOUS_IO
#define WAIT_TILL_Instruction_EXECUTION_COMPLETE(x) Flash_TimeOut(0); while(IsFlashBusy()) \
	{ \
		if(Flash_OperationTimeOut == Flash_TimeOut(x)) return  Flash_OperationTimeOut; \
	};

#else
	// do nothing
#endif

/******************************************************************************
	Typedef
*******************************************************************************/
// Acceptable values for SPI master side configuration
typedef enum _SpiMasterConfigOptions
{
  enumNull                    = 0,   // do nothing
  enumEnableTransOnly         = 0x05,  // enable transfer
  enumEnableRecvOnly          = 0x0A,  // enable receive
  enumEnableTansRecv          = 0x0F,  // enable transfer & receive

  enumEnableTransOnly_SelectSlave     = 0x35,  // enable transfer and select slave
  enumEnableRecvOnly_SelectSlave      = 0x3A,  // enable receive and select slave
  enumEnableTansRecv_SelectSlave      = 0x3F,  // enable transfer & receive and select slave

  enumDisableTransOnly            = 0x04,  // disable transfer and deselect slave
  enumDisableRecvOnly             = 0x08,  // disable receive
  enumDisableTransRecv            = 0x0C,  // disable transfer & receive

  enumDisableTransOnly_DeSelectSlave  = 0x24,  // disable transfer and deselect slave
  enumDisableRecvOnly_DeSelectSlave   = 0x28,  // disable receive and deselect slave
  enumDisableTansRecv_DeSelectSlave   = 0x2C   // disable transfer & receive and deselect slave

}SpiMasterConfigOptions;

// char stream definition for
typedef struct _structCharStream
{
  uint8_t* pChar;               // buffer address that holds the streams
  uint32_t length;              // length of the stream in bytes
}
CharStream;

/******************************************************************************
	Constants
*******************************************************************************/
#define ptrNull       (void *)0 // a null pointer
#define Flash_CS      Board_SPI1_CSN
#define NULL_PTR      0x0

// mask bit definitions for SPI master side configuration
enum
{
  MaskBit_Trans               = 0x01,  // mask bit for Transfer enable/disable
  MaskBit_Recv                = 0x02,  // mask bit for Receive enable/disable
  MaskBit_Trans_Relevant      = 0x04,  // check whether MaskBit_Trans is necessary
  MaskBit_Recv_Relevant       = 0x08,  // check whether MaskBit_Recv is necessary

  MaskBit_SlaveSelect           = 0x10,  // mask bit for Slave Select/Deselect
  MaskBit_SelectSlave_Relevant  = 0x20,  // check whether MaskBit_SelectSlave is necessary
};

enum
{
  EECS            = 0x01,  // mask bit for Transfer enable/disable
  ACCELCS         = 0x02,  // mask bit for Receive enable/disable
  DISPLAYCS       = 0x03,  // check whether MaskBit_Trans is necessary
};

/******************************************************************************
	Local Variables
*******************************************************************************/
static PIN_Handle      Flash_Pin;
static SPI_Params      Flash_SPIParams;
static SPI_Handle      Flash_SPIHandle;
static SPI_Transaction Flash_SPITransaction;
/******************************************************************************
	Local Functions
*******************************************************************************/
static void ConfigureSpiMaster(SpiMasterConfigOptions opt);

static BOOL Serialize( const CharStream* char_stream_send,   // char stream to be sent to the memory(incl. instruction, address etc)
                       CharStream* char_stream_recv,    // char stream to be received from the memory
                       SpiMasterConfigOptions optBefore,  // Pre-Configurations on the SPI master side
                       SpiMasterConfigOptions optAfter    // Post-Configurations on the SPI master side
);

static void SpiInit();
/******************************************************************************
	Global variables: none
*******************************************************************************/

/******************************************************************************
	Functions: API
*******************************************************************************/
/*******************************************************************************
Function:	 ReturnType Flash( InstructionType insInstruction, ParameterType *fp )
Arguments:	insInstruction is an enum which contains all the available Instructions
	of the SW driver.
				fp is a (union) parameter struct for all Flash Instruction parameters
Return Value: The function returns the following conditions:

	Flash_AddressInvalid,
	Flash_MemoryOverflow,
	Flash_PageEraseFailed,
	Flash_PageNrInvalid,
	Flash_SectorNrInvalid,
	Flash_FunctionNotSupported,
	Flash_NoInformationAvailable,
	Flash_OperationOngoing,
	Flash_OperationTimeOut,
	Flash_ProgramFailed,
	Flash_SpecificError,
	Flash_Success,
	Flash_WrongType

Description:  This function is used to access all functions provided with the
	current Flash device.

Pseudo Code:
	Step 1: Select the right action using the insInstruction parameter
	Step 2: Execute the Flash memory Function
	Step 3: Return the Error Code
*******************************************************************************/
uint8_t Flag_SPI = 0;
ReturnType Flash( InstructionType insInstruction, ParameterType *fp ) {
	ReturnType  rRetVal;
	uint8_t  ucStatusRegister;
	uint16_t ucDeviceIdentification;
#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	uint8_t  ucManufacturerIdentification;
#endif
  if(Flag_SPI==0)
  {
    SpiOpen();
    Flag_SPI = 1;
  }
  else
  {
    SpiInit();
  }
	switch (insInstruction) {
		case WriteEnable:
			rRetVal = Flash_WriteEnable( );
			break;

		case WriteDisable:
			rRetVal = Flash_WriteDisable( );
			break;

		case ReadDeviceIdentification:
			rRetVal = Flash_ReadDeviceID(&ucDeviceIdentification);
			(*fp).ReadDeviceIdentification.ucDeviceIdentification = ucDeviceIdentification;
			break;

#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
		case ReadManufacturerIdentification:
			rRetVal = Flash_ReadManufacturerID(&ucManufacturerIdentification);
			(*fp).ReadManufacturerIdentification.ucManufacturerIdentification = ucManufacturerIdentification;
			break;
#endif

		case ReadStatusRegister:
			rRetVal = Flash_ReadStatusReg(&ucStatusRegister);
			(*fp).ReadStatusRegister.ucStatusRegister = ucStatusRegister;
			break;

		case WriteStatusRegister:
			ucStatusRegister = (*fp).WriteStatusRegister.ucStatusRegister;
			rRetVal = Flash_WriteStatusReg(ucStatusRegister);
			break;

		case Read:
			rRetVal = Flash_Read( (*fp).Read.udAddr,
								(*fp).Read.pArray,
								(*fp).Read.udNrOfElementsToRead
								);
			break;

		case FastRead:
			rRetVal = Flash_FastRead( (*fp).Read.udAddr,
									(*fp).Read.pArray,
									(*fp).Read.udNrOfElementsToRead
									);
			break;

		case PageProgram:
			rRetVal = Flash_Program( (*fp).PageProgram.udAddr,
								 (*fp).PageProgram.pArray,
								 (*fp).PageProgram.udNrOfElementsInArray
								);
			break;

		case SectorErase:
			rRetVal = Flash_SectorErase((*fp).SectorErase.ustSectorNr );
			break;

		case BulkErase:
			rRetVal = Flash_BulkErase( );
			break;


 #ifndef NO_DEEP_POWER_DOWN_SUPPORT
		case DeepPowerDown:
			rRetVal = Flash_DeepPowerDown( );
			break;

		case ReleaseFromDeepPowerDown:
			rRetVal = Flash_ReleaseFromDeepPowerDown( );
			break;
#endif

		case Program:
			rRetVal = Flash_Program( (*fp).Program.udAddr,
									(*fp).Program.pArray,
									(*fp).Program.udNrOfElementsInArray);
			break;

		default:
			rRetVal = Flash_FunctionNotSupported;
			break;

	} /* EndSwitch */
	return rRetVal;
} /* EndFunction Flash */

/*******************************************************************************
Function:	 Flash_WriteEnable( void )
Arguments:	void

Return Value:
	Flash_Success

Description:  This function sets the Write Enable Latch(WEL)
				by sending a WREN Instruction.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially
*******************************************************************************/
ReturnType  Flash_WriteEnable( void )
{
	CharStream char_stream_send;
	uint8_t  cWREN = SPI_FLASH_INS_WREN;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cWREN;

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTransOnly_DeSelectSlave
				);
	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_WriteDisable( void )
Arguments:	void

Return Value:
	Flash_Success

Description:  This function resets the Write Enable Latch(WEL)
				by sending a WRDI Instruction.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially
*******************************************************************************/
ReturnType  Flash_WriteDisable( void )
{
	CharStream char_stream_send;
	uint8_t  cWRDI = SPI_FLASH_INS_WRDI;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cWRDI;

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTransOnly_DeSelectSlave
				);
	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_ReadDeviceID( uint16_t *uwpDeviceIdentification)
Arguments:	uwpDeviceIdentificaiton, 16-bit buffer to hold the DeviceIdentification read from the
				memory, with memory type residing in the higher 8 bits, and
				memory capacity in the lower ones.

Return Value:
	Flash_Success
	Flash_WrongType(if USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE defined)

Description:  This function returns the Device Identification (memory type + memory capacity)
				by sending an SPI_FLASH_INS_RDID Instruction.
				After retrieving the Device Identificaiton, the routine checks if the device is
				an expected device(defined by EXPECTED_DEVICE). If not,
				Flash_WrongType is returned.

				If USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE is defined, the returned 16-bit
				word comprises memory type(higher 8 bits) and memory capacity
				(lower 8 bits).
				If USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE is NOT defined, only the lower
				8-bit byte of the returned 16-bit word is valid information,i.e. the
				Device Identification.
				For memories that have a capacity of more than 16Mb(inclusive),
				USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE is defined by default.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially
	Step 3: Device Identification is returned
*******************************************************************************/
ReturnType  Flash_ReadDeviceID( uint16_t *uwpDeviceIdentification)
{
#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  cRDID = SPI_FLASH_INS_RDID;
	uint8_t  pIdentification[3];

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar	= &cRDID;

	char_stream_recv.length  = 3;
	char_stream_recv.pChar	= &pIdentification[0];

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	// Step 3: Device Identification is returned ( memory type + memory capacity )
	*uwpDeviceIdentification = char_stream_recv.pChar[1];
	*uwpDeviceIdentification <<= 8;
	*uwpDeviceIdentification |= char_stream_recv.pChar[2];

	if(EXPECTED_DEVICE == *uwpDeviceIdentification)
		return Flash_Success;
	else
		return Flash_WrongType;

#else	// USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE not defined
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  pIns[4];
//	uint8_t  cRDID = SPI_FLASH_INS_RES;
	uint8_t  pIdentification;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 4;
	char_stream_send.pChar	= &pIns[0];
	pIns[0] = SPI_FLASH_INS_RES;
	pIns[1] = SPI_FLASH_INS_DUMMY;
	pIns[2] = SPI_FLASH_INS_DUMMY;
	pIns[3] = SPI_FLASH_INS_DUMMY;

	char_stream_recv.length  = 1;
	char_stream_recv.pChar	= &pIdentification;

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	// Step 3: Get the returned device Identification
	*uwpDeviceIdentification = *char_stream_recv.pChar;

	if(EXPECTED_DEVICE == *uwpDeviceIdentification)
		return Flash_Success;
	else
		return Flash_WrongType;
#endif
}

#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
/*******************************************************************************
Function:	 Flash_ReadManufacturerID( uint8_t *ucpManufactureIdentification)
Arguments:	ucpManufacturerIdentification, 8-bit buffer to hold the manufacturer identification
				being read from the memory

Return Value:
	Flash_WrongType: if any value other than MANUFACTURER_ST(0x20) is returned
	Flash_Success : if MANUFACTURER_ST(0x20) is correctly returned

Description:  This function returns the Manufacturer Identification(0x20) by sending an
				SPI_FLASH_INS_RDID Instruction.
				After retrieving the Manufacturer Identification, the routine checks if the device is
				an ST memory product. If not, Flash_WrongType is returned.

Note: The availability of this function should be checked in the appropriate datasheet
	for each memory.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially
	Step 3: get the Manufacturer Identification
*******************************************************************************/
ReturnType  Flash_ReadManufacturerID( uint8_t *ucpManufacturerIdentification)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  cRDID = SPI_FLASH_INS_RDID;
	uint8_t  pIdentification[3];

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar	= &cRDID;
	char_stream_recv.length  = 3;
	char_stream_recv.pChar	= &pIdentification[0];

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	// Step 3: get the Manufacturer Identification
	*ucpManufacturerIdentification = pIdentification[0];
	if(MANUFACTURER_ST == *ucpManufacturerIdentification)
	{
		return Flash_Success;
	}
	else
	{
		return Flash_WrongType;
	}
}
#endif // end of #ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE

/*******************************************************************************
Function:	 Flash_ReadStatusReg( uint8_t *ucpStatusRegister)
Arguments:	ucpStatusRegister, 8-bit buffer to hold the Status Register value read
				from the memory

Return Value:
	Flash_Success

Description:  This function reads the Status Register by sending an
				SPI_FLASH_INS_RDSR Instruction.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially, get the Status Register content

*******************************************************************************/
ReturnType  Flash_ReadStatusReg( uint8_t *ucpStatusRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  cRDSR = SPI_FLASH_INS_RDSR;


	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar	= &cRDSR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar	= ucpStatusRegister;

	// Step 2: Send the packet serially, get the Status Register content
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	return Flash_Success;
}


/*******************************************************************************
Function:	 Flash_WriteStatusReg( uint8_t ucStatusRegister)
Arguments:	ucStatusRegister, an 8-bit new value to be written to the Status Register

Return Value:
	Flash_Success

Description:  This function modifies the Status Register by sending an
				SPI_FLASH_INS_WRSR Instruction.
				The Write Status Register (WRSR) Instruction has no effect
				on b6, b5, b1(WEL) and b0(WIP) of the Status Register.b6 and b5 are
				always read as 0.

Pseudo Code:
	Step 1: Disable Write protection
	Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	Step 3: Send the packet serially
	Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  Flash_WriteStatusReg( uint8_t ucStatusRegister)
{
	CharStream char_stream_send;
	uint8_t  pIns_Val[2];

	// Step 1: Disable Write protection
	Flash_WriteEnable();

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 2;
	char_stream_send.pChar  = pIns_Val;
	pIns_Val[0] = SPI_FLASH_INS_WRSR;
	pIns_Val[1] = ucStatusRegister;

	// Step 3: Send the packet serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTransOnly_DeSelectSlave
				);
	// Step 4: Wait until the operation completes or a timeout occurs.
	WAIT_TILL_Instruction_EXECUTION_COMPLETE(1)
	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_Read( uint32_t udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead)
Arguments:	udAddr, start address to read from
				ucpElements, buffer to hold the elements to be returned
				udNrOfElementsToRead, number of elements to be returned, counted in bytes.

Return Value:
	Flash_AddressInvalid
	Flash_Success

Description:  This function reads the Flash memory by sending an
				SPI_FLASH_INS_READ Instruction.
				by design, the whole Flash memory space can be read with one READ Instruction
				by incrementing the start address and rolling to 0x0 automatically,
				that is, this function is across pages and sectors.

Pseudo Code:
	Step 1: Validate address input
	Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 3: Send the packet serially, and fill the buffer with the data being returned
*******************************************************************************/
ReturnType  Flash_Read( uAddrType udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  pIns_Addr[4];

	// Step 1: Validate address input
	if(!(udAddr <  FLASH_SIZE)) return Flash_AddressInvalid;

	// Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length	= 4;
	char_stream_send.pChar	= pIns_Addr;
	pIns_Addr[0]				= SPI_FLASH_INS_READ;
	pIns_Addr[1]				= udAddr>>16;
	pIns_Addr[2]				= udAddr>>8;
	pIns_Addr[3]				= udAddr;

	char_stream_recv.length	= udNrOfElementsToRead;
	char_stream_recv.pChar	= ucpElements;

	// Step 3: Send the packet serially, and fill the buffer with the data being returned
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_FastRead( uint32_t udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead)
Arguments:	udAddr, start address to read from
				ucpElements, buffer to hold the elements to be returned
				udNrOfElementsToRead, number of elements to be returned, counted in bytes.

Return Value:
	Flash_AddressInvalid
	Flash_Success

Description:  This function reads the Flash memory by sending an
				SPI_FLASH_INS_FAST_READ Instruction.
				by design, the whole Flash memory space can be read with one FAST_READ Instruction
				by incrementing the start address and rolling to 0x0 automatically,
				that is, this function is across pages and sectors.

Pseudo Code:
	Step 1: Validate address input
	Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 3: Send the packet serially, and fill the buffer with the data being returned
*******************************************************************************/
ReturnType  Flash_FastRead( uAddrType udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8_t  pIns_Addr[5];

	// Step 1: Validate address input
	if(!(udAddr <  FLASH_SIZE)) return Flash_AddressInvalid;

	// Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length	= 5;
	char_stream_send.pChar	= pIns_Addr;
	pIns_Addr[0]				= SPI_FLASH_INS_FAST_READ;
	pIns_Addr[1]				= udAddr>>16;
	pIns_Addr[2]				= udAddr>>8;
	pIns_Addr[3]				= udAddr;
	pIns_Addr[4]				= SPI_FLASH_INS_DUMMY;

	char_stream_recv.length	= udNrOfElementsToRead;
	char_stream_recv.pChar	= ucpElements;

	// Step 3: Send the packet serially, and fill the buffer with the data being returned
	Serialize(&char_stream_send,
				&char_stream_recv,
				enumEnableTansRecv_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_PageProgram( uint32_t udAddr, uint8_t *pArray, uint32_t udNrOfElementsInArray)
Arguments:	udAddr, start address to write to
				pArray, buffer to hold the elements to be programmed
				udNrOfElementsInArray, number of elements to be programmed, counted in bytes

Return Value:
	Flash_AddressInvalid
	Flash_OperationOngoing
	Flash_OperationTimeOut
	Flash_Success

Description:  This function writes a maximum of 256 bytes of data into the memory by sending an
				SPI_FLASH_INS_PP Instruction.
				by design, the PP Instruction is effective WITHIN ONE page,i.e. 0xXX00 - 0xXXff.
				when 0xXXff is reached, the address rolls over to 0xXX00 automatically.
Note:
				This function does not check whether the target memory area is in a Software
				Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
				Instruction will be ignored.
				The function assumes that the target memory area has previously been unprotected at both
				the hardware and software levels.
				To unprotect the memory, please call Flash_WriteStatusReg(uint8_t ucStatusRegister),
				and refer to the datasheet for the setup of a proper ucStatusRegister value.
Pseudo Code:
	Step 1: Validate address input
	Step 2: Check whether any previous Write, Program or Erase cycle is on going
	Step 3: Disable Write protection (the Flash memory will automatically enable it again after
			the execution of the Instruction)
	Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	Step 5: Send the packet (Instruction & address only) serially
	Step 6: Initialize the data (data to be programmed) packet to be sent serially
	Step 7: Send the packet (data to be programmed) serially
	Step 8: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  Flash_PageProgram( uAddrType udAddr, uint8_t *pArray , uint16_t udNrOfElementsInArray)
{
	CharStream char_stream_send;
	uint8_t  pIns_Addr[4];

	// Step 1: Validate address input
	if(!(udAddr <  FLASH_SIZE)) return Flash_AddressInvalid;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on-going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	Flash_WriteEnable();

	// Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	char_stream_send.length	= 4;
	char_stream_send.pChar	= pIns_Addr;
	pIns_Addr[0]				= SPI_FLASH_INS_PP;
	pIns_Addr[1]				= udAddr>>16;
	pIns_Addr[2]				= udAddr>>8;
	pIns_Addr[3]				= udAddr;

	// Step 5: Send the packet (Instruction & address only) serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumNull
				);

	// Step 6: Initialize the data (data to be programmed) packet to be sent serially
	char_stream_send.length	= udNrOfElementsInArray;
	char_stream_send.pChar	= pArray;

	// Step 7: Send the packet (data to be programmed) serially
	Serialize(&char_stream_send,
				ptrNull,
				enumNull,
				enumDisableTransOnly_DeSelectSlave
				);

	// Step 8: Wait until the operation completes or a timeout occurs.
	WAIT_TILL_Instruction_EXECUTION_COMPLETE(1)

	return Flash_Success;
}

/*******************************************************************************
Function:	 ReturnType Flash_SectorErase( uSectorType uscSectorNr )
Arguments:	uSectorType is the number of the Sector to be erased.

Return Values:
	Flash_SectorNrInvalid
	Flash_OperationOngoing
	Flash_OperationTimeOut
	Flash_Success

Description:  This function erases the Sector specified in uscSectorNr by sending an
				SPI_FLASH_INS_SE Instruction.
				The function checks that the sector number is within the valid range
				before issuing the erase Instruction. Once erase has completed the status
				Flash_Success is returned.
Note:
				This function does not check whether the target memory area is in a Software
				Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
				Instruction will be ignored.
				The function assumes that the target memory area has previously been unprotected at both
				the hardware and software levels.
				To unprotect the memory, please call Flash_WriteStatusReg(uint8_t ucStatusRegister),
				and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
	Step 1: Validate the sector number input
	Step 2: Check whether any previous Write, Program or Erase cycle is on going
	Step 3: Disable Write protection (the Flash memory will automatically enable it
			again after the execution of the Instruction)
	Step 4: Initialize the data (Instruction & address) packet to be sent serially
	Step 5: Send the packet (Instruction & address) serially
	Step 6: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  Flash_SectorErase( uSectorType uscSectorNr )
{
	CharStream char_stream_send;
	uint8_t  pIns_Addr[4];

	// Step 1: Validate the sector number input
	if(!(uscSectorNr < FLASH_SECTOR_COUNT)) return Flash_SectorNrInvalid;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	Flash_WriteEnable();

	// Step 4: Initialize the data (Instruction & address) packet to be sent serially
	char_stream_send.length	= 4;
	char_stream_send.pChar	= &pIns_Addr[0];
	pIns_Addr[0]				= SPI_FLASH_INS_SE;
	#ifdef FLASH_SMALLER_SECTOR_SIZE
	pIns_Addr[1]				= uscSectorNr>>1;
	pIns_Addr[2]				= uscSectorNr<<7;
	#else
	pIns_Addr[1]				= uscSectorNr;
	pIns_Addr[2]				= 0;
	#endif
	pIns_Addr[3]				= 0;

	// Step 5: Send the packet (Instruction & address) serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	// Step 6: Wait until the operation completes or a timeout occurs.
	WAIT_TILL_Instruction_EXECUTION_COMPLETE(3)

	return Flash_Success;
}
/*******************************************************************************
Function:	 ReturnType Flash_BulkErase( void )
Arguments:	none

Return Values:
	Flash_OperationOngoing
	Flash_OperationTimeOut
	Flash_Success

Description:  This function erases the whole Flash memory by sending an
				SPI_FLASH_INS_BE Instruction.
Note:
				This function does not check whether the target memory area (or part of it)
				is in a Software Protection Mode(SPM) or Hardware Protection Mode(HPM),
				in which case the PP Instruction will be ignored.
				The function assumes that the target memory area has previously been unprotected at both
				the hardware and software levels.
				To unprotect the memory, please call Flash_WriteStatusReg(uint8_t ucStatusRegister),
				and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
	Step 1: Check whether any previous Write, Program or Erase cycle is on going
	Step 2: Disable the Write protection (the Flash memory will automatically enable it
			again after the execution of the Instruction)
	Step 3: Initialize the data (Instruction & address) packet to be sent serially
	Step 4: Send the packet (Instruction & address) serially
	Step 5: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  Flash_BulkErase( void )
{
	static
	CharStream char_stream_send;

	uint8_t  cBE = SPI_FLASH_INS_BE;

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Disable Write protection
	Flash_WriteEnable();

	// Step 3: Initialize the data(Instruction & address) packet to be sent serially
	char_stream_send.length	= 1;
	char_stream_send.pChar	= &cBE;

	// Step 4: Send the packet(Instruction & address) serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTansRecv_DeSelectSlave
				);

	// Step 5: Wait until the operation completes or a timeout occurs.
	WAIT_TILL_Instruction_EXECUTION_COMPLETE(BE_TIMEOUT)

	return Flash_Success;
}

#ifndef NO_DEEP_POWER_DOWN_SUPPORT
/*******************************************************************************
Function:	 Flash_DeepPowerDown( void )
Arguments:	void

Return Value:
	Flash_OperationOngoing
	Flash_Success

Description:  This function puts the device in the lowest consumption
				mode (the Deep Power-down mode) by sending an SPI_FLASH_INS_DP.
				After calling this routine, the Flash memory will not respond to any
				subsequent Instruction except for the RDP Instruction.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Check whether any previous Write, Program or Erase cycle is on going
	Step 3: Send the packet serially
*******************************************************************************/
ReturnType  Flash_DeepPowerDown( void )
{
	CharStream char_stream_send;
	uint8_t  cDP = SPI_FLASH_INS_DP;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cDP;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Send the packet serially
	Serialize(&char_stream_send,
						ptrNull,
						enumEnableTransOnly_SelectSlave,
						enumDisableTransOnly_DeSelectSlave);

	return Flash_Success;
}

/*******************************************************************************
Function:	 Flash_ReleaseFromDeepPowerDown( void )
Arguments:	void

Return Value:
	Flash_Success

Description:  This function takes the device out of the Deep Power-down
				mode by sending an SPI_FLASH_INS_RES.

Pseudo Code:
	Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	Step 2: Send the packet serially
*******************************************************************************/
ReturnType  Flash_ReleaseFromDeepPowerDown( void )
{
	CharStream char_stream_send;
	uint8_t  cRES = SPI_FLASH_INS_RES;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cRES;

	// Step 2: Send the packet serially
	Serialize(&char_stream_send,
				ptrNull,
				enumEnableTransOnly_SelectSlave,
				enumDisableTransOnly_DeSelectSlave
				);
	return Flash_Success;
}
#endif //	end of #ifndef NO_DEEP_POWER_DOWN_SUPPORT

/*******************************************************************************
Function:	 Flash_Program( uint32_t udAddr, uint8_t *pArray, uint32_t udNrOfElementsInArray )
Arguments:	udAddr, start address to program
				pArray, address of the  buffer that holds the elements to be programmed
				udNrOfElementsInArray, number of elements to be programmed, counted in bytes

Return Value:
	Flash_AddressInvalid
	Flash_MemoryOverflow
	Flash_OperationTimeOut
	Flash_Success

Description:  This function programs a chunk of data into the memory at one go.
				If the start address and the available space are checked successfully,
				this function programs data from the buffer(pArray) to the memory sequentially by
				invoking Flash_PageProgram(). This function automatically handles page boundary
				crossing, if any.
				Like Flash_PageProgram(), this function assumes that the memory to be programmed
				has been previously erased or that bits are only changed from 1 to 0.
Note:
				This function does not check whether the target memory area is in a Software
				Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
				Instruction will be ignored.
				The function assumes that the target memory area has previously been unprotected at both
				the hardware and software levels.
				To unprotect the memory, please call Flash_WriteStatusReg(uint8_t ucStatusRegister),
				and refer to the datasheet for a proper ucStatusRegister value.

Pseudo Code:
	Step 1: Validate address input
	Step 2: Check memory space available on the whole memory
	Step 3: calculte memory space available within the page containing the start address(udAddr)
	Step 3-1: if the page boundary is crossed, invoke Flash_PageProgram() repeatedly
	Step 3-2: if the page boundary is not crossed, invoke Flash_PageProgram() once only
*******************************************************************************/
ReturnType  Flash_Program( uint32_t udAddr, uint8_t *pArray, uint32_t udNrOfElementsInArray )
{
	uint16_t ucMargin;
	uint16_t ucPageCount, ucRemainder;
	ReturnType typeReturn;

	// Step 1: Validate address input
	if(!(udAddr <  FLASH_SIZE)) return Flash_AddressInvalid;

	// Step 2: Check memory space available on the whole memory
	if(udAddr + udNrOfElementsInArray > FLASH_SIZE) return Flash_MemoryOverflow;

	// Step 3: calculte memory space available within the page containing the start address(udAddr)
	// 2013-04-08
	// ANHTRAN
	// 1024 pages (256 bytes each).
	// 4 sectors (512 Kbits, 65536 bytes each)
	ucMargin = (uint8_t)(~udAddr) + 1;

	// Step 3-1: if the page boundary is crossed, invoke FlashPageWrite() repeatedly
	if(udNrOfElementsInArray > ucMargin)
	{
		typeReturn = Flash_PageProgram(udAddr, pArray, ucMargin);
		if(Flash_Success != typeReturn) return typeReturn;				// return immediately if Not successful

		udNrOfElementsInArray -= ucMargin;								// re-calculate the number of elements
		pArray += ucMargin;											 // modify the pointer to the buffer
		udAddr += ucMargin;											 // modify the start address in the memory
		ucPageCount = udNrOfElementsInArray / FLASH_WRITE_BUFFER_SIZE;  // calculate the number of pages to be programmed
		ucRemainder = udNrOfElementsInArray % FLASH_WRITE_BUFFER_SIZE;	// calculate the remainder after filling up one or more whole pages
		while(ucPageCount--)
		{
			typeReturn = Flash_PageProgram(udAddr, pArray, FLASH_WRITE_BUFFER_SIZE);
			if(Flash_Success != typeReturn) return typeReturn;			// return immediately if Not successful
			pArray += FLASH_WRITE_BUFFER_SIZE;
			udAddr += FLASH_WRITE_BUFFER_SIZE;
		};
		return Flash_PageProgram(udAddr, pArray, ucRemainder);
	}
	// Step 3-2: if the page boundary is not crossed, invoke FlashPageWrite() once only
	else
	{
		return Flash_PageProgram(udAddr, pArray, udNrOfElementsInArray);
	}
}

/*******************************************************************************
Function:	 IsFlashBusy( )
Arguments:	none

Return Value:
	TRUE
	FALSE

Description:  This function checks the Write In Progress (WIP) bit to determine whether
				the Flash memory is busy with a Write, Program or Erase cycle.

Pseudo Code:
	Step 1: Read the Status Register.
	Step 2: Check the WIP bit.
*******************************************************************************/
BOOL IsFlashBusy(void)
{
	uint8_t ucSR;

	// Step 1: Read the Status Register.
	Flash_ReadStatusReg(&ucSR);

	// Step 2: Check the WIP bit.
	if(ucSR & SPI_FLASH_WIP)
		return TRUE;
	else
		return FALSE;
}

#ifdef VERBOSE
/*******************************************************************************
Function:	 Flash_ErrorStr( ReturnType rErrNum );
Arguments:	rErrNum is the error number returned from other Flash memory Routines

Return Value: A pointer to a string with the error message

Description:  This function is used to generate a text string describing the
	error from the Flash memory. Call with the return value from other Flash memory routines.

Pseudo Code:
	Step 1: Return the correct string.
*******************************************************************************/
char *Flash_ErrorStr( ReturnType rErrNum )
{
	switch(rErrNum)
	{
	case Flash_AddressInvalid:
		return "Flash - Address is out of Range";
	case Flash_MemoryOverflow:
		return "Flash - Memory Overflows";
	case Flash_PageEraseFailed:
		return "Flash - Page Erase failed";
	case Flash_PageNrInvalid:
		return "Flash - Page Number is out of Range";
	case Flash_SectorNrInvalid:
		return "Flash - Sector Number is out of Range";
  case Flash_FunctionNotSupported:
		return "Flash - Function not supported";
	case Flash_NoInformationAvailable:
		return "Flash - No Additional Information Available";
	case Flash_OperationOngoing:
		return "Flash - Operation ongoing";
	case Flash_OperationTimeOut:
		return "Flash - Operation TimeOut";
	case Flash_ProgramFailed:
		return "Flash - Program failed";
	case Flash_Success:
		return "Flash - Success";
	case Flash_WrongType:
		return "Flash - Wrong Type";
	default:
		return "Flash - Undefined Error Value";
	} /* EndSwitch */
} /* EndFunction FlashErrorString */
#endif /* VERBOSE Definition */


/*******************************************************************************
Function:	 Flash_TimeOut(uint32_t udSeconds)
Arguments:	udSeconds holds the number of seconds before TimeOut occurs

Return Value:
	Flash_OperationTimeOut
	Flash_OperationOngoing

Example:	Flash_TimeOut(0)  // Initializes the Timer

			While(1) {
				...
				If (Flash_TimeOut(5) == Flash_OperationTimeOut) break;
				// The loop is executed for 5 Seconds before the operation is aborted
			} EndWhile

*******************************************************************************/
#ifdef TIME_H_EXISTS
/*-----------------------------------------------------------------------------
Description:	This function provides a timeout for Flash polling actions or
	other operations which would otherwise never return.
	The Routine uses the function clock() inside ANSI C library "time.h".
-----------------------------------------------------------------------------*/
ReturnType Flash_TimeOut(uint32_t udSeconds){
	static clock_t clkReset,clkCount;

	if (udSeconds == 0) { /* Set Timeout to 0 */
		clkReset=clock();
	} /* EndIf */

	clkCount = clock() - clkReset;

	if (clkCount<(CLOCKS_PER_SEC*(clock_t)udSeconds))
		return Flash_OperationOngoing;
	else
		return Flash_OperationTimeOut;
}/* EndFunction Flash_TimeOut */

#else
/*-----------------------------------------------------------------------------
Description:	This function provides a timeout for Flash polling actions or
	other operations which would otherwise never return.
	The Routine uses COUNT_FOR_A_SECOND which is considered to be a loop that
	counts for one second. It needs to be adapted to the target Hardware.
-----------------------------------------------------------------------------*/
ReturnType Flash_TimeOut(uint32_t udSeconds) {

	static uint32_t udCounter = 0;
	if (udSeconds == 0) { /* Set Timeout to 0 */
	 udCounter = 0;
	} /* EndIf */

	if (udCounter == (udSeconds * COUNT_FOR_A_SECOND)) {
		udCounter = 0;
		return Flash_OperationTimeOut;
	} else {
		udCounter++;
		return Flash_OperationOngoing;
	} /* Endif */

} /* EndFunction Flash_TimeOut */
#endif /* TIME_H_EXISTS */


/******************************************************************************
	Local Functions
*******************************************************************************/

/*******************************************************************************
     SPI init
Function:     SpiInit()
Arguments:    There is no argument for this function
Return Values:There is no return value for this function.
Description:  This function has to be called at the beginnning to configure the
			  port.
*******************************************************************************/
void SpiOpen()
{
    SPI_Params_init(&Flash_SPIParams);
    Flash_SPIParams.frameFormat = SPI_POL1_PHA1;
    Flash_SPIHandle = SPI_open(Board_SPI1, &Flash_SPIParams);
}

static void SpiInit()
{
    //SPI_Params_init(&Flash_SPIParams);
    //Flash_SPIParams.frameFormat = SPI_POL1_PHA1;
   // Flash_SPIHandle = SPI_open(Board_SPI1, &Flash_SPIParams);

}
/*******************************************************************************
Function:	 ConfigureSpiMaster(SpiMasterConfigOptions opt)
Arguments:	opt configuration options, all acceptable values are enumerated in
			  SpiMasterConfigOptions, which is a typedefed enum.
Return Values:There is no return value for this function.
Description:  This function can be used to properly configure the SPI master
			  before and after the transfer/receive operation
Pseudo Code:
   Step 1  : perform or skip select/deselect slave
   Step 2  : perform or skip enable/disable transfer
   Step 3  : perform or skip enable/disable receive
*******************************************************************************/

static void ConfigureSpiMaster(SpiMasterConfigOptions opt)
{
	if(enumNull == opt) return;
	if (opt & MaskBit_SelectSlave_Relevant)
	{
		if (opt & MaskBit_SlaveSelect)
			PIN_setOutputValue(Flash_Pin, Flash_CS, false);
		else
			PIN_setOutputValue(Flash_Pin, Flash_CS, true);
	}
}

/*******************************************************************************
Function:	 Serialize(const CharStream* char_stream_send,
			  CharStream* char_stream_recv,
			  SpiMasterConfigOptions optBefore,
			  SpiMasterConfigOptions optAfter
			  )
Arguments:	char_stream_send, the char stream to be sent from the SPI master to
			  the Flash memory, usually contains instruction, address, and data to be
			  programmed.
			  char_stream_recv, the char stream to be received from the Flash memory
			  to the SPI master, usually contains data to be read from the memory.
			  optBefore, configurations of the SPI master before any transfer/receive
			  optAfter, configurations of the SPI after any transfer/receive
Return Values:TRUE
Description:  This function can be used to encapsulate a complete transfer/receive
			  operation
Pseudo Code:
   Step 1  : perform pre-transfer configuration
   Step 2  : perform transfer/ receive
	Step 2-1: transfer ...
		(a typical process, it may vary with the specific CPU)
		Step 2-1-1:  check until the SPI master is available
		Step 2-1-2:  send the byte stream cycle after cycle. it usually involves:
					 a) checking until the transfer-data-register is ready
					 b) filling the register with a new byte
	Step 2-2: receive ...
		(a typical process, it may vary with the specific CPU)
		Step 2-2-1:  Execute ONE pre-read cycle to clear the receive-data-register.
		Step 2-2-2:  receive the byte stream cycle after cycle. it usually involves:
					 a) triggering a dummy cycle
					 b) checking until the transfer-data-register is ready(full)
					 c) reading the transfer-data-register
   Step 3  : perform post-transfer configuration
*******************************************************************************/
static BOOL Serialize(const CharStream* char_stream_send,
                      CharStream* char_stream_recv,
                      SpiMasterConfigOptions optBefore,
                      SpiMasterConfigOptions optAfter)
{
  uint8_t rxBuff[128] = { 0x00 };
//  uint8_t txBuff[100] = { 0x00 };

//  memcpy(txBuff, char_stream_send->pChar, char_stream_send->length);
  Flash_SPITransaction.arg = NULL;
  if(char_stream_send != NULL && char_stream_recv != NULL)
  {
    Flash_SPITransaction.count = char_stream_send->length + char_stream_recv->length;
  }
  else if(char_stream_send == NULL)
  {
    Flash_SPITransaction.count = char_stream_recv->length;
  }
  else
  {
    Flash_SPITransaction.count = char_stream_send->length;
  }

	Flash_SPITransaction.txBuf = char_stream_send->pChar;//txBuff;
  Flash_SPITransaction.rxBuf = rxBuff;

  // Step 1  : perform pre-transfer configuration
  ConfigureSpiMaster(optBefore);

  SPI_transfer(Flash_SPIHandle, &Flash_SPITransaction);

  // Step 3  : perform post-transfer configuration
  ConfigureSpiMaster(optAfter);

  if(char_stream_recv != NULL)
  {
    memcpy(char_stream_recv->pChar, &rxBuff[char_stream_send->length], char_stream_recv->length);
  }

	return TRUE;
}

/*******************************************************************************
 End of ext_flash.c
*******************************************************************************/
