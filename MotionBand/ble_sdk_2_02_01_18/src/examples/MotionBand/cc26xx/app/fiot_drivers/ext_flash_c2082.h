/***************** Header File for STFL-I based Serial Flash Memory Driver *****

	Filename:	c2082.h
	Description: Header file for c2082.c
		Also consult the C file for more details.

		Please note that some necessary changes are made in favor of the
		SPI-specific communication property which slightly differs from
		the STFL-I Specification designed for parallel NOR Flash memories. The major
		differences from the SPECIFICATION OF THE STFL-I SOFTWARE DRIVER INTERFACE
	(Specification-STFL-I-V2-1a) are the following:
		- Flash Configuration Selection is not used.
		- BASE_ADDR is not used.
		- InstructionType enumuerations are re-formulated to use SPI Flash instructions.
		- CONFIGURATION CONSTANTS are fixed, with #define ins(A) not used.
		...
*******************************************************************************/


/*************** User Change Area *******************************************

	The purpose of this section is to show how the SW Drivers can be customized
	according to the requirements of the hardware and Flash memory configurations.
	It is possible to choose the Flash memory start address, the CPU Bit depth, the number of Flash
	chips, the hardware configuration and performance data (TimeOut Info).

	The options are listed and explained below:

	********* Data Types *********
	The source code defines hardware independent datatypes assuming that the
	compiler implements the numerical types as

	unsigned char	8 bits (defined as uint8_t)
	char		8 bits (defined as int8_t)
	unsigned int	16 bits (defined as uint16_t)
	int			16 bits (defined as int16_t)
	unsigned long	32 bits (defined as uint32_t)
	long		32 bits (defined as int32_t)

	In case the compiler does not support the currently used numerical types,
	they can be easily changed just once here in the user area of the headerfile.
	The data types are consequently referenced in the source code as (u)int8_t,
	(u)int16_t and (u)int32_t. No other data types like 'CHAR','SHORT','INT','LONG'
	are directly used in the code.


	********* Flash Type *********
	This driver supports the following Serial Flash memory Types

	M25P05A		512Kb Serial Flash Memory		#define USE_M25P05A
	M25P10A		1Mb Serial Flash Memory			#define USE_M25P10A
	M25P20	2Mb Serial Flash Memory			#define USE_M25P20
	M25P40	4Mb Serial Flash Memory			#define USE_M25P40
	M25P80	8Mb Serial Flash Memory			#define USE_M25P80
	M25P16	16Mb Serial Flash Memory			#define USE_M25P16
	M25P32	32Mb Serial Flash Memory			#define USE_M25P32
	M25P64	64Mb Serial Flash Memory			#define USE_M25P64


	********* Flash and Board Configuration *********
	The driver also supports different configurations of the Flash chips
	on the board. In each configuration a new data Type called
	'uCPUBusType' is defined to match the current CPU data bus width.
	This data type is then used for all accesses to the memory.

	Because SPI interface communications are controlled by the
	SPI master, which, in turn, is accessed by the CPU as an 8-bit data
	buffer, the configuration is fixed for all cases.

	********* TimeOut *********
	There are timeouts implemented in the loops of the code, in order
	to enable a timeout detection for operations that would otherwise never terminate.
	There are two possibilities:

	1) The ANSI Library functions declared in 'time.h' exist

		If the current compiler supports 'time.h' the define statement
		TIME_H_EXISTS should be activated. This makes sure that
		the performance of the current evaluation HW does not change
		the timeout settings.

	2) or they are not available (COUNT_FOR_A_SECOND)

		If the current compiler does not support 'time.h', the define
		statement cannot be used. In this case the COUNT_FOR_A_SECOND
		value has to be defined so as to create a one-second delay.
		For example, if 100000 repetitions of a loop are
		needed to give a time delay of one second, then
		COUNT_FOR_A_SECOND should have the value 100000.

		Note: This delay is HW (Performance) dependent and therefore needs
		to be updated with every new HW.

		This driver has been tested with a certain configuration and other
		target platforms may have other performance data, therefore, the
		value may have to be changed.

		It is up to the user to implement this value to prevent the code
		from timing out too early and allow correct completion of the device
		operations.


	********* Additional Routines *********
	The drivers also provide a subroutine which displays the full
	error message instead of just an error number.

	The define statement VERBOSE activates additional Routines.
	Currently it activates the Flash_ErrorStr() function

	No further changes should be necessary.

*****************************************************************************/

#ifndef __EXT_FLASH__H
#define __EXT_FLASH__H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#define DRIVER_VERSION_MAJOR 2
#define DRIVER_VERSION_MINOR 0

/* With SYNCHRONOUS_IO defined, each function that sends an Instruction(e.g. PE)
	shall not return until the Flash memory finishes executing the Instruction
	or a pre-set timeout limit is reached. the pre-set timeout value is in
	accordance with the datasheet of each memory.

	To achieve Send-n-Forget feature, comment out this #define*/
#define SYNCHRONOUS_IO

#define USE_M25P80

//#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
/* Possible Values: USE_M25P05A
		USE_M25P10A
		USE_M25P20
		USE_M25P40
		USE_M25P80
		USE_M25P16
		USE_M25P32
		USE_M25P64
*/

/*#define TIME_H_EXISTS*/	/* set this macro if C-library "time.h" is supported */
/* Possible Values: TIME_H_EXISTS
			- no define - TIME_H_EXISTS */

#ifndef TIME_H_EXISTS
	#define COUNT_FOR_A_SECOND 0xFFFFFF	/* Timer Usage */
#endif

#define VERBOSE /* Activates additional Routines */
/* Currently the Error String Definition */

/********************** End of User Change Area *****************************/

/*******************************************************************************
Device Constants
*******************************************************************************/

#define MANUFACTURER_ST (0x20)	/* ST Manufacturer Identification is 0x20 */
#define MEMORYTYPE_M25Pxx (0x20)	/* JEDEC Memory Type for M25Pxx Identification is 0x20 */
#define ANY_ADDR (0x0)		/* Any address offset within the Flash Memory will do */

#ifdef USE_M25P05A				/* The M25P05A device */
	#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/
	#ifdef	USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	#define EXPECTED_DEVICE (0x2010) /* Preferred Device Identification: please refer to the datasheet */
	#else
	#define EXPECTED_DEVICE (0x05)	/* Device Identification: please refer to the datasheet */
	#endif
	#define FLASH_SIZE (0x010000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x0100)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x02)	/* Total device size in Sectors */
	#define FLASH_SMALLER_SECTOR_SIZE	/* Sector Size	= 256Kb*/
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x6)			/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P05A */

#ifdef USE_M25P10A				/* The M25P10A device */
	#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/
	#ifdef	USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	#define EXPECTED_DEVICE (0x2011) /* Preferred Device Identification: please refer to the datasheet */
	#else
	#define EXPECTED_DEVICE (0x10)	/* Device Identification: please refer to the datasheet */
	#endif
	#define FLASH_SIZE (0x020000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x0200)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x04)	/* Total device size in Sectors */
	#define FLASH_SMALLER_SECTOR_SIZE	/* Sector Size	= 256Kb*/
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x6)			/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P10A */

#ifdef USE_M25P20			/* The M25P20 device */
	#define EXPECTED_DEVICE (0x11)	/* Device Identification for the M25P20 */
	#define FLASH_SIZE (0x040000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x0400)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x04)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x6)			/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P20 */

#ifdef USE_M25P40			/* The M25P40 device */
	#define EXPECTED_DEVICE (0x12)	/* Device Identification for the M25P40 */
	#define FLASH_SIZE (0x080000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x0800)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x08)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x03)		/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P40 */

#ifdef USE_M25P80			/* The M25P80 device */
	#define EXPECTED_DEVICE (0x13)	/* Device Identification for the M25P80 */
	#define FLASH_SIZE (0x0100000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x01000)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x10)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x14)		/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P80 */

#ifdef USE_M25P16			/* The M25P16 device */
	#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/
	#ifdef	USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	#define EXPECTED_DEVICE (0x2015) /* Device Identification for the USE_M25P16 */
	#else
	#define EXPECTED_DEVICE (0x14)	/* Device Identification for the USE_M25P16 */
	#endif
	#define FLASH_SIZE (0x0200000)	/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x02000)	/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x20)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100 /* Write Buffer = 256 bytes */
	#define FLASH_MWA 1			/* Minimum Write Access */
	#define BE_TIMEOUT (0x46)		/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P16 */

#ifdef USE_M25P32			/* The USE_M25P32 device */
	#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/
	#ifdef	USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	#define EXPECTED_DEVICE (0x2016)	/* Device Identification for the USE_M25P32 */
	#else
	#define EXPECTED_DEVICE (0x15)		/* Device Identification for the USE_M25P32 */
	#endif
	#define FLASH_SIZE (0x0400000)		/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x04000)		/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x40)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100	/* Write Buffer = 256 bytes */
	#define FLASH_MWA 1				/* Minimum Write Access */
	#define BE_TIMEOUT (0x80)		/* Timeout in seconds suggested for Bulk Erase Operation*/
#endif /* USE_M25P32 */


#ifdef USE_M25P64			/* The USE_M25P64 device */
	#define USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE /* undefine this macro to read One-Byte Signature*/
	#ifdef	USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	#define EXPECTED_DEVICE (0x2017)	/* Device Identification for the USE_M25P64 */
	#else
	#define EXPECTED_DEVICE (0x16)		/* Device Identification for the USE_M25P64 */
	#endif
	#define FLASH_SIZE (0x0800000)		/* Total device size in Bytes */
	#define FLASH_PAGE_COUNT (0x08000)		/* Total device size in Pages */
	#define FLASH_SECTOR_COUNT (0x80)	/* Total device size in Sectors */
	#define FLASH_WRITE_BUFFER_SIZE 0x100	/* Write Buffer = 256 bytes */
	#define FLASH_MWA 1				/* Minimum Write Access */
	#define BE_TIMEOUT (0x160)			/* Timeout in seconds suggested for Bulk Erase Operation*/
	#define NO_DEEP_POWER_DOWN_SUPPORT		/* No support for Deep Power-down feature*/
#endif /* USE_M25P64 */
/*******************************************************************************
	DERIVED DATATYPES
*******************************************************************************/
/******** InstructionsCode ********/
#define SPI_FLASH_INS_DUMMY 0xAA		// dummy byte
enum
{
	//Instruction set
	SPI_FLASH_INS_WREN	= 0x06,	// write enable
	SPI_FLASH_INS_WRDI	= 0x04,	// write disable
	SPI_FLASH_INS_RDSR	= 0x05,	// read status register
	SPI_FLASH_INS_WRSR	= 0x01,	// write status register
	SPI_FLASH_INS_READ	= 0x03,	// read data bytes
	SPI_FLASH_INS_FAST_READ	= 0x0B,	// read data bytes at higher speed
	SPI_FLASH_INS_PP		= 0x02,	// page program
	SPI_FLASH_INS_SE		= 0xD8,	// sector erase

 	#ifndef NO_DEEP_POWER_DOWN_SUPPORT
		SPI_FLASH_INS_RES	= 0xAB,	// release from deep power-down
		SPI_FLASH_INS_DP		= 0xB9,	// deep power-down
	#endif
	#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
		SPI_FLASH_INS_RDID	= 0x9F,	// read identification
	#endif

	SPI_FLASH_INS_BE		= 0xC7	// bulk erase
};


/******** InstructionsType ********/

typedef enum {
	WriteEnable,
	WriteDisable,
	ReadDeviceIdentification,
	#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	ReadManufacturerIdentification,
	#endif
	ReadStatusRegister,
	WriteStatusRegister,
	Read,
	FastRead,
	PageProgram,
	SectorErase,
	BulkErase,

	#ifndef NO_DEEP_POWER_DOWN_SUPPORT
	DeepPowerDown,
	ReleaseFromDeepPowerDown,
	#endif

	Program
} InstructionType;

/******** ReturnType ********/

typedef enum {
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
	Flash_WrongType,
	Flash_Success,
	Flash_Still_Working
} ReturnType;

/******** SectorType ********/

typedef uint8_t uSectorType;

/******** PageType ********/

typedef uint16_t uPageType;

/******** AddrType ********/

typedef uint32_t uAddrType;

/******** ParameterType ********/

typedef union {

	/**** WriteEnable has no parameters ****/

	/**** WriteDisable has no parameters ****/

	/**** ReadDeviceIdentification Parameters ****/
	struct {
		uint16_t ucDeviceIdentification;
	} ReadDeviceIdentification;

	/**** ReadManufacturerIdentification Parameters ****/
	struct {
		uint8_t ucManufacturerIdentification;
	} ReadManufacturerIdentification;

	/**** ReadStatusRegister Parameters ****/
	struct {
		uint8_t ucStatusRegister;
	} ReadStatusRegister;

	/**** WriteStatusRegister Parameters ****/
	struct {
		uint8_t ucStatusRegister;
	} WriteStatusRegister;

	/**** Read Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsToRead;
		void *pArray;
	} Read;

	/**** FastRead Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsToRead;
		void *pArray;
	} FastRead;

	/**** PageWrite Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsInArray;
		void *pArray;
	} PageWrite;

	/**** PageProgram Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsInArray;
		void *pArray;
	} PageProgram;

	/**** PageErase Parameters ****/
	struct {
		uPageType upgPageNr;
	} PageErase;

	/**** SectorErase Parameters ****/
	struct {
		uSectorType ustSectorNr;
	} SectorErase;

	/**** Write Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsInArray;
		void *pArray;
	} Write;

	/**** Program Parameters ****/
	struct {
		uAddrType udAddr;
		uint32_t udNrOfElementsInArray;
		void *pArray;
	} Program;

} ParameterType;


/******************************************************************************
	Standard functions
*******************************************************************************/
	ReturnType	Flash( InstructionType insInstruction, ParameterType *fp );
	ReturnType	Flash_WriteEnable( void );
	ReturnType	Flash_WriteDisable( void );
	ReturnType	Flash_ReadDeviceID( uint16_t *uwpDeviceIdentification);
#ifdef USE_JEDEC_STANDARD_TWO_BYTE_SIGNATURE
	ReturnType	Flash_ReadManufacturerID( uint8_t *ucpManufacturerIdentification);
#endif
	ReturnType	Flash_ReadStatusReg( uint8_t *ucpStatusRegister);
	ReturnType	Flash_WriteStatusReg( uint8_t ucStatusRegister);
	ReturnType	Flash_Read( uAddrType udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead);
	ReturnType	Flash_FastRead( uAddrType udAddr, uint8_t *ucpElements, uint32_t udNrOfElementsToRead);
	ReturnType	Flash_PageProgram( uAddrType udAddr, uint8_t *pArray, uint16_t udNrOfElementsInArray );
	ReturnType	Flash_SectorErase( uSectorType uscSectorNr );
	ReturnType	Flash_BulkErase( void );
#ifndef NO_DEEP_POWER_DOWN_SUPPORT
	ReturnType	Flash_DeepPowerDown( void );
	ReturnType	Flash_ReleaseFromDeepPowerDown( void );
#endif
	ReturnType	Flash_Program( uint32_t udAddr, uint8_t *pArray , uint32_t udNrOfElementsInArray);

/******************************************************************************
	Utility functions
*******************************************************************************/
#ifdef VERBOSE
	char *Flash_ErrorStr( ReturnType rErrNum );
#endif

	ReturnType	Flash_TimeOut( uint32_t udSeconds );

/*******************************************************************************
List of Errors and Return values, Explanations and Help.
********************************************************************************

Error Name:	Flash_AddressInvalid
Description:	The address given is out of the range of the Flash device.
Solution:	Check whether the address is in the valid range of the
			Flash device.
********************************************************************************

Error Name:	Flash_PageEraseFailed
Description:	The Page erase Instruction did not complete successfully.
Solution:	Try to erase the Page again. If this fails once more, the device
			may be faulty and need to be replaced.
********************************************************************************

Error Name:	Flash_PageNrInvalid
Note:	The Flash memory is not at fault.
Description:	A Page has been selected (Parameter), which is not
			within the valid range. Valid Page numbers are from 0 to
			FLASH_PAGE_COUNT - 1.
Solution:	Check that the Page number given is in the valid range.
********************************************************************************

Error Name:	Flash_SectorNrInvalid
Note:	The Flash memory is not at fault.
Description:	A Sector has been selected (Parameter), which is not
			within the valid range. Valid Page numbers are from 0 to
			FLASH_SECTOR_COUNT - 1.
Solution:	Check that the Sector number given is in the valid range.
********************************************************************************

Return Name:	Flash_FunctionNotSupported
Description:	The user has attempted to make use of a functionality not
			available on this Fash device (and thus not provided by the
			software drivers).
Solution:	This may happen after changing Flash SW Drivers in existing
			environments. For example an application tries to use a
			functionality which is no longer provided with the new device.
********************************************************************************

Return Name:	Flash_NoInformationAvailable
Description:	The system cannot give any additional information about the error.
Solution:	None
********************************************************************************

Error Name:	Flash_OperationOngoing
Description:	This message is one of two messages that are given by the TimeOut
			subroutine. It means that the ongoing Flash operation is still within
			the defined time frame.
********************************************************************************

Error Name:	Flash_OperationTimeOut
Description:	The Program/Erase Controller algorithm could not finish an
			operation successfully. It should have set bit 7 of the Status
			Register from 0 to 1, but that did not happen within a predetermined
			time. The program execution was therefore cancelled by a
			timeout. This may be because the device is damaged.
Solution:	Try the previous Instruction again. If it fails a second time then it
			is likely that the device will need to be replaced.
********************************************************************************

Error Name:	Flash_ProgramFailed
Description:	The value that should be programmed has not been written correctly
			to the Flash memory.
Solutions:	Make sure that the Page which is supposed to receive the value
			was erased successfully before programming. Try to erase the Page and
			to program the value again. If it fails again then the device may
			be faulty.
********************************************************************************

Error Name:	Flash_WrongType
Description:	This message appears if the Manufacture and Device Identifications read from
			the current Flash device do not match the expected identifier
			codes. This means that the source code is not explicitely written for
			the currently used Flash chip. It may work, but the operation cannot be
			guaranteed.
Solutions:	Use a different Flash chip with the target hardware or contact
			STMicroelectronics for a different source code library.
********************************************************************************

Return Name:	Flash_Success
Description:	This value indicates that the Flash memory Instruction was executed
			correctly.
********************************************************************************/

/******************************************************************************
	External variable declaration
*******************************************************************************/

// none in this version of the release

/*******************************************************************************
Flash Status Register Definitions (see Datasheet)
*******************************************************************************/
enum
{
	SPI_FLASH_SRWD = 0x80,	// Status Register Write Protect
	SPI_FLASH_BP2	= 0x10,	// Block Protect Bit2
	SPI_FLASH_BP1	= 0x08,	// Block Protect Bit1
	SPI_FLASH_BP0	= 0x04,	// Block Protect Bit0
	SPI_FLASH_WEL	= 0x02,	// write enable latch
	SPI_FLASH_WIP	= 0x01	// write/program/erase in progress indicator
};

/*******************************************************************************
Specific Function Prototypes
*******************************************************************************/
#ifndef BOOL
#define BOOL int
#define TRUE 1
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

BOOL IsFlashBusy(void);
void SpiOpen();
/*******************************************************************************
List of Specific Errors and Return values, Explanations and Help.
*******************************************************************************

// none in this version of the release
********************************************************************************/

#endif /* __EXT_FLASH__H	*/
/* In order to avoid a repeated usage of the header file */

/*******************************************************************************
	End of ext_flash.h
********************************************************************************/
