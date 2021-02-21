/*!
 *		@file			  MB_board.h
 *  	@author  		Vy Luu
 *		@copyright	Fiot Co.,Ltd
 *  	@version 		1.0
 *  	@date    		2016-12-17
 *		@brief			This file is responsible for setting up the board
 */
#ifndef __MB_BOARD_H__
#define __MB_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>
/********************************************************************************
 * EXTERNS
 ********************************************************************************/
extern const PIN_Config BoardGpioInitTable[];

/********************************************************************************
 * CONSTANTS
 ********************************************************************************/
/* Same RF Configuration as 7x7 EM */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

#define Board_M25P80_CS             PIN_UNASSIGNED
#define Board_M25P80_SCK            PIN_UNASSIGNED
#define Board_M25P80_MISO           PIN_UNASSIGNED
#define Board_M25P80_MOSI           PIN_UNASSIGNED

/* UART Board */
#define Board_UART_RX               IOID_2          /* RXD  */
#define Board_UART_TX               IOID_3          /* TXD  */
#define Board_UART_CTS              PIN_UNASSIGNED         /* CTS  */
#define Board_UART_RTS              PIN_UNASSIGNED         /* RTS */

/* I2C Board */
#define Board_I2C_SCL               PIN_UNASSIGNED
#define Board_I2C_SDA               PIN_UNASSIGNED

/* SPI Board */
#define Board_SPI0_MISO             PIN_UNASSIGNED          /* RF1.20 */
#define Board_SPI0_MOSI             PIN_UNASSIGNED          /* RF1.18 */
#define Board_SPI0_CLK              PIN_UNASSIGNED         /* RF1.16 */
#define Board_SPI0_CSN              PIN_UNASSIGNED
#define Board_SPI1_MISO             PIN_UNASSIGNED
#define Board_SPI1_MOSI             PIN_UNASSIGNED
#define Board_SPI1_CLK              PIN_UNASSIGNED
#define Board_SPI1_CSN              PIN_UNASSIGNED

/* SPI */
#define Board_SPI_FLASH_CS          PIN_UNASSIGNED
#define Board_FLASH_CS_ON           0
#define Board_FLASH_CS_OFF          1

/* Booster pack generic */
#define Board_DIO0                  PIN_UNASSIGNED
#define Board_DIO1_RFSW             PIN_UNASSIGNED
#define Board_DIO12                 PIN_UNASSIGNED
#define Board_DIO15                 PIN_UNASSIGNED
#define Board_DIO16_TDO             PIN_UNASSIGNED
#define Board_DIO17_TDI             PIN_UNASSIGNED
#define Board_DIO21                 PIN_UNASSIGNED
#define Board_DIO22                 PIN_UNASSIGNED

#define Board_DIO23_ANALOG          PIN_UNASSIGNED
#define Board_DIO24_ANALOG          PIN_UNASSIGNED
#define Board_DIO25_ANALOG          PIN_UNASSIGNED
#define Board_DIO26_ANALOG          PIN_UNASSIGNED
#define Board_DIO27_ANALOG          PIN_UNASSIGNED
#define Board_DIO28_ANALOG          PIN_UNASSIGNED
#define Board_DIO29_ANALOG          PIN_UNASSIGNED
#define Board_DIO30_ANALOG          PIN_UNASSIGNED

/* Booster pack LCD (430BOOST - Sharp96 Rev 1.1) */
#define Board_LCD_CS                PIN_UNASSIGNED // SPI chip select
#define Board_LCD_EXTCOMIN          PIN_UNASSIGNED // External COM inversion
#define Board_LCD_ENABLE            PIN_UNASSIGNED // LCD enable
#define Board_LCD_POWER             PIN_UNASSIGNED // LCD power control
#define Board_LCD_CS_ON             1
#define Board_LCD_CS_OFF            0

/* PWM outputs */
#define Board_PWMPIN0               PIN_UNASSIGNED
#define Board_PWMPIN1               PIN_UNASSIGNED
#define Board_PWMPIN2               PIN_UNASSIGNED
#define Board_PWMPIN3               PIN_UNASSIGNED
#define Board_PWMPIN4               PIN_UNASSIGNED
#define Board_PWMPIN5               PIN_UNASSIGNED
#define Board_PWMPIN6               PIN_UNASSIGNED
#define Board_PWMPIN7               PIN_UNASSIGNED
/********************************************************************************
 * INSTANCE IDENTIFIERS
 ********************************************************************************/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC2650_LAUNCHXL_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650_LAUNCHXL_SPI0
#define Board_SPI1                  CC2650_LAUNCHXL_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  CC2650_LAUNCHXL_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650_LAUNCHXL_CRYPTO0
/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             CC2650_LAUNCHXL_GPTIMER0A
#define Board_GPTIMER0B             CC2650_LAUNCHXL_GPTIMER0B
#define Board_GPTIMER1A             CC2650_LAUNCHXL_GPTIMER1A
#define Board_GPTIMER1B             CC2650_LAUNCHXL_GPTIMER1B
#define Board_GPTIMER2A             CC2650_LAUNCHXL_GPTIMER2A
#define Board_GPTIMER2B             CC2650_LAUNCHXL_GPTIMER2B
#define Board_GPTIMER3A             CC2650_LAUNCHXL_GPTIMER3A
#define Board_GPTIMER3B             CC2650_LAUNCHXL_GPTIMER3B
/* Generic PWM instance identifiers */
#define Board_PWM0                  CC2650_LAUNCHXL_PWM0
#define Board_PWM1                  CC2650_LAUNCHXL_PWM1
#define Board_PWM2                  CC2650_LAUNCHXL_PWM2
#define Board_PWM3                  CC2650_LAUNCHXL_PWM3
#define Board_PWM4                  CC2650_LAUNCHXL_PWM4
#define Board_PWM5                  CC2650_LAUNCHXL_PWM5
#define Board_PWM6                  CC2650_LAUNCHXL_PWM6
#define Board_PWM7                  CC2650_LAUNCHXL_PWM7
/********************************************************************************
 * NUMBER OF PERIPHERALS AND THEIR NAMES
 ********************************************************************************/

/*!
 *  @def    CC2650_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_I2CName {
    CC2650_LAUNCHXL_I2C0 = 0,

    CC2650_LAUNCHXL_I2CCOUNT
} CC2650_LAUNCHXL_I2CName;

/*!
 *  @def    CC2650_LAUNCHXL_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_CryptoName {
    CC2650_LAUNCHXL_CRYPTO0 = 0,

    CC2650_LAUNCHXL_CRYPTOCOUNT
} CC2650_LAUNCHXL_CryptoName;


/*!
 *  @def    CC2650_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_SPIName {
    CC2650_LAUNCHXL_SPI0 = 0,
    CC2650_LAUNCHXL_SPI1,
    CC2650_LAUNCHXL_SPICOUNT
} CC2650_LAUNCHXL_SPIName;

/*!
 *  @def    CC2650_LAUNCHXL_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2650_LAUNCHXL_TRNGName {
    CC2650_LAUNCHXL_TRNG0 = 0,
    CC2650_LAUNCHXL_TRNGCOUNT
} CC2650_LAUNCHXL_TRNGName;

/*!
 *  @def    CC2650_LAUNCHXL_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_UARTName {
    CC2650_LAUNCHXL_UART0 = 0,

    CC2650_LAUNCHXL_UARTCOUNT
} CC2650_LAUNCHXL_UARTName;

/*!
 *  @def    CC2650_LAUNCHXL_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650_LAUNCHXL_UdmaName {
    CC2650_LAUNCHXL_UDMA0 = 0,

    CC2650_LAUNCHXL_UDMACOUNT
} CC2650_LAUNCHXL_UdmaName;

/*!
 *  @def    CC2650_LAUNCHXL_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2650_LAUNCHXL_GPTimerName
{
    CC2650_LAUNCHXL_GPTIMER0A = 0,
    CC2650_LAUNCHXL_GPTIMER0B,
    CC2650_LAUNCHXL_GPTIMER1A,
    CC2650_LAUNCHXL_GPTIMER1B,
    CC2650_LAUNCHXL_GPTIMER2A,
    CC2650_LAUNCHXL_GPTIMER2B,
    CC2650_LAUNCHXL_GPTIMER3A,
    CC2650_LAUNCHXL_GPTIMER3B,
    CC2650_LAUNCHXL_GPTIMERPARTSCOUNT
} CC2650_LAUNCHXL_GPTimerName;

/*!
 *  @def    CC2650_LAUNCHXL_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2650_LAUNCHXL_GPTimers
{
    CC2650_LAUNCHXL_GPTIMER0 = 0,
    CC2650_LAUNCHXL_GPTIMER1,
    CC2650_LAUNCHXL_GPTIMER2,
    CC2650_LAUNCHXL_GPTIMER3,
    CC2650_LAUNCHXL_GPTIMERCOUNT
} CC2650_LAUNCHXL_GPTimers;

/*!
 *  @def    CC2650_LAUNCHXL_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum CC2650_LAUNCHXL_PWM
{
    CC2650_LAUNCHXL_PWM0 = 0,
    CC2650_LAUNCHXL_PWM1,
    CC2650_LAUNCHXL_PWM2,
    CC2650_LAUNCHXL_PWM3,
    CC2650_LAUNCHXL_PWM4,
    CC2650_LAUNCHXL_PWM5,
    CC2650_LAUNCHXL_PWM6,
    CC2650_LAUNCHXL_PWM7,
    CC2650_LAUNCHXL_PWMCOUNT
} CC2650_LAUNCHXL_PWM;

/*!
 *  @def    CC2650_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum CC2650_LAUNCHXL_ADCBufName {
    CC2650_LAUNCHXL_ADCBuf0 = 0,
    CC2650_LAUNCHXL_ADCBufCOUNT
} CC2650_LAUNCHXL_ADCBufName;


/*!
 *  @def    CC2650_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC2650_LAUNCHXL_ADCName {
    CC2650_LAUNCHXL_ADC0 = 0,
    CC2650_LAUNCHXL_ADC1,
    CC2650_LAUNCHXL_ADC2,
    CC2650_LAUNCHXL_ADC3,
    CC2650_LAUNCHXL_ADC4,
    CC2650_LAUNCHXL_ADC5,
    CC2650_LAUNCHXL_ADC6,
    CC2650_LAUNCHXL_ADC7,
    CC2650_LAUNCHXL_ADCDCOUPL,
    CC2650_LAUNCHXL_ADCVSS,
    CC2650_LAUNCHXL_ADCVDDS,
    CC2650_LAUNCHXL_ADCCOUNT
} CC2650_LAUNCHXL_ADCName;


#ifdef __cplusplus
}
#endif

#endif /* __MB_BOARD_H__ */
