/*!
 *	@file				main.c
 *  @author  		Dinh Le
 *	@copyright	Fiot Co.,Ltd
 *  @version 		1.0
 *  @date    		2016-04-12
 *	@brief			Main entry of the BLE MotionBand application.
 *
 */
/********************************************************************************
 *	INCLUDES
 ********************************************************************************/
// TI RTOS
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Error.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <icall.h>
#include "MB_Uart.h"
#include "peripheral.h"
// BLE
#include "bcomdef.h"

// Application
#include "MB_Define.h"

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

/********************************************************************************
 * MACROS
 ********************************************************************************/

/********************************************************************************
 * CONSTANTS
 ********************************************************************************/

/********************************************************************************
 * TYPEDEFS
 ********************************************************************************/

/********************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************/

/********************************************************************************
 * EXTERNS
 ********************************************************************************/

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*******************************************************************************
 * @fn          Main
 *
 * @brief       Application Main
 *
 * @param       None.
 *
 * @return      None.
 ********************************************************************************/
int main()
{
  //!********************************************************
  //!Register Application callback to trap asserts raised in the Stack
  //!********************************************************
  RegisterAssertCback(AssertHandler);
  //!********************************************************
  //!Initilize Board
  //!********************************************************
  PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
	//!********************************************************
  //!Set constraints for Standby, powerdown and idle mode
	//!PowerCC26XX_SB_DISALLOW may be redundant
	//!********************************************************
  Power_setConstraint(PowerCC26XX_SB_DISALLOW);
  Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif // POWER_SAVING
	//!********************************************************
  //!Initialize ICall module
	//!********************************************************
  ICall_init();
	//!********************************************************
  //!Start tasks of external images - Priority 5
	//!********************************************************
  ICall_createRemoteTasks();
	//!********************************************************
  //!Kick off profile - Priority 3
	//!********************************************************
  GAPRole_createTask();
	//!********************************************************
  //!Control Main Task - Priority 1
	//!********************************************************
  MB_CreateTask();
	//!********************************************************
  //!Initiliaze UART
	//!********************************************************
  FIOT_UART_CreateTask();
  	//!********************************************************
  //!enable interrupts and start SYS/BIOS
	//!********************************************************
  BIOS_start();

  return 0;
}
/*******************************************************************************
 * @fn          AssertHandler
 *
 * @brief       This is the Application's callback handler for asserts raised
 *              in the stack.  When EXT_HAL_ASSERT is defined in the Stack
 *              project this function will be called when an assert is raised,
 *              and can be used to observe or trap a violation from expected
 *              behavior.
 *
 *              As an example, for Heap allocation failures the Stack will raise
 *              HAL_ASSERT_CAUSE_OUT_OF_MEMORY as the assertCause and
 *              HAL_ASSERT_SUBCAUSE_NONE as the assertSubcause.  An application
 *              developer could trap any malloc failure on the stack by calling
 *              HAL_ASSERT_SPINLOCK under the matching case.
 *
 *              An application developer is encouraged to extend this function
 *              for use by their own application.  To do this, add hal_assert.c
 *              to your project workspace, the path to hal_assert.h (this can
 *              be found on the stack side). Asserts are raised by including
 *              hal_assert.h and using macro HAL_ASSERT(cause) to raise an
 *              assert with argument assertCause.  the assertSubcause may be
 *              optionally set by macro HAL_ASSERT_SET_SUBCAUSE(subCause) prior
 *              to asserting the cause it describes. More information is
 *              available in hal_assert.h.
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * @return      None.
 ********************************************************************************/
void AssertHandler(uint8 assertCause, uint8 assertSubcause)
{
  // check the assert cause
  switch (assertCause)
  {
    default:
      HAL_ASSERT_SPINLOCK;
  }

  return;
}
/********************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS.
 *
 * @param       eb - Pointer to Error Block.
 *
 * @return      None.
 ********************************************************************************/
void smallErrorHook(Error_Block *eb)
{
  for (;;);
}
/********************************************************************************
 * END
 ********************************************************************************/
