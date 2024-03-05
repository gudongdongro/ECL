//--------------------------------------------------
// stm32f4xx_it.c
// Interrupt Service Routines.
//--------------------------------------------------

//#include "stm32f4xx_hal.h" // deleted by Prof. Lee
#include "stm32f4xx_hal_pcd.h"  // added by Prof. Lee
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

// External variables
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/*************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers   */
/*************************************************************/

void NMI_Handler(void){}

void HardFault_Handler(void)
{
  while (1){}
}

void MemManage_Handler(void)
{
  while (1){}
}

void BusFault_Handler(void)
{
  while (1){}
}

void UsageFault_Handler(void)
{
  while (1){}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void){}

void PendSV_Handler(void){}

/*   // deleted by Prof. Lee. 우리의 경우 SysTick_Handler를 안씀
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
*/

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
