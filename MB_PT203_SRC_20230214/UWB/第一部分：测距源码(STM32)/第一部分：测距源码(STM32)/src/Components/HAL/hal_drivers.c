#include "hal_drivers.h"

/**************************************************************************************************
 * @fn      Hal_DriverInit
 *
 * @brief   Initialize HW - These need to be initialized before anyone.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_Driver_Init (void)
{
	/* CFG 中断优先级组*/
	Hal_NVIC_Init(HAL_NVIC_GROUP_SN);
	
  /* TIMER */
#if (defined HAL_TIMER) && (HAL_TIMER == TRUE)
	HalTimerInit();
#endif

  /* FLASH */
#if (defined HAL_FLASH) && (HAL_FLASH == TRUE)
  HalFlashInit();
#endif	

  /* LED */
#if (defined HAL_LED) && (HAL_LED == TRUE)
  HalLedInit();
#endif

  /* UART */
#if (defined HAL_USART) && (HAL_USART == TRUE)
	HalUARTInit();
#endif
  
  /* SPI */
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
	HalSpiInit();
#endif

  /* HID */
#if (defined HAL_USB) && (HAL_USB == TRUE)
	//HalUsbInit();
#endif
}

