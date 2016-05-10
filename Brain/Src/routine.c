
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#define __USED_BY_ROUTINE__
#include "global.h"
#include "routine.h"
#include "main.h"

extern osSemaphoreId RTN_tNeedToUpdateMotorHandle;


void RTN_updateMotor(void const * argument)
{
	uint8_t unMotorIndex;

	for(;;)
	{
		xSemaphoreTake(RTN_tNeedToUpdateMotorHandle, portMAX_DELAY);
		for (unMotorIndex = 0; unMotorIndex < MOTOR_NUMBER; unMotorIndex++)
		{

		}
	    if (ERROR == tErrorStatus)
	    {
	      HAL_GPIO_WritePin(GPIO_DEMO_LED_PORT, DEMO_LED_Pin, GPIO_PIN_RESET);
	    }
	    else
	    {
	      HAL_GPIO_TogglePin(GPIO_DEMO_LED_PORT, DEMO_LED_Pin);
	    }
	}
}
