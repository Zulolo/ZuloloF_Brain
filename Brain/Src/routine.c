
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
#include "gpio.h"

#define __USED_BY_ROUTINE__
#include "global.h"
#include "routine.h"
#include "motor.h"
#include "main.h"

extern osSemaphoreId RTN_tNeedToUpdateMotorHandle;

void RTN_updateMotor(void const * argument)
{
//	uint8_t unMotorIndex;
	
	HAL_ADC_Start_DMA(&MOTOR_SPEED_ADC_HANDLER, (uint32_t *)&unMotorSpeedADC_Buf, MOTOR_SPEED_ADC_DMA_DEPTH);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim6);
	for(;;)
	{
		xSemaphoreTake(RTN_tNeedToUpdateMotorHandle, portMAX_DELAY);
//		for (unMotorIndex = 0; unMotorIndex < MOTOR_NUMBER; unMotorIndex++)
//		{
			MTR_unReadMotorStatus(MOTOR_NUMBER);
//			MTR_unUpdateMotorStatus(MOTOR_NUMBER);
//		}
	}
}
