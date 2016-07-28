
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"

#define __USED_BY_AIR_QUALITY__
#include "air_quality.h"
#include "main.h"



void AQ_Measure(void const * argument)
{
	while(1){
		HAL_GPIO_WritePin(AIR_QLT_LED_ENABLE_GPIO_Port, AIR_QLT_LED_ENABLE_Pin, GPIO_PIN_RESET);
		// Start timer
		__HAL_TIM_SET_COUNTER(&NRF905_COMM_TIMEOUT_HANDLER, 0);
		__HAL_TIM_SET_AUTORELOAD(&NRF905_COMM_TIMEOUT_HANDLER, ADC_START_DELAY_AFTER_PULSE);
		HAL_TIM_Base_Start_IT(&AIR_QUALITY_MSR_DELAY_HANDLER);
		osDelay(100);
	}
}
