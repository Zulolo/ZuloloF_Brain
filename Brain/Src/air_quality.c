
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "string.h"

#define __USED_BY_AIR_QUALITY__
#include "air_quality.h"
#include "main.h"

extern osSemaphoreId AQ_tADC_Cnvt_CpltHandle;
uint16_t unMaxADC_Value;

int32_t AQ_nDataManager(uint32_t unADC_Value)
{
	static uint8_t unIndex = 0;
	static uint16_t unADC_ValueBuff[ADC_VALUE_BUFF_LEN] = {0, };
	uint16_t unADC_ValueAvg;
	
	if (unADC_Value > unMaxADC_Value){
		unMaxADC_Value = unADC_Value;
	}
	unADC_ValueBuff[unIndex] = (uint16_t)unADC_Value;
	unIndex++;
	if (ADC_VALUE_BUFF_LEN == unIndex){
		unIndex = 0;
		unADC_ValueAvg = BLB_unGetAverage_16Bits(unADC_ValueBuff, ADC_VALUE_BUFF_LEN);
		IS_GLOBAL_PARA_ACCESSABLE;
		tSensoreData.unAirQuality = unADC_ValueAvg;
		RELEASE_GLOBAL_PARA_ACCESS;
	}	
	return 0;	
}
		
void AQ_Measure(void const * argument)
{
	while(1){
		HAL_GPIO_WritePin(AIR_QLT_LED_ENABLE_GPIO_Port, AIR_QLT_LED_ENABLE_Pin, GPIO_PIN_RESET);
		// Start timer
		__HAL_TIM_SET_COUNTER(&AIR_QUALITY_MSR_DELAY_HANDLER, 0);
		__HAL_TIM_SET_AUTORELOAD(&AIR_QUALITY_MSR_DELAY_HANDLER, ADC_START_DELAY_AFTER_PULSE);
		HAL_TIM_Base_Start_IT(&AIR_QUALITY_MSR_DELAY_HANDLER);
		
		xSemaphoreTake(AQ_tADC_Cnvt_CpltHandle, portMAX_DELAY);
		AQ_nDataManager(HAL_ADC_GetValue(&AIR_QUALITY_ADC_HANDLER));
		osDelay(10);
	}
}
