
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"

#define __USED_BY_TM__
#include "global.h"
#include "tm_measure.h"
#include "main.h"

#define MAX_TH_MEASURE_BIT_NUM											40
#define AM2301_MEASUREMENT_BYTE_NUM									5
#define RESET_BIT_IN_BYTE_ARRAY(pArray, unIndex)		(*((pArray) + (unIndex)/BIT_NUM_PER_BYTE) = \
																											*((pArray) + (unIndex)/BIT_NUM_PER_BYTE) & (~(0x01 << (BIT_NUM_PER_BYTE - 1 - (unIndex)%BIT_NUM_PER_BYTE))))
#define SET_BIT_IN_BYTE_ARRAY(pArray, unIndex)			(*((pArray) + (unIndex)/BIT_NUM_PER_BYTE) = \
																											*((pArray) + (unIndex)/BIT_NUM_PER_BYTE) | (0x01 << (BIT_NUM_PER_BYTE - 1 - (unIndex)%BIT_NUM_PER_BYTE)))
extern osSemaphoreId TH_tMeasureData_CommCpltHandle;

uint8_t unMeasureRslt[AM2301_MEASUREMENT_BYTE_NUM];

int32_t sendStartTrigger(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = AM2301_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(AM2301_DATA_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(AM2301_DATA_GPIO_Port, AM2301_DATA_Pin, GPIO_PIN_RESET);
	osDelay(2);
	return 0;
}
	
int32_t waitTH_Response(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	HAL_GPIO_WritePin(AM2301_DATA_GPIO_Port, AM2301_DATA_Pin, GPIO_PIN_SET);
		
	GPIO_InitStruct.Pin = AM2301_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(AM2301_DATA_GPIO_Port, &GPIO_InitStruct);
	return 0;
}
		
int32_t startRecv(uint32_t* pTempMeasureRslt){
//	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_TIM_IC_Stop_DMA(&TH_RCV_DATA_TIMER_HANDLER, TIM_CHANNEL_1);
	__HAL_TIM_SET_COUNTER(&TH_RCV_DATA_TIMER_HANDLER, 0);
	if (HAL_TIM_IC_Start_DMA(&TH_RCV_DATA_TIMER_HANDLER, TIM_CHANNEL_1, pTempMeasureRslt, MAX_TH_MEASURE_BIT_NUM + 2) != HAL_OK){
		return (-1);
	}
	xSemaphoreTake(TH_tMeasureData_CommCpltHandle, portMAX_DELAY);
//	GPIO_InitStruct.Pin = AM2301_DATA_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	HAL_GPIO_Init(AM2301_DATA_GPIO_Port, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(AM2301_DATA_GPIO_Port, AM2301_DATA_Pin, GPIO_PIN_SET);
	return 0;
}
			
uint8_t* handleTH_Data(uint16_t* pRawData){
	static uint8_t unTempValue[AM2301_MEASUREMENT_BYTE_NUM];
	uint8_t unIndex;
	
	for (unIndex = 0; unIndex < MAX_TH_MEASURE_BIT_NUM; unIndex++){
		if (((uint16_t)((*(pRawData + unIndex + 2) - *(pRawData + unIndex + 1)))) < 95){
			RESET_BIT_IN_BYTE_ARRAY(unTempValue, unIndex);
		}else{
			SET_BIT_IN_BYTE_ARRAY(unTempValue, unIndex);
		}
	}
	if ((uint8_t)(unTempValue[0] + unTempValue[1] + unTempValue[2] + unTempValue[3]) == unTempValue[4]){
		return unTempValue;
	}else{
		return NULL;
	}
}

int32_t measureTH(void){
	static uint32_t tempMeasureRslt[(MAX_TH_MEASURE_BIT_NUM + 2) >> 1];
	
	if (sendStartTrigger() < 0){
		return (-1);
	}
	
	if (waitTH_Response() < 0){
		return (-1);
	}
	
	if (startRecv(tempMeasureRslt) < 0){
		return (-1);
	}
		
	if (handleTH_Data((uint16_t*)tempMeasureRslt) == NULL){
		return (-1);
	}else{
		memcpy(unMeasureRslt, tempMeasureRslt, AM2301_MEASUREMENT_BYTE_NUM);
		return 0;		
	}
}
	
void TH_Measure(void const * argument)
{
	while(1){
		measureTH();
		osDelay(2000);
	}
}
