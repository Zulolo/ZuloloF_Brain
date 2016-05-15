
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

#define __USED_BY_MOTOR__
#include "global.h"
#include "motor.h"
#include "main.h"

extern osSemaphoreId MTR_tMotorSpeedChangedHandle;
extern osSemaphoreId MTR_tMotorSPI_CommCpltHandle;
extern osMessageQId MotorCommQueueHandle;

//void MTR_giveMotorSpeedADC_Sem(struct __DMA_HandleTypeDef * hdma)
//{
//  static portBASE_TYPE xHigherPriorityTaskWoken;
//  xHigherPriorityTaskWoken = pdFALSE;
//  xSemaphoreGiveFromISR(MTR_tMotorSpeedChangedSemaphore, &xHigherPriorityTaskWoken);
//  if(xHigherPriorityTaskWoken == pdTRUE)
//  {
//	  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//  }
//}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	tHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
	if(tHigherPriorityTaskWoken == pdTRUE)
	{
		portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	tHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(MTR_tMotorSpeedChangedHandle, &tHigherPriorityTaskWoken);
	if(tHigherPriorityTaskWoken == pdTRUE)
	{
		portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
	}
}

uint16_t MTR_calculateMotorSpeedADC(void)
{
	uint8_t unIndex;
	uint32_t unADC_Avg = 0;
	for (unIndex = 0; unIndex < MOTOR_SPEED_ADC_DMA_DEPTH; unIndex++)
	{
		unADC_Avg += unMotorSpeedADC_Buf[unIndex] & ADC_12BIT_MASK;
	}
	return ((uint16_t)(unADC_Avg / MOTOR_SPEED_ADC_DMA_DEPTH));
}

void MTR_ctrlMotor(void const * argument)
{
	static uint16_t unMotorSpeedADC;
	uint8_t unMotorIndex;
	for(;;)
	{
		xSemaphoreTake(MTR_tMotorSpeedChangedHandle, portMAX_DELAY);
		unMotorSpeedADC = MTR_calculateMotorSpeedADC();
		for (unMotorIndex = 0; unMotorIndex < MOTOR_NUMBER; unMotorIndex++)
		{
			MTR_tMotor[unMotorIndex].structMotor.unSpeedADC = unMotorSpeedADC;
		}
	}
}

void MTR_unUpdateMotorStatus(uint8_t unMotorIndex)
{
	static MOTOR_SPI_COMM_T tMotorComm;

	tMotorComm.unMotorIndex = unMotorIndex;
	tMotorComm.unPayLoad[0] = COMM_READ_MSR;
	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}

	tMotorComm.unPayLoad[0] = COMM_READ_RPM;
	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}

	tMotorComm.unPayLoad[0] = COMM_READ_BATTERY;
	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}

	tMotorComm.unPayLoad[0] = COMM_READ_CURRENT;
	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
}
BOOLEAN_T MTR_bIsReadCmd(MOTOR_SPI_COMM_T tMotorComm)
{
	return TRUE;
}

void MTR_parseReadData(MOTOR_SPI_COMM_T tMotorCommLast, uint16_t* pMotorCommRxBuffer)
{

}

void MTR_MotorComm(void const * argument)
{
	static MOTOR_SPI_COMM_T tMotorComm = MTR_DUMMY_CMD_CONTENT;
	static MOTOR_SPI_COMM_T tMotorCommLast = MTR_DUMMY_CMD_CONTENT;
	static uint16_t unMotorCommRxBuffer[MAX_MOTOR_COMM_LENGTH + 1];
	for(;;)
	{
		if (xQueueReceive(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)(tMotorComm.unPayLoad), (uint8_t *)unMotorCommRxBuffer, 3) != HAL_OK)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		// Comunication finished
		if (MTR_bIsReadCmd(tMotorCommLast) == TRUE)
		{
			// The data read during this transaction is the read command's response
			// analyze it
			MTR_parseReadData(tMotorCommLast, unMotorCommRxBuffer);
		}

		if ((uxQueueMessagesWaiting(MotorCommQueueHandle) == 0) && (MTR_bIsReadCmd(tMotorComm) == TRUE))
		{
			// No communication any more,
			// if last communication is one read command, send one dummy read to get last read result
			if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)(T_MOTOR_DUMMY_CMD.unPayLoad), (uint8_t *)unMotorCommRxBuffer, 3) != HAL_OK)
			{
				M_handleErr(NOT_USED_FOR_NOW);
			}
			tMotorCommLast = T_MOTOR_DUMMY_CMD;
			if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
			{
				M_handleErr(NOT_USED_FOR_NOW);
			}
		}
		else
		{
			tMotorCommLast = tMotorComm;
		}
	}
}
