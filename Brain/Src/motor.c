
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

#define __USED_BY_MOTOR__
#include "global.h"
#include "motor.h"
#include "main.h"

extern osSemaphoreId MTR_tMotorSpeedChangedHandle;
extern osSemaphoreId MTR_tMotorSPI_CommCpltHandle;
extern osMessageQId MotorCommQueueHandle;
__IO uint32_t unTestCNT = 0;
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

//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	static portBASE_TYPE tHigherPriorityTaskWoken;
//	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
//	{
//		tHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
//		MTR_tMotor[MTR_unMotorSelectedIndex].structMotor.unCommCNT++;
//		if(tHigherPriorityTaskWoken == pdTRUE)
//		{
//			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
//		}
//	}
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	static portBASE_TYPE tHigherPriorityTaskWoken;
//	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
//	{
//		tHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
//		MTR_tMotor[MTR_unMotorSelectedIndex].structMotor.unCommCNT++;
//		if(tHigherPriorityTaskWoken == pdTRUE)
//		{
//			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
//		}
//	}
//}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
	{
		if ((HAL_SPI_GetError(hspi) & HAL_SPI_ERROR_CRC) != 0)
		{
			tHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
			MTR_tMotor[MTR_unMotorSelectedIndex].structMotor.unCommErrCNT++;
			if(tHigherPriorityTaskWoken == pdTRUE)
			{
				portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
			}
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
	{
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
		MTR_tMotor[MTR_unMotorSelectedIndex].structMotor.unCommOK_CNT++;
		if(tHigherPriorityTaskWoken == pdTRUE)
		{
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	if (hadc->Instance == MOTOR_SPEED_ADC_HANDLER.Instance)
	{
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(MTR_tMotorSpeedChangedHandle, &tHigherPriorityTaskWoken);
		if(tHigherPriorityTaskWoken == pdTRUE)
		{
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}		
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
	__IO uint16_t unMotorSpeedADC;
	__IO uint8_t unMotorIndex;
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
	tMotorComm.unPayLoad[0] = 1 | MTR_COMM_RW_CMD_MASK;	//COMM_READ_MSR | MTR_COMM_RW_CMD_MASK;
	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}

//	tMotorComm.unPayLoad[0] = COMM_READ_RPM | MTR_COMM_RW_CMD_MASK;
//	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//	{
//		M_handleErr(NOT_USED_FOR_NOW);
//	}

//	tMotorComm.unPayLoad[0] = COMM_READ_BATTERY | MTR_COMM_RW_CMD_MASK;
//	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//	{
//		M_handleErr(NOT_USED_FOR_NOW);
//	}

//	tMotorComm.unPayLoad[0] = COMM_READ_CURRENT | MTR_COMM_RW_CMD_MASK;
//	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//	{
//		M_handleErr(NOT_USED_FOR_NOW);
//	}
}

void MTR_parseReadData(MOTOR_SPI_COMM_T* pMotorComm, uint16_t* pMotorCommRxBuffer)
{
	// CRC check
	if (HAL_SPI_GetError(&MOTOR_COMM_SPI_HANDLER) == HAL_SPI_ERROR_NONE)
	{
		MTR_tMotor[pMotorComm->unMotorIndex].unValue[pMotorComm->unPayLoad[0]] = pMotorCommRxBuffer[0];
	}
}

void sendDummyCMDtoRead(MOTOR_SPI_COMM_T* pMotorCommLast, uint16_t* pMotorCommRxBuffer)
{
	SELECT_MOTOR(pMotorCommLast->unMotorIndex)
	if(HAL_SPI_TransmitReceive_DMA(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(T_MOTOR_DUMMY_CMD.unPayLoad), (uint8_t *)pMotorCommRxBuffer,
			MTR_COMM_RD_CMD_CNT - 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	else
	{
		if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		else
		{
			MTR_parseReadData(pMotorCommLast, pMotorCommRxBuffer);
		}
	}
	DESELECT_MOTOR(pMotorCommLast->unMotorIndex)
	*pMotorCommLast = T_MOTOR_DUMMY_CMD;
}



// If there is no other command need to send or the next command is send to another motor
// and the last command is a read command, send one dummy command to this motor to retrieve read value
void MTR_MotorComm(void const * argument)
{
	MOTOR_SPI_COMM_T tMotorComm = MTR_DUMMY_CMD_CONTENT;
	__IO MOTOR_SPI_COMM_T tMotorCommLast = MTR_DUMMY_CMD_CONTENT;
	__IO uint16_t unMotorCommRxBuffer[MAX_MOTOR_COMM_LENGTH + 1];
	
	DESELECT_ALL_MOTOR;
	for(;;)
	{
		if (xQueueReceive(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}

		if (tMotorComm.unMotorIndex >= MOTOR_NUMBER)
		{
			continue;
		}

		/*********************************************************/
		/* ----======== Pre send command stage start ========----*/
		// If this command belongs to a different motor and last time command is one read command
		// DUMMY/Invalid command is already considered in IS_MTR_COMM_RD_CMD macro
//		if ((tMotorComm.unMotorIndex != tMotorCommLast.unMotorIndex) && IS_MTR_COMM_RD_CMD(tMotorCommLast.unPayLoad[0]))
//		{
//			sendDummyCMDtoRead(&tMotorCommLast, unMotorCommRxBuffer);
//		}
		/* ----======== Pre send command stage end ========----*/
		/*********************************************************/


		/*********************************************************/
		/* ----======== Send command stage start ========----*/
		SELECT_MOTOR(tMotorComm.unMotorIndex)
//		if(HAL_SPI_Transmit_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(tMotorComm.unPayLoad),
//			(IS_MTR_COMM_RD_CMD(tMotorComm.unPayLoad[0]) ? (MTR_COMM_RD_CMD_CNT - 1) : (MTR_COMM_WR_CMD_CNT - 1))) != HAL_OK)


		if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(tMotorComm.unPayLoad), (uint8_t *)unMotorCommRxBuffer,
				(IS_MTR_COMM_RD_CMD(tMotorComm.unPayLoad[0]) ? (MTR_COMM_RD_CMD_CNT - 1) : (MTR_COMM_WR_CMD_CNT - 1))) != HAL_OK)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
		else
		{
			// Communication finished, but the data read in this transaction belongs to last time's query
			// Same motor, last time read command (not DUMMY/Invalid)
//			if (IS_MTR_COMM_RD_CMD(tMotorCommLast.unPayLoad[0]))
//			{
//				// analyze it
//				MTR_parseReadData(&tMotorCommLast, unMotorCommRxBuffer);
//			}
			unTestCNT++;
		}
		DESELECT_MOTOR(tMotorComm.unMotorIndex)

		/* ----======== Send command stage end ========----*/
		/*********************************************************/


		/*********************************************************/
		/* ----======== After send command stage start ========----*/
		// There is no command anymore
//		if ((uxQueueMessagesWaiting(MotorCommQueueHandle) == 0) && (IS_MTR_COMM_RD_CMD(tMotorComm.unPayLoad[0])))
//		{
//			// No communication any more,
//			// if last communication is one read command, send one dummy read to get last read result
//			sendDummyCMDtoRead(&tMotorCommLast, unMotorCommRxBuffer);
//		}
//		else
//		{
//			tMotorCommLast = tMotorComm;
//		}
		/* ----======== After send command stage end ========----*/
		/*********************************************************/
	}
}
