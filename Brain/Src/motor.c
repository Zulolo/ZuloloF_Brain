
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
	
uint16_t calCRC16(uint8_t* pBytes, uint32_t unLength)
{
	uint16_t crc = 0;
	uint32_t unIndex;
	uint8_t unPosInTable;
	
	for (unIndex = 0; unIndex < unLength; unIndex++)
	{
		/* XOR-in next input byte into MSB of crc, that's our new intermediate divident */
		unPosInTable = (uint8_t)((crc >> 8) ^ (*(pBytes + REVS_BYTE_ORDER(unIndex)))); /* equal: ((crc ^ (b << 8)) >> 8) */
		/* Shift out the MSB used for division per lookuptable and XOR with the remainder */
		crc = (uint16_t)((crc << 8) ^ (uint16_t)(CRC_TABLE16[unPosInTable]));
	}

	return crc;
}

void selectMotor(uint8_t unMotorIndex)
{
	if (unMotorIndex < MOTOR_NUMBER) 
	{
		HAL_GPIO_WritePin(SPI1_MOTOR_SELECT_Port[(unMotorIndex)], SPI1_MOTOR_SELECT_Pin[(unMotorIndex)], GPIO_PIN_RESET);
	}
}

void deselectMotor(uint8_t unMotorIndex)
{
	if ((unMotorIndex) < MOTOR_NUMBER)
	{
		HAL_GPIO_WritePin(SPI1_MOTOR_SELECT_Port[(unMotorIndex)], SPI1_MOTOR_SELECT_Pin[(unMotorIndex)], GPIO_PIN_SET);
	}
}
	

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
	{
		if ((HAL_SPI_GetError(hspi) & HAL_SPI_ERROR_CRC) != 0)
		{
//			__HAL_SPI_CLEAR_CRCERRFLAG(hspi);
			tHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
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
		if(tHigherPriorityTaskWoken == pdTRUE)
		{
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
	if (hspi->Instance == MOTOR_COMM_SPI_HANDLER.Instance)
	{
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(MTR_tMotorSPI_CommCpltHandle, &tHigherPriorityTaskWoken);
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

// Set motor target duty according to ADC
void setTargetDutyAccdToADC(MOTOR_UNION_T* tMotor)
{
	if (tMotor->structMotor.unSpeedADC < MTR_SPEED_ADC_MIN)
	{
		tMotor->structMotor.unTargetDuty = MIN_MOTOR_PWR_DUTY;
	}
	else if (tMotor->structMotor.unSpeedADC > MTR_SPEED_ADC_MAX)
	{
		tMotor->structMotor.unTargetDuty = MAX_MOTOR_PWR_DUTY;
	}
	else
	{
		tMotor->structMotor.unTargetDuty = tMotor->structMotor.unSpeedADC >> 2;
	}
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
			setTargetDutyAccdToADC(&(MTR_tMotor[unMotorIndex]));
		}
	}
}

//void MTR_unReadMotorStatus(uint8_t unMotorIndex)
//{
//	static MOTOR_SPI_COMM_T tMotorComm;

//	tMotorComm.unMotorIndex = unMotorIndex;
//	tMotorComm.unPayLoad[0] = COMM_READ_MSR | MTR_COMM_RW_CMD_MASK;
//	if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//	{
//		M_handleErr(NOT_USED_FOR_NOW);
//	}

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
//}

void MTR_unReadMotorStatus(uint8_t unMaxMotorNum)
{
	static MOTOR_SPI_COMM_T tMotorComm;
	static uint8_t unMotorIndex;
	
//	for (unMotorIndex = 0; unMotorIndex < unMaxMotorNum; unMotorIndex++)
//	{
//		tMotorComm.unMotorIndex = unMotorIndex;
//		tMotorComm.unPayLoad[0] = COMM_READ_MSR | MTR_COMM_RW_CMD_MASK;
//		if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//		{
//			M_handleErr(NOT_USED_FOR_NOW);
//		}	
//	}

//	for (unMotorIndex = 0; unMotorIndex < unMaxMotorNum; unMotorIndex++)
//	{
		tMotorComm.unMotorIndex = 0;	//unMotorIndex;
		tMotorComm.unPayLoad[0] = COMM_READ_LOCATING_PERIOD | MTR_COMM_RW_CMD_MASK;
		if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}
//	}
	
//	for (unMotorIndex = 0; unMotorIndex < unMaxMotorNum; unMotorIndex++)
//	{
//		tMotorComm.unMotorIndex = unMotorIndex;
//		tMotorComm.unPayLoad[0] = COMM_READ_BATTERY | MTR_COMM_RW_CMD_MASK;
//		if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//		{
//			M_handleErr(NOT_USED_FOR_NOW);
//		}
//	}
//	
//	for (unMotorIndex = 0; unMotorIndex < unMaxMotorNum; unMotorIndex++)
//	{
//		tMotorComm.unMotorIndex = unMotorIndex;
//		tMotorComm.unPayLoad[0] = COMM_READ_CURRENT | MTR_COMM_RW_CMD_MASK;
//		if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//		{
//			M_handleErr(NOT_USED_FOR_NOW);
//		}
//	}
}

void MTR_unUpdateMotorStatus(uint8_t unMaxMotorNum)
{
	static MOTOR_SPI_COMM_T tMotorComm;
	static uint8_t unMotorIndex;
	
	// Update speed
	for (unMotorIndex = 0; unMotorIndex < unMaxMotorNum; unMotorIndex++)
	{
		tMotorComm.unMotorIndex = unMotorIndex;
		tMotorComm.unPayLoad[0] = COMM_WRITE_TARGET_DUTY;
		tMotorComm.unPayLoad[1] = MTR_tMotor[unMotorIndex].structMotor.unTargetDuty;
		if (xQueueSendToBack(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
		{
			M_handleErr(NOT_USED_FOR_NOW);
		}	
	}
}
//	

void MTR_analyzeReadData(MOTOR_SPI_COMM_T* pMotorComm, uint16_t* pMotorCommRxBuffer)
{
	uint8_t unReadItem;
	uint16_t unCRC;
	
	// CRC check
//	unCRC = calCRC16((uint8_t *)pMotorCommRxBuffer, 2);
	
	unReadItem = MTR_GET_RD_ITEM(pMotorComm->unPayLoad[0]);
	// Validation check
	if (unReadItem < COMM_READ_MAX)
	{
		MTR_tMotor[pMotorComm->unMotorIndex].unValue[unReadItem] = pMotorCommRxBuffer[0];
		MTR_tMotor[pMotorComm->unMotorIndex].structMotor.unCommOK_CNT++;
	}
	else
	{
		MTR_tMotor[pMotorComm->unMotorIndex].structMotor.unCommErrCNT++;
	}

}

// If some read commands were sent out and there is more read command in the queue,
// dummy command will be sent out to the motors which have been sent to with read command.
// By this way we can optimize that only one dummy command will be sent out to each motor before write (analyze) it.
uint16_t unMotorCommRxBuf[MAX_MOTOR_COMM_LENGTH + 1];	
void readFromMotor(MOTOR_SPI_COMM_T* pMotorComm)
{
	// read command | CRC | dummy
	static uint16_t unTrashData;
	static uint16_t unCRC;
	
	// 1. Read command
	osDelay(MTR_COMM_INTVL_MIN);
	selectMotor(pMotorComm->unMotorIndex);
	osDelay(1);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(pMotorComm->unPayLoad), (uint8_t *)(&unTrashData), 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	osDelay(1);
	deselectMotor(pMotorComm->unMotorIndex);
	
	// 2. CRC
	osDelay(MTR_COMM_INTVL_MIN);
	unCRC = calCRC16((uint8_t *)(pMotorComm->unPayLoad), 2);
	selectMotor(pMotorComm->unMotorIndex);
	osDelay(1);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(&unCRC), (uint8_t *)(&unTrashData), 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	osDelay(1);
	deselectMotor(pMotorComm->unMotorIndex);
	
	// 3. dummy for register value
	osDelay(MTR_COMM_INTVL_MIN);
	selectMotor(pMotorComm->unMotorIndex);
	osDelay(1);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(T_MOTOR_DUMMY_CMD.unPayLoad), (uint8_t *)unMotorCommRxBuf, 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	osDelay(1);
	deselectMotor(pMotorComm->unMotorIndex);	
	
		// 4. dummy for CRC
	osDelay(MTR_COMM_INTVL_MIN);
	selectMotor(pMotorComm->unMotorIndex);
	osDelay(1);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(T_MOTOR_DUMMY_CMD.unPayLoad), (uint8_t *)(unMotorCommRxBuf + 1), 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	osDelay(1);
	deselectMotor(pMotorComm->unMotorIndex);	
	
	MTR_analyzeReadData(pMotorComm, unMotorCommRxBuf);
}

void writeToMotor(MOTOR_SPI_COMM_T* pMotorComm)
{
	// write command | write data | CRC
	static uint16_t unMotorCommRxBuffer[MAX_MOTOR_COMM_LENGTH + 1];
	static uint16_t unCRC;
	
	// 1. write command
	selectMotor(pMotorComm->unMotorIndex);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(pMotorComm->unPayLoad), (uint8_t *)unMotorCommRxBuffer, 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	deselectMotor(pMotorComm->unMotorIndex);
	
	// 2. write command
	osDelay(MTR_COMM_INTVL_MIN);
	selectMotor(pMotorComm->unMotorIndex);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(pMotorComm->unPayLoad + 1), (uint8_t *)unMotorCommRxBuffer, 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	deselectMotor(pMotorComm->unMotorIndex);
	
	// 3. CRC
	osDelay(MTR_COMM_INTVL_MIN);
	unCRC = calCRC16((uint8_t *)(pMotorComm->unPayLoad), 4);
	selectMotor(pMotorComm->unMotorIndex);
	if(HAL_SPI_TransmitReceive_IT(&MOTOR_COMM_SPI_HANDLER, (uint8_t*)(&unCRC), (uint8_t *)(unMotorCommRxBuffer + 1), 1) != HAL_OK)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	if (xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, portMAX_DELAY) != pdTRUE)
	{
		M_handleErr(NOT_USED_FOR_NOW);
	}
	deselectMotor(pMotorComm->unMotorIndex);		
}

void MTR_MotorComm(void const * argument)
{
	static MOTOR_SPI_COMM_T tMotorComm = MTR_DUMMY_CMD_CONTENT;
	static MOTOR_SPI_COMM_T tMotorCommLast[MOTOR_NUMBER] = {MTR_DUMMY_CMD_CONTENT, MTR_DUMMY_CMD_CONTENT,
			MTR_DUMMY_CMD_CONTENT, MTR_DUMMY_CMD_CONTENT};
	static MOTOR_SPI_COMM_T tMotorCommTemp = MTR_DUMMY_CMD_CONTENT;
	static TickType_t unCommSinceLastTime;
	static TickType_t	unLastCommTime[MOTOR_NUMBER] = {0, 0, 0, 0};

	DESELECT_ALL_MOTOR;
	tMotorComm.unMotorIndex = 0;	//unMotorIndex;
	tMotorComm.unPayLoad[0] = COMM_READ_LOCATING_PERIOD | MTR_COMM_RW_CMD_MASK;
	for(;;)
	{	
//		if (xQueueReceive(MotorCommQueueHandle, &tMotorComm, portMAX_DELAY) != pdTRUE)
//		{
//			M_handleErr(NOT_USED_FOR_NOW);
//		}

//		if (tMotorComm.unMotorIndex >= MOTOR_NUMBER)
//		{
//			M_handleErr(NOT_USED_FOR_NOW);
//		}

		/*********************************************************/
		/* ----======== Pre send command stage start ========----*/
		// If this command belongs to a different motor and last time command is one read command
		// DUMMY/Invalid command is already considered in IS_MTR_COMM_RD_CMD macro
//		if ((tMotorComm.unMotorIndex != tMotorCommLast.unMotorIndex) && IS_MTR_COMM_RD_CMD(tMotorCommLast.unPayLoad[0]))
//		{
//			sendDummyCMDtoRead(&tMotorCommLast, unMotorCommRxBuffer);
//			tMotorCommLast.unMotorIndex = MTR_INVALID_MOTOR_INDEX;
//			tMotorCommLast.unPayLoad[0] = MTR_INVALID_MOTOR_CMD;
//			osDelay(10);
//		}
		/* ----======== Pre send command stage end ========----*/
		/*********************************************************/

		

		/*********************************************************/
		/* ----======== Send command stage start ========----*/
//		unCommSinceLastTime = (TickType_t)(xTaskGetTickCount() - unLastCommTime[tMotorComm.unMotorIndex]);
//		if (unCommSinceLastTime < MTR_COMM_INTVL_MIN)
//		{
//			osDelay(MTR_COMM_INTVL_MIN - unCommSinceLastTime);
//		}

//		if (IS_MTR_COMM_RD_CMD(tMotorComm.unPayLoad[0])) 
//		{
//			// read command
			readFromMotor(&tMotorComm);
			
//		}
//		else if (IS_MTR_COMM_WR_CMD(tMotorComm.unPayLoad[0]))
//		{
//			// write command
//			writeToMotor(&tMotorComm);
//		}
		


		/* ----======== Send command stage end ========----*/
		/*********************************************************/


		/*********************************************************/
		/* ----======== After send command stage start ========----*/
		// There is no read command anymore, send dummy command to retrieve all read data (if the previously transaction are read)
//		if (uxQueueMessagesWaiting(MotorCommQueueHandle) == 0)
//		{
//			// No communication any more,
//			retrieveAllReadData(tMotorCommLast, unLastCommTime);
//		}
//		else
//		{
//			if (xQueuePeek(MotorCommQueueHandle, &tMotorCommTemp, portMAX_DELAY) != pdTRUE)
//			{
//				M_handleErr(NOT_USED_FOR_NOW);
//			}
//			if (IS_MTR_COMM_WR_CMD(tMotorCommTemp.unPayLoad[0]) && IS_MTR_COMM_RD_CMD(tMotorComm.unPayLoad[0]))
//			{
//				retrieveAllReadData(tMotorCommLast, unLastCommTime);
//			}
//		}

		/* ----======== After send command stage end ========----*/
		/*********************************************************/
	}
}
