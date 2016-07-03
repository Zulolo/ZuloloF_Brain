
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
#include "string.h"

#define __USED_BY_RF__
#include "global.h"
#include "rf.h"
#include "main.h"

extern osSemaphoreId WL_tNRF905SPI_CommCpltHandle;

int32_t setNRF905Mode(nRF905Mode_t tNRF905Mode)
{
	static nRF905Mode_t tNRF905ModeGlobal = NRF905_MODE_PWR_DOWN;

	if (tNRF905Mode >= NRF905_MODE_MAX){
		return (-1);
	}
	if (tNRF905Mode == tNRF905ModeGlobal){
		return 0;
	}

	HAL_GPIO_WritePin(NRF905_TX_EN_GPIO_Port, NRF905_TX_EN_Pin, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TX_EN_PIN_POS]);
	HAL_GPIO_WritePin(NRF905_TRX_CE_GPIO_Port, NRF905_TRX_CE_Pin, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_TRX_CE_PIN_POS]);
	HAL_GPIO_WritePin(NRF905_PWR_UP_GPIO_Port, NRF905_PWR_UP_Pin, unNRF905MODE_PIN_LEVEL[tNRF905Mode][NRF905_PWR_UP_PIN_POS]);

	tNRF905ModeGlobal = tNRF905Mode;
	osDelay(2);

	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if ((GPIO_Pin == NRF905_AM_Pin) || (GPIO_Pin == NRF905_DR_Pin) || (GPIO_Pin == NRF905_CD_Pin))
  {
		tHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(WL_tNRF905SPI_CommCpltHandle, &tHigherPriorityTaskWoken);
		if(tHigherPriorityTaskWoken == pdTRUE)
		{
			portEND_SWITCHING_ISR(tHigherPriorityTaskWoken);
		}
  }
}

WAIT_PIN_CHANGE_RSLT_T WL_WaitPinRiseWithTimeout(GPIO_TypeDef* pGPIO_Port, uint16_t unGPIO_Pin, uint32_t unWL_Timeout10Us)
{
	if (HAL_GPIO_ReadPin(pGPIO_Port, unGPIO_Pin) == GPIO_PIN_SET){
		return PIN_CHANGE_ALREADY_MATCH;
	}
	__HAL_TIM_SET_AUTORELOAD(&NRF905_COMM_TIMEOUT_HANDLER, unWL_Timeout10Us);
	HAL_TIM_Base_Start_IT(&NRF905_COMM_TIMEOUT_HANDLER);

	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	if (HAL_GPIO_ReadPin(pGPIO_Port, unGPIO_Pin) == GPIO_PIN_SET){
		return PIN_CHANGE_OK;
	}else{
		return PIN_CHANGE_TIMEOUT;
	}
}

static int32_t nGetAddrFromCH_NO(uint16_t unChannelNumber, uint8_t* pRX_Address)
{
	pRX_Address[0] = (uint8_t)(unChannelNumber & 0x03);
	pRX_Address[1] = (uint8_t)(unChannelNumber & 0x0D);
	pRX_Address[2] = (uint8_t)(unChannelNumber & 0x50);
	pRX_Address[3] = (uint8_t)(unChannelNumber & 0xAA);
	return 0;
}

int32_t nRF905_SPI_WR(uint8_t unCMD, const uint8_t* pData, uint16_t unFrameLength)
{
	static uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		return (-1);
	}
	setNRF905Mode(NRF905_MODE_STD_BY);
	unRF905_SPI_TX_Frame[0] = unCMD;
	memcpy(unRF905_SPI_TX_Frame + 1, pData, unFrameLength);
	RESET_NRF905_SPI_CS_PIN;
	HAL_SPI_Transmit_IT(&NRF905_COMM_SPI_HANDLER, unRF905_SPI_TX_Frame, unFrameLength + 1);
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	SET_NRF905_SPI_CS_PIN;
	return 0;
}

uint8_t* nRF905_SPI_RD(uint8_t unCMD, int16_t unFrameLength)
{
	static uint8_t unRF905_SPI_TX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	static uint8_t unRF905_SPI_RX_Frame[NRF905_RX_PAYLOAD_LEN + 1];
	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		return NULL;
	}
	setNRF905Mode(NRF905_MODE_STD_BY);
	unRF905_SPI_TX_Frame[0] = unCMD;
	RESET_NRF905_SPI_CS_PIN;
	HAL_SPI_TransmitReceive_IT(&NRF905_COMM_SPI_HANDLER, unRF905_SPI_TX_Frame, unRF905_SPI_RX_Frame, unFrameLength + 1);
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	SET_NRF905_SPI_CS_PIN;
	
	return unRF905_SPI_RX_Frame;
}

static int32_t nRF905CR_Initial(void)
{
	uint8_t* pRXwStatus;

	nRF905_SPI_WR(NRF905_CMD_WC(0), NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT));

	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8), (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));

	pRXwStatus = nRF905_SPI_RD(NRF905_CMD_RC(0), ARRAY_SIZE(NRF905_CR_DEFAULT));

	if (memcmp(pRXwStatus + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) != 0){
		return -1;
	}
	return 0;
}

void WL_startRFComm(void const * argument)
{
	static nRF905State_t tNRF905State = NRF905_STATE_STDBY;

	HAL_TIM_OnePulse_Init(&NRF905_COMM_TIMEOUT_HANDLER, TIM_OPMODE_SINGLE);
	nRF905CR_Initial();
	for (;;){
		switch(tNRF905State){
		case NRF905_STATE_STDBY:
			setNRF905Mode(NRF905_MODE_BURST_RX);
			tNRF905State = NRF905_STATE_CD;
			break;

		case NRF905_STATE_NO_CD:
			tNRF905State = NRF905_STATE_HOPPING;
			break;

		case NRF905_STATE_HOPPING:

			break;

		case NRF905_STATE_CD:
			if (WL_WaitPinRiseWithTimeout(NRF905_CD_GPIO_Port, NRF905_CD_Pin, NRF905_SAME_FRQ_MAX_TIME) == PIN_CHANGE_TIMEOUT){
				tNRF905State = NRF905_STATE_HOPPING;
			}else{
				tNRF905State = NRF905_STATE_RXING;
			}

			break;

		default:

			break;
		}
	}
}

