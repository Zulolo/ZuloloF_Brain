
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"

#define __USED_BY_RF__
#include "global.h"
#include "rf.h"
#include "motor.h"
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

#define DISABLE_NRF905_DR_PIN_EXTI							(EXTI->IMR & (~(NRF905_DR_Pin)))
#define CLEAR_NRF905_DR_PIN_EXTI_PENDING				(EXTI->PR | (NRF905_DR_Pin))
#define ENABLE_NRF905_STATUS_PIN_EXTI(unPin)		(EXTI->IMR | (unPin))

void waitPinCleaning(void)
{
	HAL_TIM_Base_Stop_IT(&NRF905_COMM_TIMEOUT_HANDLER);
	DISABLE_NRF905_DR_PIN_EXTI;
	if (__HAL_TIM_GET_FLAG(&NRF905_COMM_TIMEOUT_HANDLER, TIM_FLAG_UPDATE)){
		__HAL_TIM_CLEAR_FLAG(&NRF905_COMM_TIMEOUT_HANDLER, TIM_FLAG_UPDATE);
	}
	if (__HAL_GPIO_EXTI_GET_FLAG(NRF905_DR_Pin)){
		CLEAR_NRF905_DR_PIN_EXTI_PENDING;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static portBASE_TYPE tHigherPriorityTaskWoken;
  if (NRF905_DR_Pin == GPIO_Pin)
  {
		waitPinCleaning();
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
	waitPinCleaning();
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, 0);
	
	if (HAL_GPIO_ReadPin(pGPIO_Port, unGPIO_Pin) == GPIO_PIN_SET){
		return PIN_CHANGE_ALREADY_MATCH;
	}
	
	__HAL_TIM_SET_COUNTER(&NRF905_COMM_TIMEOUT_HANDLER, 0);
	__HAL_TIM_SET_AUTORELOAD(&NRF905_COMM_TIMEOUT_HANDLER, unWL_Timeout10Us);
	ENABLE_NRF905_STATUS_PIN_EXTI(unGPIO_Pin);
	HAL_TIM_Base_Start_IT(&NRF905_COMM_TIMEOUT_HANDLER);

	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	
	waitPinCleaning();
	
	if (HAL_GPIO_ReadPin(pGPIO_Port, unGPIO_Pin) == GPIO_PIN_SET){
		return PIN_CHANGE_OK;
	}else{
		return PIN_CHANGE_TIMEOUT;
	}
}

static int32_t nGetAddrFromCH_NO(uint16_t unChannelNumber, uint8_t* pRX_Address)
{
	pRX_Address[0] = (uint8_t)(unChannelNumber & 0x03);		// 0x00
	pRX_Address[1] = (uint8_t)(unChannelNumber & 0x0D);		// 0x0C
	pRX_Address[2] = (uint8_t)(unChannelNumber & 0x50);		// 0x40
	pRX_Address[3] = (uint8_t)(unChannelNumber & 0xAA);		// 0x08
	return 0;
}

int32_t nRF905_SPI_WR(uint8_t unCMD, const uint8_t* pData, uint16_t unFrameLength)
{
	static uint8_t unRF905_SPI_TX_Frame[NRF905_TX_PAYLOAD_LEN + 1];
	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		return (-1);
	}
	setNRF905Mode(NRF905_MODE_STD_BY);
	unRF905_SPI_TX_Frame[0] = unCMD;
	memcpy(unRF905_SPI_TX_Frame + 1, pData, unFrameLength);
	RESET_NRF905_SPI_CS_PIN;
	HAL_SPI_Transmit_DMA(&NRF905_COMM_SPI_HANDLER, unRF905_SPI_TX_Frame, unFrameLength + 1);
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
	if (HAL_SPI_TransmitReceive_DMA(&NRF905_COMM_SPI_HANDLER, unRF905_SPI_TX_Frame, 
		unRF905_SPI_RX_Frame, unFrameLength + 1) != HAL_OK){
		return NULL;
	}
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	SET_NRF905_SPI_CS_PIN;
	
	return unRF905_SPI_RX_Frame;
}

int32_t nRF905SendFrame(const uint8_t* pData, uint16_t unFrameLength)
{
	if (unFrameLength > NRF905_RX_PAYLOAD_LEN){
		return (-1);
	}
	
	setNRF905Mode(NRF905_MODE_STD_BY);
	nRF905_SPI_WR(NRF905_CMD_WTP, pData, unFrameLength);
	setNRF905Mode(NRF905_MODE_BURST_TX);
//	osDelay(5);
	if (WL_WaitPinRiseWithTimeout(NRF905_DR_GPIO_Port, NRF905_DR_Pin, NRF905_MAX_DR_WAIT_TIME) == PIN_CHANGE_TIMEOUT){
		return (-1);
	}
	setNRF905Mode(NRF905_MODE_STD_BY);
	return 0;
}

static int32_t nRF905CR_Initial(void)
{
	uint8_t* pRXwStatus;

	setNRF905Mode(NRF905_MODE_STD_BY);
	
	// Since although set SPI as Mode 0 (CLK Polarity is Low), the start status of CLK pin may be high
	// Send one any/dummy data to no one (nRF905 CS NOT selected) to reset CLK pin
	HAL_SPI_Transmit_DMA(&NRF905_COMM_SPI_HANDLER, (uint8_t *)NRF905_CR_DEFAULT, 2);
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, portMAX_DELAY);
	
	nRF905_SPI_WR(NRF905_CMD_WC(0), NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT));
	osDelay(2);
	nRF905_SPI_WR(NRF905_CMD_WC(0), NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT));

	pRXwStatus = nRF905_SPI_RD(NRF905_CMD_RC(0), ARRAY_SIZE(NRF905_CR_DEFAULT));

	if (NULL == pRXwStatus){
		return (-1);
	}
	if (memcmp(pRXwStatus + 1, NRF905_CR_DEFAULT, ARRAY_SIZE(NRF905_CR_DEFAULT)) != 0){
		return (-1);
	}
	nGetAddrFromCH_NO(NRF905_CR_DEFAULT[0] | ((uint16_t)(NRF905_CR_DEFAULT[1] & 0x01) << 8),
		(uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	if (nRF905_SPI_WR(NRF905_CMD_WTA, (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		return (-1);
	}
	return 0;
}

uint8_t* pWirelessDataManager(uint8_t* pRxPayload, uint8_t unPayloadLength) 
{
	static uint8_t unDataNeedToResponse[NRF905_TX_PAYLOAD_LEN] = {0, };
	switch(*(pRxPayload + 1)){
		case RF_READ_SENSOR_VALUE:
			unDataNeedToResponse[0] = RF_READ_SENSOR_VALUE;
			IS_GLOBAL_PARA_ACCESSABLE;
			memcpy(unDataNeedToResponse + 1, &tSensoreData, sizeof(tSensoreData));
			RELEASE_GLOBAL_PARA_ACCESS;
			return unDataNeedToResponse;
		case RF_WRITE_MOTOR_PAR:
			switch(*(pRxPayload + 2)){
				case MOTOR_WRITE_MOTOR_NEED_TO_RUN:
					return NULL;
				default:
					return NULL;				
			}			
		default:
			return (pRxPayload + 1);
	}
	
	return NULL;
}

int32_t nRF905ChnPwrManager(uint16_t unFrqPwr)
{
	unFrqPwr = NRF905_CMD_CC(unFrqPwr);
	if (nRF905_SPI_WR(unFrqPwr >> 8, (uint8_t*)(&unFrqPwr), 1) < 0){
		return (-1);
	}
	tRemoteControlMap.unNRF905ChNoAndPwr = unFrqPwr;
	nGetAddrFromCH_NO(unFrqPwr & CH_MSK_IN_CC_REG, (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)));
	if (nRF905_SPI_WR(NRF905_CMD_WC(NRF905_RX_ADDRESS_IN_CR), (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_RX_ADDR_LEN) < 0){
		return (-1);
	}

	if (nRF905_SPI_WR(NRF905_CMD_WTA, (uint8_t*)(&(tRemoteControlMap.unNRF905RX_Address)), NRF905_TX_ADDR_LEN) < 0){
		return (-1);
	}

	return 0;
}

void WL_startRFComm(void const * argument)
{
	static nRF905State_t tNRF905State = NRF905_STATE_STDBY;
	static uint8_t unHoppingTableIndex = 0;
	uint8_t* pRxPayload;
	uint8_t* pTxPayload;

	DISABLE_NRF905_DR_PIN_EXTI;
	waitPinCleaning();
	HAL_TIM_OnePulse_Init(&NRF905_COMM_TIMEOUT_HANDLER, TIM_OPMODE_SINGLE);
	if (nRF905CR_Initial() < 0){
		M_handleErr(NULL);
		vTaskSuspend(NULL);
	}		
	for (;;){
		switch(tNRF905State){
		case NRF905_STATE_STDBY:		
			tNRF905State = NRF905_STATE_DR;
			break;

		case NRF905_STATE_HOPPING:
			if (unHoppingTableIndex < (ARRAY_SIZE(unRF_HOPPING_TABLE) - 1)){
				unHoppingTableIndex++;
			}else{
				unHoppingTableIndex = 0;
			}
			if (nRF905ChnPwrManager(unRF_HOPPING_TABLE[unHoppingTableIndex]) < 0){
			 tNRF905State = NRF905_STATE_END;
			}else{
				tRemoteControlMap.unNRF905HoppingNumer++;
				tNRF905State = NRF905_STATE_DR;
			}
			break;
			
		case NRF905_STATE_DR:
			setNRF905Mode(NRF905_MODE_BURST_RX);
			if (WL_WaitPinRiseWithTimeout(NRF905_DR_GPIO_Port, NRF905_DR_Pin, NRF905_MAX_DR_WAIT_TIME) == PIN_CHANGE_TIMEOUT){
				tRemoteControlMap.unNRF905CommRecvFrameErr++;
				tRemoteControlMap.unNRF905CommRecvFrameErrTotal++;
				tNRF905State = NRF905_STATE_HOPPING;
				HAL_GPIO_TogglePin(GPIO_DEMO_LED_PORT, DEMO_LED_Pin);
			}else{
				// Read data from nRF905
				pRxPayload = nRF905_SPI_RD(NRF905_CMD_RRP, NRF905_RX_PAYLOAD_LEN);
				if (NULL == pRxPayload){
					tNRF905State = NRF905_STATE_END;
				}else{
					// Data manage		
					pTxPayload = pWirelessDataManager(pRxPayload, NRF905_RX_PAYLOAD_LEN);
					if (NULL == pTxPayload){
						tNRF905State = NRF905_STATE_END;
					}else{
						// Ready to response
						if (nRF905SendFrame(pTxPayload, NRF905_RX_PAYLOAD_LEN) < 0 ){
							tNRF905State = NRF905_STATE_END;
						}else{
							tNRF905State = NRF905_STATE_DR;	
							tRemoteControlMap.unNRF905CommRecvFrameErr = 0;
							tRemoteControlMap.unNRF905CommRecvFrameOK++;
						}							
					}	
				}
			}
			break;
			
		case NRF905_STATE_END:
			M_handleErr(NULL);
			vTaskSuspend(NULL);
			break;
		default:

			break;
		}
	}
}

