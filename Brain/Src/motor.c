
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
#include "motor.h"
#include "main.h"

void MTR_giveMotorSpeedADC_Sem(struct __DMA_HandleTypeDef * hdma)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(MTR_tMotorSpeedChangedSemaphore, &xHigherPriorityTaskWoken);
  if(xHigherPriorityTaskWoken == pdTRUE)
  {
	  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }
}

void MTR_calculateMotorSpeedADC(void)
{
  uint8_t unIndex;
  uint32_t unADC_Avg = 0;
  for (unIndex = 0; unIndex < MOTOR_SPEED_ADC_DMA_DEPTH; unIndex++)
  {
    unADC_Avg += unMotorSpeedADC_Buf[unIndex];
  }
  unMotorSpeedADC_Avg = unADC_Avg / MOTOR_SPEED_ADC_DMA_DEPTH;
}

void MTR_ctrlMotor(void const * argument)
{
  for(;;)
  {
    xSemaphoreTake(MTR_tMotorSpeedChangedSemaphore, portMAX_DELAY);
    MTR_calculateMotorSpeedADC();
  }
}
