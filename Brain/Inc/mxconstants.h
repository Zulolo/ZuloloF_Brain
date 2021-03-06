/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MOTOR_SPEED_ADC_DMA_DEPTH 32

#define NRF905_AM_Pin GPIO_PIN_2
#define NRF905_AM_GPIO_Port GPIOE
#define NRF905_DR_Pin GPIO_PIN_3
#define NRF905_DR_GPIO_Port GPIOE
#define NRF905_TX_EN_Pin GPIO_PIN_4
#define NRF905_TX_EN_GPIO_Port GPIOE
#define BUTTON_TEST_Pin GPIO_PIN_6
#define BUTTON_TEST_GPIO_Port GPIOE
#define ADC_MOTOR_SPEED_CTRL_Pin GPIO_PIN_0
#define ADC_MOTOR_SPEED_CTRL_GPIO_Port GPIOC
#define AIR_QLT_ADC_Pin GPIO_PIN_2
#define AIR_QLT_ADC_GPIO_Port GPIOA
#define SPI1_W25X16_CS_Pin GPIO_PIN_4
#define SPI1_W25X16_CS_GPIO_Port GPIOA
#define MOTOR_COM_SCK_Pin GPIO_PIN_5
#define MOTOR_COM_SCK_GPIO_Port GPIOA
#define MOTOR_COM_MISO_Pin GPIO_PIN_6
#define MOTOR_COM_MISO_GPIO_Port GPIOA
#define MOTOR_COM_MOSI_Pin GPIO_PIN_7
#define MOTOR_COM_MOSI_GPIO_Port GPIOA
#define SPI1_MOTOR_SELECT_1_Pin GPIO_PIN_4
#define SPI1_MOTOR_SELECT_1_GPIO_Port GPIOC
#define SPI1_MOTOR_SELECT_2_Pin GPIO_PIN_5
#define SPI1_MOTOR_SELECT_2_GPIO_Port GPIOC
#define DEMO_LED_Pin GPIO_PIN_0
#define DEMO_LED_GPIO_Port GPIOB
#define SPI1_MOTOR_SELECT_3_Pin GPIO_PIN_1
#define SPI1_MOTOR_SELECT_3_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SPI1_MOTOR_SELECT_4_Pin GPIO_PIN_7
#define SPI1_MOTOR_SELECT_4_GPIO_Port GPIOE
#define SPI1_MOTOR_SELECT_5_Pin GPIO_PIN_8
#define SPI1_MOTOR_SELECT_5_GPIO_Port GPIOE
#define SPI1_MOTOR_SELECT_6_Pin GPIO_PIN_9
#define SPI1_MOTOR_SELECT_6_GPIO_Port GPIOE
#define SPI1_MOTOR_SELECT_7_Pin GPIO_PIN_10
#define SPI1_MOTOR_SELECT_7_GPIO_Port GPIOE
#define SPI1_MOTOR_SELECT_8_Pin GPIO_PIN_11
#define SPI1_MOTOR_SELECT_8_GPIO_Port GPIOE
#define AIR_QLT_LED_ENABLE_Pin GPIO_PIN_10
#define AIR_QLT_LED_ENABLE_GPIO_Port GPIOD
#define AM2301_DATA_Pin GPIO_PIN_6
#define AM2301_DATA_GPIO_Port GPIOC
#define NRF905_CS_Pin GPIO_PIN_7
#define NRF905_CS_GPIO_Port GPIOD
#define NRF905_PWR_UP_Pin GPIO_PIN_8
#define NRF905_PWR_UP_GPIO_Port GPIOB
#define NRF905_TRX_CE_Pin GPIO_PIN_9
#define NRF905_TRX_CE_GPIO_Port GPIOB
#define NRF905_CD_Pin GPIO_PIN_1
#define NRF905_CD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
