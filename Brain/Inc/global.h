
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "stm32f4xx.h"

#define GPIO_DEMO_LED_PORT 								GPIOB
#define MOTOR_SPEED_ADC				  					ADC3
#define MOTOR_SPEED_ADC_HANDLER						hadc3
#define MOTOR_COMM_SPI_HANDLER						hspi1
#define NRF905_COMM_SPI_HANDLER						hspi3
#define ADC_ROUTINE_TIMER_HANDLER					htim2
#define TH_RCV_DATA_TIMER_HANDLER					htim3
#define MOTOR_ROUTINE_TIMER_HANDLER				htim6
#define NRF905_COMM_TIMEOUT_HANDLER				htim13		// 10us per CNT
#define ADC_12BIT_MASK				  					0x0FFF
#define MOTOR_NUMBER											4
#define BIT_MASK													0x00000001
#define BIT_NUM_PER_BYTE									8
#define IS_GLOBAL_PARA_WRITABLE						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY);\
																						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY);\
																						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY);\
																						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY);\
																						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY)
#define IS_GLOBAL_PARA_READABLE						xSemaphoreTake(tGlobalParaAccessHandle, portMAX_DELAY)
#define RELEASE_GLOBAL_PARA_READ_ACCESS		xSemaphoreGive(tGlobalParaAccessHandle)
#define RELEASE_GLOBAL_PARA_WRITE_ACCESS	xSemaphoreGive(tGlobalParaAccessHandle);\
																						xSemaphoreGive(tGlobalParaAccessHandle);\
																						xSemaphoreGive(tGlobalParaAccessHandle);\
																						xSemaphoreGive(tGlobalParaAccessHandle);\
																						xSemaphoreGive(tGlobalParaAccessHandle)

typedef struct
{
	__IO uint16_t	unNULL;
	struct
	{
		__IO uint16_t bMotorNeedToRun:1;
		__IO uint16_t bRotateDirection:1;
	}MCR;
	struct
	{
		__IO uint16_t bMotorPowerOn:1;
		__IO uint16_t bZeroCrossDetecting:1;
		__IO uint16_t bLocked:1;
		__IO uint16_t bThisPhaseDetectedZX:1;
		__IO uint16_t bNewComFrameReceived:1;
	}MSR;
	__IO uint16_t	unMissedZXD_CNT;
	__IO uint16_t	unSuccessZXD_CNT;
	__IO uint16_t unLocatingDuty;			/*!<  PWM Locating Duty  */
	__IO uint16_t unRampUpDuty;				/*!<  PWM Ramp Up Start Duty  */
	__IO uint16_t unTargetDuty;				/*!<  PWM Target (Locked State) Duty  */
	__IO uint16_t unActualDuty;				/*!<  PWM Actual Duty  */
	__IO uint16_t unLocatingPeriod;		/*!<  Locating State One Phase Period  */
	__IO uint16_t unSpeedADC;					/*!<  ADC read data, used to determine Target Duty */
	__IO uint16_t unReserved1;					/*!<  For 4 bytes alignment */
	__IO uint32_t unRampUpPeriod;			/*!<  Ramp Up Start One Phase Period  */
	__IO uint32_t unActualPeriod;			/*!<  Actual One Phase Period  */
	__IO uint32_t unPhaseChangeCNT;		/*!<  Phase changed counter  */
	__IO uint16_t unRPM;							/*!<  Actual RPM  */
	__IO uint16_t unBattery;					/*!<  Battery Voltage  */
	__IO uint16_t unCurrent;					/*!<  Current  */
	__IO uint16_t unReserved2;					/*!<  For 4 bytes alignment  */
	__IO uint32_t unCommOK_CNT;				/*!<  Communication OK number */
	__IO uint32_t unCommErrCNT;				/*!<  Communication error number */
} MOTOR_T;

typedef union
{
	uint16_t unValue[sizeof(MOTOR_T)/sizeof(uint16_t)];
	MOTOR_T structMotor;
} MOTOR_UNION_T;

typedef struct
{
	__IO uint16_t  unMotorIndex;
	__IO uint16_t  unPayLoad[3];
} MOTOR_SPI_COMM_T;

typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
} BOOLEAN_T;

extern osSemaphoreId tGlobalParaAccessHandle;
#endif



