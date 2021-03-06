/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "global.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myDEMO_LED_TaskHandle;
osThreadId myMotorCtrlTaskHandle;
osThreadId myRoutineUpdateHandle;
osThreadId myMotorCommTaskHandle;
osThreadId myNRF905CommHandle;
osThreadId myTmptHmdtMsmtHandle;
osThreadId myAirQualityHandle;
osMessageQId MotorCommQueueHandle;
osSemaphoreId MTR_tMotorSPI_CommCpltHandle;
osSemaphoreId RTN_tNeedToUpdateMotorHandle;
osSemaphoreId MTR_tMotorSpeedChangedHandle;
osSemaphoreId WL_tNRF905SPI_CommCpltHandle;
osSemaphoreId TH_tMeasureData_CommCpltHandle;
osSemaphoreId GLB_tGlobalParaAccessHandle;
osSemaphoreId AQ_tADC_Cnvt_CpltHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
extern void blinkDemoLED(void const * argument);
extern void MTR_ctrlMotor(void const * argument);
extern void RTN_updateMotor(void const * argument);
extern void MTR_MotorComm(void const * argument);
extern void WL_startRFComm(void const * argument);
extern void TH_Measure(void const * argument);
extern void AQ_Measure(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of MTR_tMotorSPI_CommCplt */
  osSemaphoreDef(MTR_tMotorSPI_CommCplt);
  MTR_tMotorSPI_CommCpltHandle = osSemaphoreCreate(osSemaphore(MTR_tMotorSPI_CommCplt), 1);

  /* definition and creation of RTN_tNeedToUpdateMotor */
  osSemaphoreDef(RTN_tNeedToUpdateMotor);
  RTN_tNeedToUpdateMotorHandle = osSemaphoreCreate(osSemaphore(RTN_tNeedToUpdateMotor), 1);

  /* definition and creation of MTR_tMotorSpeedChanged */
  osSemaphoreDef(MTR_tMotorSpeedChanged);
  MTR_tMotorSpeedChangedHandle = osSemaphoreCreate(osSemaphore(MTR_tMotorSpeedChanged), 1);

  /* definition and creation of WL_tNRF905SPI_CommCplt */
  osSemaphoreDef(WL_tNRF905SPI_CommCplt);
  WL_tNRF905SPI_CommCpltHandle = osSemaphoreCreate(osSemaphore(WL_tNRF905SPI_CommCplt), 1);

  /* definition and creation of TH_tMeasureData_CommCplt */
  osSemaphoreDef(TH_tMeasureData_CommCplt);
  TH_tMeasureData_CommCpltHandle = osSemaphoreCreate(osSemaphore(TH_tMeasureData_CommCplt), 1);

  /* definition and creation of GLB_tGlobalParaAccess */
  osSemaphoreDef(GLB_tGlobalParaAccess);
  GLB_tGlobalParaAccessHandle = osSemaphoreCreate(osSemaphore(GLB_tGlobalParaAccess), 1);

  /* definition and creation of AQ_tADC_Cnvt_Cplt */
  osSemaphoreDef(AQ_tADC_Cnvt_Cplt);
  AQ_tADC_Cnvt_CpltHandle = osSemaphoreCreate(osSemaphore(AQ_tADC_Cnvt_Cplt), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	osSemaphoreWait(MTR_tMotorSPI_CommCpltHandle, 5);
	osSemaphoreWait(RTN_tNeedToUpdateMotorHandle, 5);
	osSemaphoreWait(MTR_tMotorSpeedChangedHandle, 5);
	osSemaphoreWait(WL_tNRF905SPI_CommCpltHandle, 5);
	osSemaphoreWait(TH_tMeasureData_CommCpltHandle, 5);
	osSemaphoreWait(AQ_tADC_Cnvt_CpltHandle, 5);

	xSemaphoreTake(MTR_tMotorSPI_CommCpltHandle, 0);
	xSemaphoreTake(RTN_tNeedToUpdateMotorHandle, 0);
	xSemaphoreTake(MTR_tMotorSpeedChangedHandle, 0);
	xSemaphoreTake(WL_tNRF905SPI_CommCpltHandle, 0);
	xSemaphoreTake(TH_tMeasureData_CommCpltHandle, 0);
	xSemaphoreTake(AQ_tADC_Cnvt_CpltHandle, 0);	
	
	xSemaphoreGive(GLB_tGlobalParaAccessHandle);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myDEMO_LED_Task */
  osThreadDef(myDEMO_LED_Task, blinkDemoLED, osPriorityLow, 0, 256);
  myDEMO_LED_TaskHandle = osThreadCreate(osThread(myDEMO_LED_Task), NULL);

  /* definition and creation of myMotorCtrlTask */
  osThreadDef(myMotorCtrlTask, MTR_ctrlMotor, osPriorityHigh, 0, 256);
  myMotorCtrlTaskHandle = osThreadCreate(osThread(myMotorCtrlTask), NULL);

  /* definition and creation of myRoutineUpdate */
  osThreadDef(myRoutineUpdate, RTN_updateMotor, osPriorityNormal, 0, 256);
  myRoutineUpdateHandle = osThreadCreate(osThread(myRoutineUpdate), NULL);

  /* definition and creation of myMotorCommTask */
  osThreadDef(myMotorCommTask, MTR_MotorComm, osPriorityAboveNormal, 0, 256);
  myMotorCommTaskHandle = osThreadCreate(osThread(myMotorCommTask), NULL);

  /* definition and creation of myNRF905Comm */
  osThreadDef(myNRF905Comm, WL_startRFComm, osPriorityIdle, 0, 256);
  myNRF905CommHandle = osThreadCreate(osThread(myNRF905Comm), NULL);

  /* definition and creation of myTmptHmdtMsmt */
  osThreadDef(myTmptHmdtMsmt, TH_Measure, osPriorityIdle, 0, 256);
  myTmptHmdtMsmtHandle = osThreadCreate(osThread(myTmptHmdtMsmt), NULL);

  /* definition and creation of myAirQuality */
  osThreadDef(myAirQuality, AQ_Measure, osPriorityIdle, 0, 256);
  myAirQualityHandle = osThreadCreate(osThread(myAirQuality), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of MotorCommQueue */
  osMessageQDef(MotorCommQueue, 64, MOTOR_SPI_COMM_T);
  MotorCommQueueHandle = osMessageCreate(osMessageQ(MotorCommQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
