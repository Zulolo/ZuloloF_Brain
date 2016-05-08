
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __USED_BY_MOTOR__
  #define __EXTERN_MOTOR__
#else
  #define __EXTERN_MOTOR__ extern
#endif

#define MTR_SPD_CHNG_SEM_MAX		10

__EXTERN_MOTOR__ QueueHandle_t MTR_tMotorSpeedChangedSemaphore;
//__EXTERN_MOTOR__ void MTR_giveMotorSpeedADC_Sem(struct __DMA_HandleTypeDef * hdma);
__EXTERN_MOTOR__ void MTR_calculateMotorSpeedADC(void);
__EXTERN_MOTOR__ void MTR_ctrlMotor(void const * argument);

#endif


