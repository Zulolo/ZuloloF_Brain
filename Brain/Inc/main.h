
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __USED_BY_MAIN__
	#define __EXTERN_MAIN__

	const char NOT_USED_FOR_NOW[] = "Not used for now";
	extern ADC_HandleTypeDef hadc3;
	extern DMA_HandleTypeDef hdma_adc3;
#else
	#define __EXTERN_MAIN__ extern
	extern char NOT_USED_FOR_NOW[];
#endif

	__EXTERN_MAIN__ ErrorStatus tErrorStatus;
	__EXTERN_MAIN__ uint16_t unMotorSpeedADC_Buf[MOTOR_SPEED_ADC_DMA_DEPTH]; // the result and DMA is half word
	__EXTERN_MAIN__ void M_handleErr(char* pLog);

#endif


