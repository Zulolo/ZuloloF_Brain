
#ifndef __AIR_QUALITY_H
#define __AIR_QUALITY_H

#include "stm32f4xx.h"
#include "global.h"

#ifdef __USED_BY_AIR_QUALITY__
	#define __EXTERN_AIR_QUALITY__
	#define ADC_START_DELAY_AFTER_PULSE						290
	#define ADC_VALUE_BUFF_LEN										32
	#else
	#define __EXTERN_AIR_QUALITY__ extern
#endif

__EXTERN_AIR_QUALITY__ int32_t AQ_nDataManager(uint32_t unADC_Value);
#endif
	
