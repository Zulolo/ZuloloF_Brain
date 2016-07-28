
#ifndef __AIR_QUALITY_H
#define __AIR_QUALITY_H

#include "stm32f4xx.h"
#include "global.h"

#ifdef __USED_BY_AIR_QUALITY__
	#define __EXTERN_AIR_QUALITY__
	#define ADC_START_DELAY_AFTER_PULSE						280
	#else
	#define __EXTERN_AIR_QUALITY__ extern
#endif


#endif
	
