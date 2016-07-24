
#ifndef __TM_H
#define __TM_H

#include "stm32f4xx.h"

typedef struct
{
	struct
	{
		__IO uint16_t unNULL:16;
	}SCR;												// sensor control bits
	struct
	{
		__IO uint16_t unNULL:16;
	}SSR;												// sensor status bits
	uint16_t	unHumidity;				// Need to divided by 10 to get real one
	int16_t		nTemperature;			// Need to divided by 10 to get real one
} SENSOR_DATA_T;

#ifdef __USED_BY_TM__
	#define __EXTERN_TM__
	
	#else
	#define __EXTERN_TM__ extern
#endif

__EXTERN_TM__ SENSOR_DATA_T tSensoreData;
#endif
	
