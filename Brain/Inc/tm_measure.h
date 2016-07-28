
#ifndef __TM_H
#define __TM_H

#include "stm32f4xx.h"
#include "global.h"

#ifdef __USED_BY_TM__
	#define __EXTERN_TM__
	
	#else
	#define __EXTERN_TM__ extern
#endif

#endif
	
