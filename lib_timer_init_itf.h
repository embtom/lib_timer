/* ****************************************************************************************************
 * lib_ser_init_itf.h within the following project: bld_device_cmake_Nucleo_STM32F401
 *	
 *  compiler:   GNU Tools ARM Embedded (4.7.201xqx)
 *  target:     Cortex Mx
 *  author:		thomas
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	Jun 7, 2018			thomas			- creation of lib_ser_init_itf.h
 *  
 */



#ifndef INT_OS_BASIC_TIMER_SH_LIB_TIMER_STM32_LIB_TIMER_INIT_ITF_H_
#define INT_OS_BASIC_TIMER_SH_LIB_TIMER_STM32_LIB_TIMER_INIT_ITF_H_

/* *******************************************************************
 * includes
 * ******************************************************************/

/* system */
#ifdef CORTEX_M3
	#include <stm32f1xx.h>
	#include <stm32f1xx_hal_rcc.h>		// RCC_* functions
	#include <stm32f1xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f1xx_hal_tim.h>		// TIM_* functions
#elif CORTEX_M4
	#include <stm32f4xx.h>
	#include <stm32f4xx_hal_rcc.h>		// RCC_* functions
	#include <stm32f4xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f4xx_hal_tim.h>		// TIM_* functions
#else
	#error No Architecture is set at lib_ser
#endif


/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_SER_DEVICE_CFG_TIM(_tim) 			\
{											\
	.tim_device = _tim, 					\
	.tim_isr_type = _tim##_IRQn				\
}

#define M_SER_DEVICE_CFG_TIM_ISR(_tim, _isr) 	\
{												\
	.tim_device = _tim, 						\
	.tim_isr_type = _isr						\
}


/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

struct timer_device_cfg {
	TIM_TypeDef const * const tim_device;
	const IRQn_Type tim_isr_type;
};



/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/


/* *******************************************************************
 * static function declarations
 * ******************************************************************/

#endif /* INT_OS_BASIC_TIMER_SH_LIB_TIMER_STM32_LIB_TIMER_INIT_ITF_H_ */
