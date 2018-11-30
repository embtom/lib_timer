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



#ifndef LIB_TIMER_INIT_STM32_H_
#define LIB_TIMER_INIT_STM32_H_

/* *******************************************************************
 * includes
 * ******************************************************************/

/* system */
#ifdef ARCH_STM32F1
	#include <stm32f1xx.h>
	#include <stm32f1xx_hal_rcc.h>		// RCC_* functions
	#include <stm32f1xx_hal_dma.h>		// Recursively required from tm32f1xx_hal_tim.h
	#include <stm32f1xx_hal_tim.h>		// TIM_* functions
#elif ARCH_STM32F4
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
#define M_LIB_TIMER__CFG_TIM(_tim) 			\
{											\
	.tim_device = _tim, 					\
	.tim_isr_type = _tim##_IRQn				\
}

#define M_LIB_TIMER__CFG_TIM_ISR(_tim, _isr) 	\
{												\
	.tim_device = _tim, 						\
	.tim_isr_type = _isr						\
}

#define M_LIB_TIMER_INIT__CFG_MAP(__cfg_map_var, __cfg_map_table)			\
	static struct timer_device_cfg __attribute__ ((section(".text"))) __cfg_map_var[] = __cfg_map_table;

#define M_LIB_TIMER_INIT__CFG_CNT(__cfg_map_var)		\
	sizeof(__cfg_map_var)/sizeof(*__cfg_map_var)

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

struct timer_device_cfg {
	TIM_TypeDef const * const tim_device;
	const IRQn_Type tim_isr_type;
};

/* *******************************************************************
 * global functions declaration
 * ******************************************************************/

/* ************************************************************************//**
 * \brief	Init of the timer component
 *
 * The RT signals are utilized and the number of it is limited. A shared
 * memory object is used to mark the signals, which are already utilized
 * at the process
 *
 * Attention:
 * At the POSIX environment have to be called at the start of the "main"
 * because the signal mask are modified
 *
 * \return	EOK				Success
 * 			-ESTD_NXIO		Failure at signal memory map
 *			-ESTD_NOMEM		Not enough memory available
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-ESTD_EBUSY		There are still some wakeup objects currently in use
 * ****************************************************************************/
int lib_timer__init(struct timer_device_cfg const * const _cfg_map, unsigned int _cfg_cnt);

#endif /* LIB_TIMER_INIT_STM32_H_ */
