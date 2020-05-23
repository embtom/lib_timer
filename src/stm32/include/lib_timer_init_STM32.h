/*
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/embtom)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _LIB_TIMER_INIT_STM32_H_
#define _LIB_TIMER_INIT_STM32_H_

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
 * Init of the STM32 timer variant. The microcontoller hareware timers are
 * utilized.
 *
 * \return	EOK				Success
 * 			-ESTD_NXIO		Failure at signal memory map
 *			-ESTD_NOMEM		Not enough memory available
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-ESTD_EBUSY		There are still some wakeup objects currently in use
 * ****************************************************************************/
int lib_timer__init(struct timer_device_cfg const * const _cfg_map, unsigned int _cfg_cnt);

#endif /* _LIB_TIMER_INIT_STM32_H_ */
