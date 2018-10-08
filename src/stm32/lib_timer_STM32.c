/* ****************************************************************************************************
 * lib_timer.c within the following project: bld_device_cmake_LINUX
 *
 *  compiler:   GNU Tools ARM Embedded (4.7.201xqx)
 *  target:     Cortex Mx
 *  author:		thomas
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	19.04.2018			thomas			- creation of lib_timer.c
 *
 */

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdint.h>

/* system */
#include <FreeRTOS.h>
#include <lib_thread.h>
#include <lib_log.h>
#include <lib_isr.h>

/* frame */
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
	#error No Architecture is set at lib_clock
#endif

/* project*/
#include "lib_convention__errno.h"
#include "lib_timer_init_itf.h"
#include "lib_timer.h"


/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_LIB_TIMER_ID              "LIB_TIMER"
#define M_LIB_MAP_INITIALZED        0xABCDABCD

#define JF_TIM_TIMER		  TIM4;
#define M_LIB_TIMER_MAX_TIM_VALUE      (0xFFFF)    // 16bit counters

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

struct internal_timer {
	TIM_HandleTypeDef   tim_timer_hdl;
	lib_isr_hdl_t		isr_timer_hdl;
    enum timer_mode		timer_mode;
    void                *callback_arg;
    timer_cb_t			*callback;
    int 				close_requested;
    struct timer_device_cfg const *tim_timer_cfg;
    unsigned int 		timer_id;
    unsigned int 		ftim_freq;
    unsigned int 		last_timeout;
    unsigned int 		last_prescaler;
    unsigned int 		last_period;
};


/* *******************************************************************
 * Static Function Prototypes
 * ******************************************************************/
static inline int lib_timer__cfg_parse_and_map(timer_hdl_t _timer_hdl, struct timer_device_cfg const * const _cfg_map_addr, unsigned int _cfg_map_cnt, unsigned int *_used_addr);
static inline int lib_timer__clk_init_and_get(timer_hdl_t _timer_hdl);
static inline int lib_timer__clk_disable(timer_hdl_t _timer_hdl);
static unsigned int lib_timer__calc_prescaler(unsigned int _ftim, unsigned int _row);
static inline unsigned int lib_timer__calc_prescaler_and_period(float _timeout, unsigned int _ftim, unsigned int *_prescaler, unsigned int *_period);
static inline int lib_timer__setup_tim(timer_hdl_t _timer_hdl, unsigned int _prescaler, unsigned int _period);
static void lib_timer__isr_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg);

/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/
static struct timer_device_cfg const * s_timer_cfg_map_addr = NULL;
static unsigned int s_timer_cfg_map_cnt;
static unsigned int *s_timer_used_addr = NULL;

static unsigned int s_divider_row[] = {20, 10, 5, 2};

/* *******************************************************************
 * Global Functions
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
 *			-ESTD_EBUSY		There are still some wakeup objects currently in use
 * ****************************************************************************/
int lib_timer__init(struct timer_device_cfg const * const _cfg_map, unsigned int _cfg_cnt)
{
	int ret, line;

	if ((_cfg_map == NULL) || (_cfg_cnt == 0)) {
		line = __LINE__;
		ret = -ESTD_INVAL;
	}

	s_timer_used_addr = (unsigned int*)pvPortMalloc(_cfg_cnt * sizeof(unsigned int));
	if (s_timer_used_addr == NULL) {
		line = __LINE__;
		ret = -ESTD_NOMEM;
	}

	s_timer_cfg_map_cnt = _cfg_cnt;
	s_timer_cfg_map_addr = _cfg_map;
	return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)\n",__func__ ,ret , line);
	return ret;
}

/* ************************************************************************//**
 * \brief	Cleanup of the timer component
 *
 * The timer cleanup process fails if any timer object is currently at usage
 * (-ESTD_BUSY)
 *
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-ESTD_BUSY		There are still some timer objects currently in use
 * ****************************************************************************/
int lib_timer__cleanup(void)
{
	int line, ret;
	return EOK;

	msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)\n",__func__ ,ret , line);
	return ret;
}

/* ************************************************************************//**
 * \brief	Open a new timer object
 *
 * The timer can be used at two different operation modes, depends if _cb is set:
 *   TIMER_MODE_callback : If the adjusted timeoutt is expired a callback
 *   					   is triggered.
 *   TIMER_MODE_cyclic	 : Timer is set to a cyclic wakeup timer
 *   					   mode with the specified interval
 *
 * \param	*_hdl [out]	 : pointer to handle of the timer object
 * 						   (will be allocated; only valid on successful return)
 * \param	*_arg [in]   : pointer to a timer callback argument.
 * 						   (will be ignored if (_cb == NULL))
 * \param   *_cb [in]	 : The timer operation mode is selected if a timer
 * 						   callback is set or not
 *
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-ESTD_BUSY		There are still some timer objects currently in use
 *			-EPAR_NULL		NULL pointer specified for _hdl
 *			-ESTD_NOMEM		Insufficient memory available to initialize the wakeup object
 *			-ESTD_AGAIN		All available signals or timers are currently in use
 * ****************************************************************************/
int lib_timer__open(timer_hdl_t *_hdl, void *_arg, timer_cb_t *_cb)
{
	int i;
	int line, ret;
	unsigned int Ftim_Hz;
	timer_hdl_t hdl;

	unsigned int pclk;

	TIM_Base_InitTypeDef init_arg;

	if (s_timer_cfg_map_addr == NULL) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}

	if (_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	hdl = (timer_hdl_t)pvPortMalloc(sizeof(struct internal_timer));
	if (hdl == NULL) {
		line = __LINE__;
		ret = -ESTD_NOMEM;
		goto ERR_0;
	}

	ret = lib_timer__cfg_parse_and_map(hdl, s_timer_cfg_map_addr, s_timer_cfg_map_cnt, s_timer_used_addr);
	if (ret < EOK) {
		line = __LINE__;
		goto ERR_1;
	}
	hdl->timer_id = ret;

	ret = lib_timer__clk_init_and_get (hdl);
	if (ret < EOK) {
		line = __LINE__;
		goto ERR_2;
	}
	hdl->ftim_freq = ret;

	ret = lib_isr__attach(&hdl->isr_timer_hdl, hdl->tim_timer_cfg->tim_isr_type, &lib_timer__isr_event, (void*)hdl);
	if(ret < EOK) {
		line = __LINE__;
	 	goto ERR_2;
	}

	// clock tree: SYSCLK --AHBprescaler--> HCLK --APB1prescaler--> PCLK1 --TIM6multiplier--> to TIM 2,3,4,6,7
	// clock tim6: Input=PCLK1 (APB1 clock) (multiplied x2 in case of APB1 clock divider > 1 !!! (RCC_CFGR.PRE1[10:8].msb[10] = 1))
	hdl->callback = _cb;
	hdl->callback_arg = _arg;

	*_hdl = hdl;
	return EOK;

	ERR_2:
	s_timer_used_addr[hdl->timer_id] = 0;

	ERR_1:
	vPortFree(hdl);

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i (line %u)\n",__func__, ret, line );
	return ret;
}

/* ************************************************************************//**
 * \brief	Close of a timer object
 *
 * \param	*_hdl [out]	 : pointer to handle of the timer object
 * 						   (will be allocated; only valid on successful return)
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 *			-ESTD_INVAL		timer _hdl is not valid
 * ****************************************************************************/
int lib_timer__close(timer_hdl_t *_hdl)
{
	int ret;
	int line;

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

	if (s_timer_cfg_map_addr == NULL) {
		line = __LINE__;
		ret = -EEXEC_NOINIT;
		goto ERR_0;
	}


    if (*_hdl == NULL) {
        line = __LINE__;
        ret = -ESTD_INVAL;
        goto ERR_0;
    }

    ret = HAL_TIM_OnePulse_DeInit(&(*_hdl)->tim_timer_hdl);
	if (ret != HAL_OK) {
        line = __LINE__;
        ret = -EHAL_ERROR;
        goto ERR_0;
	}

	ret = lib_isr__detach (&(*_hdl)->isr_timer_hdl);
	if (ret < EOK) {
		line = __LINE__;
	    goto ERR_0;
	}

	ret = lib_timer__clk_disable(*_hdl);
	if (ret < EOK) {
		line = __LINE__;
	    goto ERR_0;
	}

	s_timer_used_addr[(*_hdl)->timer_id] = 0;
	vPortFree(*_hdl);

	return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i (line: %u)\n",__func__, ret, line );
	return ret;
}

/* ************************************************************************//**
 * \brief	Start of the timer
 *
 * \param	_hdl [out]	 : handle of the timer object
 * \param   _tmoms		 : timer interval in ms (must not be 0)
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 * ****************************************************************************/
int lib_timer__start(timer_hdl_t _hdl, unsigned int _tmoms)
{
	int ret;
	float error, timeout;
	unsigned int prescaler, period;

	if (_hdl == NULL) {
		goto ERR_0;
	}

	if (_hdl->last_timeout != _tmoms) {
		timeout = _tmoms / 1000.0;
		error = lib_timer__calc_prescaler_and_period(timeout, _hdl->ftim_freq, &prescaler, &period);
		_hdl->last_prescaler = prescaler;
		_hdl->last_period = period;
		_hdl->last_timeout = _tmoms;

		ret = lib_timer__setup_tim (_hdl, _hdl->last_prescaler, _hdl->last_period);
		if (ret < EOK) {
			goto ERR_0;
		}
	} else {
		__HAL_TIM_ENABLE(&_hdl->tim_timer_hdl);
	}

	_hdl->tim_timer_hdl.Instance->CNT = 0;
	return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
	return ret;
}

/* ************************************************************************//**
 * \brief	Stop of the timer
 *
 * \param	_hdl		 : handle of the timer object
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 * ****************************************************************************/
int lib_timer__stop(timer_hdl_t _hdl)
{
	int line, ret;

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

	__HAL_TIM_DISABLE(&_hdl->tim_timer_hdl);

    return EOK;

    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
    return ret;
}

/* ************************************************************************//**
 * \brief	 Resume of the previously stopped timer with the remaining time
 *
 *	Resume of the timer object with the remaining timer interval
 *
 * \param	_hdl		 : handle of the timer object
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 * ****************************************************************************/
int lib_timer__resume(timer_hdl_t _hdl)
{
	int line, ret;

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

	__HAL_TIM_ENABLE(&_hdl->tim_timer_hdl);

    return EOK;

    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
    return ret;
}

/* ************************************************************************//**
 * \brief	Wait on the specified cyclic timer object
 *
 * This function waits until the specified timer object's timeout interval
 * has elapsed. Since this is not synchronized with the actual function call,
 * the call may unblock virtually instantly, particularly when being executed
 * for the first time. Hence this function should be called at the top of
 * a while or for loop.
 *
 * If the cyclic timer object will be destroyed the wakeup wait routine unblocks
 * with -ESTD_INTR
 *
 *
 * \param	_hdl [in/out]	handle of the timer object
 * \return	EOK				Success
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _wu_obj
 *			-ESTD_INTR		The cyclic timer object is destroyed
 * ****************************************************************************/
int lib_timer__wakeup_wait(timer_hdl_t _hdl)
{
	int ret;
    return EOK;

    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
    return ret;
}


/* *******************************************************************
 * static function definitions
 * ******************************************************************/
static inline int lib_timer__cfg_parse_and_map(timer_hdl_t _timer_hdl, struct timer_device_cfg const * const _cfg_map_addr, unsigned int _cfg_map_cnt, unsigned int *_used_addr)
{
	int i;

	for (i = 0; i <_cfg_map_cnt; i++) {
		if (_used_addr[i] == 0) {
			_used_addr[i] = 1;
			break;
		}
	}

	if (i >= _cfg_map_cnt) {
		_timer_hdl->tim_timer_cfg = NULL;
		return -ESTD_NODEV;
	}else {
		_timer_hdl->tim_timer_cfg = &_cfg_map_addr[i];
		return i;
	}
}

static inline int lib_timer__clk_init_and_get(timer_hdl_t _timer_hdl)
{
	unsigned int ftim_hz;
	unsigned int pclk;

	if (_timer_hdl->tim_timer_cfg->tim_device == TIM1) {
		__HAL_RCC_TIM1_CLK_ENABLE();
		pclk = 2;
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM2) {
		__HAL_RCC_TIM2_CLK_ENABLE();
		pclk = 1;
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM3) {
		__HAL_RCC_TIM3_CLK_ENABLE();
		pclk = 1;
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM4) {
		__HAL_RCC_TIM4_CLK_ENABLE();
		pclk = 1;
	}
	else {
		return -ESTD_NODEV;
	}

	// clock tree: SYSCLK --AHBprescaler--> HCLK --APB1prescaler--> PCLK1 --TIM6multiplier--> to TIM 2,3,4,6,7
	// clock tim6: Input=PCLK1 (APB1 clock) (multiplied x2 in case of APB1 clock divider > 1 !!! (RCC_CFGR.PRE1[10:8].msb[10] = 1))
	switch (pclk)
	{
		case 1:
			ftim_hz = HAL_RCC_GetPCLK1Freq();
			if (RCC->CFGR & RCC_CFGR_PPRE1_2) {
				ftim_hz *= 2;
			}
			break;
		case 2:
			ftim_hz = HAL_RCC_GetPCLK2Freq();
			if (RCC->CFGR & RCC_CFGR_PPRE2_2) {
				ftim_hz *= 2;
			}
			break;
		default :
			return -ESTD_FAULT;
	}
	return ftim_hz;
}


static inline int lib_timer__clk_disable(timer_hdl_t _timer_hdl)
{
	if (_timer_hdl->tim_timer_cfg->tim_device == TIM1) {
		__HAL_RCC_TIM1_CLK_DISABLE();
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM2) {
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM3) {
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
	else if (_timer_hdl->tim_timer_cfg->tim_device == TIM4) {
		__HAL_RCC_TIM4_CLK_DISABLE();
	}
	else {
		return -ESTD_NODEV;
	}
	return EOK;
}


static unsigned int lib_timer__calc_prescaler(unsigned int _ftim, unsigned int _row)
{
	unsigned int prescaler = _ftim;
	unsigned int prescaler_last;
	while(prescaler > M_LIB_TIMER_MAX_TIM_VALUE) {
		prescaler_last = prescaler;
		prescaler /= _row;
	}
	return prescaler_last / _row;
}

static inline unsigned int lib_timer__calc_prescaler_and_period(float _timeout, unsigned int _ftim, unsigned int *_prescaler, unsigned int *_period)
{
	unsigned int divider_row, divider_selector = 0;
	unsigned int timer_freq;
	float calc_timeout, error;
	unsigned int find_mode = 0;

	divider_row = s_divider_row[divider_selector];
	*_prescaler = lib_timer__calc_prescaler(_ftim, divider_row);

	do {
		timer_freq = _ftim / *_prescaler;
		*_period = timer_freq * _timeout;

		if ((*_period > M_LIB_TIMER_MAX_TIM_VALUE) && !find_mode) {
			*_prescaler *= 2;
		}
		else if ((*_period > M_LIB_TIMER_MAX_TIM_VALUE) && find_mode) {
			*_prescaler += divider_row;
		}
		else if (*_prescaler > M_LIB_TIMER_MAX_TIM_VALUE) {
			divider_selector++;
			if (divider_selector > (sizeof(s_divider_row)/sizeof(unsigned int) - 1)) {
				if (find_mode) {
					*_prescaler = *_period = M_LIB_TIMER_MAX_TIM_VALUE;
					break;
				} else {
					find_mode =1;
					divider_selector=0;
				}
			}
			divider_row = s_divider_row[divider_selector];
			*_prescaler = lib_timer__calc_prescaler(_ftim, divider_row);
			*_period = M_LIB_TIMER_MAX_TIM_VALUE +1;
		}
	}while((*_period > M_LIB_TIMER_MAX_TIM_VALUE) || (*_prescaler > M_LIB_TIMER_MAX_TIM_VALUE));

	calc_timeout = (float)((*_prescaler) * (*_period))/_ftim;
	error = 1 - calc_timeout / _timeout;
//	printf("Requested timeout: %f Calc_timeout: %f Prescaler: %u Period: %u find_mode %u error: %f\n",_timeout , calc_timeout, *_prescaler, *_period, find_mode, error);
	return error;
}

static inline int lib_timer__setup_tim (timer_hdl_t _timer_hdl, unsigned int _prescaler, unsigned int _period)
{
	int ret;
	TIM_Base_InitTypeDef init_arg;	// universal temporary init structure for timer configuration

	/* setup Timer 4 for counting mode */
	/* Time Base configuration */
	init_arg.Prescaler 			= _prescaler ;		// Specifies the prescaler value used to divide the TIM clock. (0 = div by 1) This parameter can be a number between 0x0000 and 0xFFFF
	init_arg.CounterMode 		= TIM_COUNTERMODE_UP;
	init_arg.Period 			= _period & M_LIB_TIMER_MAX_TIM_VALUE;	// Auto reload register (upcounting mode => reset cnt when value is hit, and throw an overflow interrupt)
	init_arg.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;		// not available for TIM6 and 7 => will be ignored
	init_arg.RepetitionCounter 	= 0;							// start with 0 again after overflow

	_timer_hdl->tim_timer_hdl.Init = init_arg;
	_timer_hdl->tim_timer_hdl.Instance = (TIM_TypeDef*)_timer_hdl->tim_timer_cfg->tim_device;

	_timer_hdl->tim_timer_hdl.Instance->CNT = 0;
	ret = HAL_TIM_OnePulse_Init(&(_timer_hdl->tim_timer_hdl),TIM_OPMODE_SINGLE);
	if (ret != HAL_OK) {
		return -EHAL_ERROR;
	}

	__HAL_TIM_CLEAR_FLAG(&_timer_hdl->tim_timer_hdl, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
	__HAL_TIM_ENABLE_IT(&_timer_hdl->tim_timer_hdl, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&(_timer_hdl->tim_timer_hdl));
	return EOK;
}

static void lib_timer__isr_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg)
{
	timer_hdl_t timer_hdl;
	if (_arg == NULL)
		return;

	timer_hdl = (struct internal_timer*)_arg;

	  /* TIM Update event */
	if(__HAL_TIM_GET_FLAG(&timer_hdl->tim_timer_hdl, TIM_FLAG_UPDATE) != RESET) {
		if(__HAL_TIM_GET_IT_SOURCE(&timer_hdl->tim_timer_hdl, TIM_IT_UPDATE) !=RESET) {

			if (timer_hdl->callback != NULL) {
				(*timer_hdl->callback)(timer_hdl,timer_hdl->callback_arg);
			}
			__HAL_TIM_CLEAR_IT(&timer_hdl->tim_timer_hdl, TIM_IT_UPDATE);

	    }
	}

}

