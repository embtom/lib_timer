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
#define M_LIB_TIMER_CFG_MAP_DESCRIPTOR						\
{														\
	M_SER_DEVICE_CFG_TIM_ISR(TIM1, NonMaskableInt_IRQn),\
	M_SER_DEVICE_CFG_TIM(TIM2),							\
	M_SER_DEVICE_CFG_TIM(TIM3),							\
	M_SER_DEVICE_CFG_TIM(TIM5)							\
}

#define M_LIB_TIMER_CFG_NMBR	sizeof(s_timer_cfg_map)/sizeof(*s_timer_cfg_map)

#define M_LIB_TIMER_ID              "LIB_TIMER"
#define M_LIB_MAP_INITIALZED        0xABCDABCD

#define JF_TIM_TIMER		  TIM4;
#define JF_MAX_TIM_VALUE      (0xFFFF)    // 16bit counters

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/

struct internal_timer {
	TIM_HandleTypeDef   tim_timer_hdl;
    enum timer_mode		timer_mode;
    void                *callback_arg;
    int 				close_requested;
    struct timer_device_cfg *tim_timer_cfg;

};


typedef int32_t   jiffy_t;    // Jiffy type 4 byte integer
typedef TIM_HandleTypeDef jf_timer_t;

typedef volatile struct {
	jf_timer_t	timer_hdl;
	jiffy_t		*value;        // Pointer to timers current value
	uint32_t    freq;          // timer's  frequency
	uint32_t    jiffies;       // jiffies max value (timer's max value)
	jiffy_t     jpus;          // Variable for the delay function
}jf_t;

/* *******************************************************************
 * Static Function Prototypes
 * ******************************************************************/
static int lib_clock__jf_init (jf_t *_jf, uint32_t _jf_freq, uint32_t _jiffies);
static int lib_clock__jf_check_usec (int32_t _usec);
static jiffy_t lib_clock__jf_per_usec (jf_t *_jf);
static int lib_clock__jf_timer_setfreq (jf_t *_jf, uint32_t _jf_freq, uint32_t _jiffies);
static void lib_clock__jf_timer_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg);

/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/
static struct timer_device_cfg __attribute__ ((section(".text"))) s_timer_cfg_map[] = M_LIB_TIMER_CFG_MAP_DESCRIPTOR;
static unsigned int s_timer_cfg_used[M_LIB_TIMER_CFG_NMBR] = {0};

static unsigned s_lib_timer_init_calls = 0;


static jf_t s_jf;
static lib_isr_hdl_t *s_jf_isr;
static unsigned int s_milliseconds_ticks;
static uint64_t s_milliseconds_ticks_64bits;
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
int lib_timer__init(void)
{
	int line, ret;
	return EOK;

    msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)",__func__ ,ret , line);
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

	msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)",__func__ ,ret , line);
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
	timer_hdl_t hdl;

	if (_hdl == NULL) {
		line = __LINE__;
		ret = -EPAR_NULL;
		goto ERR_0;
	}

	hdl = pvPortMalloc(sizeof(struct internal_timer));
	if (hdl == NULL) {
		line = __LINE__;
		ret = -ESTD_NOMEM;
		goto ERR_0;
	}

	for (i = 0; i <M_LIB_TIMER_CFG_NMBR; i++) {
		if (s_timer_cfg_used[i] == 0) {
			s_timer_cfg_used[i] = 1;
			break;
		}
	}
	hdl->tim_timer_cfg = &s_timer_cfg_map[i];

	if (hdl->tim_timer_cfg->tim_device == TIM1)
		__HAL_RCC_TIM1_CLK_ENABLE();
	else if (hdl->tim_timer_cfg->tim_device == TIM2)
		__HAL_RCC_TIM2_CLK_ENABLE();
	else if (hdl->tim_timer_cfg->tim_device == TIM3)
		__HAL_RCC_TIM3_CLK_ENABLE();
	else if (hdl->tim_timer_cfg->tim_device == TIM4)
		__HAL_RCC_TIM4_CLK_ENABLE();
	else {
		line = __LINE__;
		ret = -ESTD_NODEV;
		goto ERR_1;
	}


	*_hdl = hdl;
	return EOK;

	ERR_1:
	s_timer_cfg_used[i] = 0;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n (line %u)",__func__, ret, line );
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
	return EOK;

	ERR_0:
	msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
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
	int ret;
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
	int ret;
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
static int lib_clock__jf_init (jf_t *_jf, uint32_t _jf_freq, uint32_t _jiffies)
{
	int ret;
	ret = lib_clock__jf_timer_setfreq (_jf,_jf_freq, _jiffies);
    if (ret < EOK) {
    	return ret;
    }
    _jf->jiffies = _jiffies;
    _jf->freq = _jf_freq;
    _jf->jpus = lib_clock__jf_per_usec (_jf);
   return EOK;
}


/*!
 * \brief
 *    A code based polling version delay implementation, using jiffies for timing.
 *    This is NOT accurate but it ensures that the time passed is always
 *    more than the requested value.
 *    The delay values are multiplications of 1 usec.
 * \param
 *    usec     Time in usec for delay
 */
 static int lib_clock__jf_check_usec (int32_t _usec)
 {
   static jiffy_t m1=-1, cnt;
   jiffy_t m, m2;

   if (m1 == -1) {
      m1 = *s_jf.value;
      cnt = s_jf.jpus * _usec;
   }

   if (cnt>0) {
      m2 = *s_jf.value;
      m = m2-m1;
      cnt-= (m>0) ? m : s_jf.jiffies + m;
      m1 = m2;
      return 1;   // wait
   }
   else {
      m1 = -1;
      return 0;   // do not wait any more
   }
}

 // Return the systems best approximation for jiffies per usec
 static jiffy_t lib_clock__jf_per_usec (jf_t *_jf)
 {
    jiffy_t jf = _jf->freq / 1000000;

    if (jf <= _jf->jiffies)
       return jf;
    else
       // We can not count beyond timer's reload
       return 0;
 }

/*
 * Time base configuration using the TIM7
 * \param jf_freq  The TIMER's frequency
 * \param jiffies  The TIMER's max count value
 */
static int lib_clock__jf_timer_setfreq (jf_t *_jf, uint32_t _jf_freq, uint32_t _jiffies)
{
	TIM_Base_InitTypeDef init_arg;	// universal temporary init structure for timer configuration
	int Ftim_Hz;

	/* TIM4 clock enable */
	__HAL_RCC_TIM4_CLK_ENABLE();

		// clock tree: SYSCLK --AHBprescaler--> HCLK --APB1prescaler--> PCLK1 --TIM6multiplier--> to TIM 2,3,4,6,7
		// clock tim6: Input=PCLK1 (APB1 clock) (multiplied x2 in case of APB1 clock divider > 1 !!! (RCC_CFGR.PRE1[10:8].msb[10] = 1))
	if (RCC->CFGR & RCC_CFGR_PPRE1_2)
		Ftim_Hz = HAL_RCC_GetPCLK1Freq() * 2;
	else
		Ftim_Hz = HAL_RCC_GetPCLK2Freq();

	/* setup Timer 4 for counting mode */
	/* Time Base configuration */
	init_arg.Prescaler 			= Ftim_Hz /_jf_freq ;										// Specifies the prescaler value used to divide the TIM clock. (0 = div by 1) This parameter can be a number between 0x0000 and 0xFFFF
	init_arg.CounterMode 		= TIM_COUNTERMODE_UP;
	init_arg.Period 			= _jiffies & JF_MAX_TIM_VALUE;						// Auto reload register (upcounting mode => reset cnt when value is hit, and throw an overflow interrupt)
	init_arg.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;		// not available for TIM6 and 7 => will be ignored
	init_arg.RepetitionCounter 	= 0;							// start with 0 again after overflow

	_jf->timer_hdl.Init = init_arg;
	_jf->timer_hdl.Instance = JF_TIM_TIMER;

	HAL_TIM_Base_Init(&_jf->timer_hdl);
	_jf->value = &_jf->timer_hdl.Instance->CNT;

	__HAL_TIM_ENABLE(&_jf->timer_hdl);
	__HAL_TIM_ENABLE_IT(&_jf->timer_hdl, TIM_IT_UPDATE);

	// Timer internal prescaler
	Ftim_Hz /= ((TIM4->PSC) + 1);

	return EOK;
}

static void lib_clock__jf_timer_event(IRQn_Type _isr_vector, unsigned int _vector, void *_arg)
{
	unsigned int tick_time = 0;

	if (s_jf.freq) {
		tick_time = (s_jf.jiffies * 1000.0)/s_jf.freq;
	}
	s_milliseconds_ticks += tick_time;
	s_milliseconds_ticks_64bits += tick_time;
	 __HAL_TIM_CLEAR_FLAG(&s_jf.timer_hdl, TIM_IT_UPDATE);

}


