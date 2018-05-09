/* ****************************************************************************************************
 * lib_timer.h within the following project: bld_device_cmake_LINUX
 *	
 *  compiler:   GNU Tools ARM Embedded (4.7.201xqx)
 *  target:     Cortex Mx
 *  author:		thomas
 * ****************************************************************************************************/

/* ****************************************************************************************************/

/*
 *	******************************* change log *******************************
 *  date			user			comment
 * 	19.04.2018			thomas			- creation of lib_timer.h
 *  
 */



#ifndef INT_OS_BASIC_LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_H_
#define INT_OS_BASIC_LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* *******************************************************************
 * includes
 * ******************************************************************/

/* *******************************************************************
 * defines
 * ******************************************************************/

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
enum timer_mode {
	TIMER_MODE_callback,
	TIMER_MODE_cyclic
};

typedef struct internal_timer *timer_hdl_t;
typedef void (timer_cb_t)(timer_hdl_t _hdl, void* _arg);
/* *******************************************************************
 * global functions declaration
 * ******************************************************************/

/* *******************************************************************
 * \brief	Initialization of the lib_timer
 * ---------
 * \remark  Initilisation of the lib_timer
 *
 * ---------
 * \return	'0', if successful, < '0' if not successful
 * ******************************************************************/
int lib_timer__init(void);

/* *******************************************************************
 * \brief	Cleanup of the lib_timer
 * ---------
 * \remark  Initilisation of the lib_timer
 *
 * ---------
 * \return	'0', if successful, < '0' if not successful
 * ******************************************************************/
int lib_timer__cleanup(void);

int lib_timer__open(timer_hdl_t *_hdl, void *_arg, timer_cb_t *_cb);

int lib_timer__close(timer_hdl_t *_hdl);

int lib_timer__start(timer_hdl_t _hdl, unsigned int _tmoms);

int lib_timer__stop(timer_hdl_t _hdl);

int lib_timer__resume(timer_hdl_t _hdl);

int lib_timer__wakeup_wait(timer_hdl_t _hdl);



#ifdef __cplusplus
}
#endif

#endif /* INT_OS_BASIC_LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_H_ */
