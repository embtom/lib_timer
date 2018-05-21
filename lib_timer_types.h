/* ****************************************************************************************************
 * lib_timer_types.h within the following project: bld_device_cmake_LINUX
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



#ifndef LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_TYPES_H_
#define LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_TYPES_H_

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

typedef struct internal_timer* timer_hdl_t;
typedef void (timer_cb_t)(timer_hdl_t _hdl, void* _arg);

#ifdef __cplusplus
}
#endif

#endif /* INT_OS_BASIC_LIB_TIMER_SH_LIB_TIMER_POSIX_LIB_TIMER_H_ */
