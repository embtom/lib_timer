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
#ifndef LIB_TIMER_INIT_POSIX_H_
#define LIB_TIMER_INIT_POSIX_H_
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
int lib_timer__init(void); 
#endif /* LIB_TIMER_INIT_POSIX_H_ */