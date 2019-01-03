/*
 * This file is part of the EMBTOM project
 * Copyright (c) 2018-2019 Thomas Willetal 
 * (https://github.com/tom3333)
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

#ifndef _LIB_TIMER_H_
#define _LIB_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* *******************************************************************
 * includes
 * ******************************************************************/

/* project*/
#include "lib_timer_types.h"

/* *******************************************************************
 * global functions declaration
 * ******************************************************************/

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
int lib_timer__cleanup(void);

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
int lib_timer__open(timer_hdl_t *_hdl, void *_arg, timer_cb_t *_cb);

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
int lib_timer__close(timer_hdl_t *_hdl);

/* ************************************************************************//**
 * \brief	Start of the timer
 *
 * \param	_hdl		 : handle of the timer object
 * \param   _tmoms		 : timer interval in ms (must not be 0)
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 * ****************************************************************************/
int lib_timer__start(timer_hdl_t _hdl, unsigned int _tmoms);

/* ************************************************************************//**
 * \brief	Stop of the timer
 *
 * \param	_hdl		 : handle of the timer object
 * \return	'0', if successful, < '0' if not successful
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _hdl
 * ****************************************************************************/
int lib_timer__stop(timer_hdl_t _hdl);

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
int lib_timer__resume(timer_hdl_t _hdl);

/* ************************************************************************//**
 * \brief	Wait on the specified cyclic timer object
 *
 * This function waits until the specified timer object's timeout interval
 * has elapsed. Since this is not synchronized with the actual function call,
 * the call may unblock virtually instantly, particularly when being executed
 * for the first time. Hence this function should be called at the top of
 * a while or for loop.
 *
 * If the wakeup object will be destroyed the wakeup wait routine unblocks
 * with -ESTD_INTR
 *
 *
 * \param	_hdl [in/out]	handle of the timer object
 * \return	EOK				Success
 *			-EEXEC_NOINIT	Component not (yet) initialized (any more)
 *			-EPAR_NULL		NULL pointer specified for _wu_obj
 *			-ESTD_INTR		The cyclic timer object is destroyed
 * ****************************************************************************/
int lib_timer__wakeup_wait(timer_hdl_t _hdl);

#ifdef __cplusplus
}
#endif

#endif /* _LIB_TIMER_H_ */
