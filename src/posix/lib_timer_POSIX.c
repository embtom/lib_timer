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

/* *******************************************************************
 * includes
 * ******************************************************************/

/* c-runtime */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <limits.h>

/* system */
#include <signal.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <time.h>

/* project*/
#include <lib_log.h>
#include "lib_convention__errno.h"
#include "lib_thread.h"
#include "lib_timer.h"


/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_LIB_TIMER_SIGMIN          SIGRTMIN
#define M_LIB_TIMER_SIGMAX          SIGRTMAX
#define M_LIB_TIMER_NUMBER_SIG      (M_LIB_TIMER_SIGMAX - M_LIB_TIMER_SIGMIN + 1)

#define M_LIB_TIMER_CLOCK_SRC       CLOCK_MONOTONIC
#define M_LIB_TIMER_UNUSED			0
#define M_LIB_TIMER_CONFIRMED		1
#define M_LIB_TIMER_REQUESTED		2

#define M_LIB_TIMER_ID              "LIB_TIMER"
#define M_LIB_MAP_INITIALZED        0xABCDABCD

/* *******************************************************************
 * custom data types (e.g. enumerations, structures, unions)
 * ******************************************************************/
struct signals_region {
    pthread_mutex_t mmap_mtx;
    unsigned int initialized;
    int used_signals_max_cnt;
    int used_signals_data;
};

struct internal_timer {
    timer_t             timer_id;	// timeout timer ID
    int                 timer_signo;	// signal index, i.e. "relative" signal number
    sigset_t            sigset;		// the signal set on which to be woken up
    enum timer_mode		timer_mode;
    int                 signal_fd;
    timer_cb_t          *callback;
    void                *callback_arg;
    struct itimerspec   pause_value;
    int 				close_requested;
    sem_hdl_t			close_sem;
};

struct timeout_distributer
{
    thread_hdl_t timeout_th;
    timer_hdl_t cancelHdl;
    int unblock_fd[2];
    int epoll_fd;
    int running;
};

/* *******************************************************************
 * Static Function Prototypes
 * ******************************************************************/
static int lib_timer__shm_create_or_open(size_t _shm_size);
static int lib_timer__shm_initialize(struct signals_region * _shm_addr, size_t _shm_size);
static int lib_timer__shm_request_signal_timer(struct signals_region *_shm_addr);
static int lib_timer__shm_confirm_signal_timer(struct signals_region *_shm_addr, int _timer_signal_number);
static int lib_timer__shm_free_signal_timer(struct signals_region *_shm_addr, int _timer_signal_number);
static int lib_timer__shm_timer_to_release(struct signals_region *_shm_addr);
static int lib_timer__init_timeout_distributor(void);
static int lib_timer__cleanup_timeout_distributor(void);
static void* lib_timer__timeout_distributor_worker(void *_arg);

/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/
static unsigned s_lib_timer_init_calls = 0;
static struct signals_region *s_shm_signals_mmap = NULL;
static ssize_t s_shm_signals_mmap_size;
static int s_shm_fd = -1;
static int *s_local_used_signals = NULL;
static struct timeout_distributer s_timeout_dist_hdl;
static unsigned int 		close_requested = 0;

/* *******************************************************************
 * Global Functions
 * ******************************************************************/

/* ************************************************************************//**
 * \brief	Init of the timer component
 *
 * The posix timmers are based on  realtime signals and the number is limited.
 * A shared memory object is used to mark the signals, which are already utilized
 * at a process.
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
	struct sigaction sa;

    int i,line, ret = EOK;
    int shm_fd;
    sigset_t sigset;
    struct signals_region *shm_signals_mmap;
    ssize_t shm_signals_mmap_size;

    if (s_shm_signals_mmap != NULL) {
        /* check whether component is not (yet) initialized (any more) */
        s_lib_timer_init_calls++;
        return EOK;
    }
    shm_signals_mmap_size = sizeof(struct signals_region) + M_LIB_TIMER_NUMBER_SIG * sizeof(int);

    ret = lib_timer__shm_create_or_open(shm_signals_mmap_size);
    if (ret < EOK) {
        line = __LINE__;
        goto ERR_SHM_OPEN_CREATE;
    }
    shm_fd = ret;

    shm_signals_mmap = (struct signals_region *)mmap(NULL,shm_signals_mmap_size,PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_signals_mmap == MAP_FAILED) {
        line = __LINE__;
        ret = -ESTD_NXIO;
        goto ERR_MEM_FAILED;
    }

    s_local_used_signals = (int*)calloc(M_LIB_TIMER_NUMBER_SIG,sizeof(int));
    if(s_local_used_signals == NULL) {
        line = __LINE__;
        ret = -ESTD_NOMEM;
        goto ERR_MEM_FAILED;
    }

    if (shm_signals_mmap->initialized == M_LIB_MAP_INITIALZED) {
        s_lib_timer_init_calls++;
        s_shm_fd = shm_fd;
        s_shm_signals_mmap = shm_signals_mmap;
        s_shm_signals_mmap_size = shm_signals_mmap_size;
        return EOK;
    }


    ret = lib_timer__shm_initialize(shm_signals_mmap, shm_signals_mmap_size);
    if(ret < EOK) {
        line = __LINE__;
        goto ERR_SHM_INITIALZE;
    }

    /*
     * Block all reserved real-time signals so they can be used for the timers.
     * This has to be done in main() before any threads are created so they all inherit the same mask.
     * Doing it later is subject to race conditions.
     */

    /* reset signal set */
    if (sigemptyset(&sigset) != 0){
        /* should actually never happen */
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_SIGSETEMTY;
    }

    /* add all reserved real-time signals to the signal set */
    for (i = M_LIB_TIMER_SIGMIN; i <= M_LIB_TIMER_SIGMAX; i++)
    {
        if (sigaddset(&sigset, i) != 0){
            /* should actually never happen */
            line = __LINE__;
            ret = convert_std_errno(errno);
            goto ERR_SIGADDSET;
        }
    }

    /* block on the configured signal set */
    ret = pthread_sigmask(SIG_BLOCK, &sigset, NULL);
    if (ret != 0){
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_SIGADDSET;
    }

    ret = lib_timer__init_timeout_distributor();
    if (ret < EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_SIGADDSET;
    }


    shm_signals_mmap->initialized = M_LIB_MAP_INITIALZED;
    s_shm_fd = shm_fd;
    s_shm_signals_mmap = shm_signals_mmap;
    s_shm_signals_mmap_size = shm_signals_mmap_size;

    /* increment initialization count */
    s_lib_timer_init_calls=1;
    return EOK;

    ERR_SIGADDSET:
    ERR_SIGSETEMTY:
        free(s_local_used_signals);
    ERR_SHM_INITIALZE:

    ERR_MEM_FAILED:
        close(shm_fd);

    ERR_SHM_OPEN_CREATE:
    s_shm_signals_mmap = NULL;
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
    int ret;
    int *used_signals;
    char shm_name[NAME_MAX]= {0};

    /* check whether component is properly initialized */
    if ((s_shm_signals_mmap == NULL) || (s_lib_timer_init_calls == 0)) {
        ret = -EEXEC_NOINIT;
        msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"");
        return ret;
    }

    /* check whether component should be unloaded */
    if (s_lib_timer_init_calls == 1) {
        int i;

        used_signals = &s_shm_signals_mmap->used_signals_data;

        pthread_mutex_lock(&s_shm_signals_mmap->mmap_mtx);
        /* check whether there are any signals left which are currently in use */
        for (i = 0; i < s_shm_signals_mmap->used_signals_max_cnt; i++) {
            if (s_local_used_signals[i] != 0) {
                pthread_mutex_unlock(&s_shm_signals_mmap->mmap_mtx);
                ret = -ESTD_BUSY;
                msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)\n",__func__ ,ret , __LINE__);
                return ret;
            }
        }
        s_lib_timer_init_calls = 0;
        pthread_mutex_unlock(&s_shm_signals_mmap->mmap_mtx);
        free(s_local_used_signals);
        munmap((void*)s_shm_signals_mmap,s_shm_signals_mmap_size);
        s_shm_signals_mmap = NULL;
        s_shm_signals_mmap_size = 0;
        close(s_shm_fd);
        s_shm_fd = -1;
        sprintf(&shm_name[0],"/%s_%u",program_invocation_short_name,getpid());
        shm_unlink(&shm_name[0]);

        ret = lib_timer__cleanup_timeout_distributor();
        if (ret < EOK) {
             msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s() failed with error %i (line: %u)\n",__func__ ,ret , __LINE__);
        }
        return EOK;
    }

    /* decrement initialization count */
    s_lib_timer_init_calls--;
    return EOK;
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
    //struct itimerspec	itval;	/* timeout value structure */
    struct sigevent		sigev;	/* sigevent structure for storing the event type, the signal number and the value pointer */
    int timer_signo;
    int ret, line;
    timer_hdl_t         hdl;

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

    if (s_shm_signals_mmap == NULL) {
        line = __LINE__;
        ret = -EEXEC_NOINIT;
        goto ERR_0;
    }

    hdl = (timer_hdl_t)calloc(1,sizeof(struct internal_timer));
    if(hdl == NULL) {
        line = __LINE__;
        ret = -ESTD_NOMEM;
        goto ERR_0;
    }

    ret = lib_timer__shm_request_signal_timer(s_shm_signals_mmap);
    if (ret < EOK) {
        line = __LINE__;
        goto ERR_1;
    }
    timer_signo = ret;

    /* initialize sigevent structure in the wakeup object */
    sigev.sigev_notify = SIGEV_SIGNAL;
    sigev.sigev_signo = timer_signo + M_LIB_TIMER_SIGMIN;
    sigev.sigev_value.sival_ptr = hdl;


    ret = sigemptyset(&hdl->sigset);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_2;
    }

    ret = sigaddset(&hdl->sigset, sigev.sigev_signo);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_2;
    }

    if (sigaddset(&hdl->sigset, SIGUSR2) != 0){
        /* should actually never happen */
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_2;
    }

    if (_cb != NULL) {
        int signal_fd;
        struct epoll_event  epev  = {0};
        epev.events        = EPOLLIN;
        epev.data.ptr      = hdl;

        ret = sigprocmask(SIG_BLOCK, &hdl->sigset, NULL);
        if (ret != EOK) {
            line = __LINE__;
            ret = convert_std_errno(errno);
            goto ERR_2;
        }

        signal_fd = signalfd (-1, &hdl->sigset, SFD_NONBLOCK|SFD_CLOEXEC);
        if (signal_fd < EOK) {
            line = __LINE__;
            ret = convert_std_errno(errno);
            goto ERR_2;
        }
        hdl->signal_fd = signal_fd;

        ret = epoll_ctl(s_timeout_dist_hdl.epoll_fd, EPOLL_CTL_ADD, signal_fd, &epev);
        if (ret != EOK) {
            line = __LINE__;
            ret = convert_std_errno(errno);
            goto ERR_2;
        }
        hdl->timer_mode = TIMER_MODE_callback;
    }
    else {
    	hdl->timer_mode = TIMER_MODE_cyclic;
    }

    ret = timer_create(M_LIB_TIMER_CLOCK_SRC, &sigev, &hdl->timer_id);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_2;
    }

    ret = lib_timer__shm_confirm_signal_timer(s_shm_signals_mmap, timer_signo);
    if (ret != EOK) {
        line = __LINE__;
        goto ERR_3;
    }
    s_local_used_signals[timer_signo] = M_LIB_TIMER_CONFIRMED;
    hdl->timer_signo = timer_signo;
    hdl->callback_arg = _arg;
    hdl->callback = _cb;
    memset(&hdl->pause_value, 0, sizeof(struct itimerspec));
    hdl->close_requested = 0;
    close_requested = 0;

    *_hdl = hdl;
    msg (LOG_LEVEL_info, M_LIB_TIMER_ID,"Timer ID %i created\n", timer_signo);
    return EOK;

    ERR_3:
    timer_delete(hdl->timer_id);
    ERR_2:
    lib_timer__shm_free_signal_timer(s_shm_signals_mmap, timer_signo);
    ERR_1:
    free(hdl);
    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
    if (_hdl != NULL) {
        *_hdl = NULL;
    }
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
    int line, ret;
    int timer_signo, signal_fd;
    struct itimerspec	itval;	/* timeout value structure */

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

    if ((s_shm_signals_mmap == NULL) || (s_lib_timer_init_calls == 0)) {

    	line = __LINE__;
        ret = -EEXEC_NOINIT;
        goto ERR_0;
    }

    if (*_hdl == NULL) {
        line = __LINE__;
        ret = -ESTD_INVAL;
        goto ERR_0;
    }

    ret = lib_timer__shm_timer_to_release(s_shm_signals_mmap);
    if (ret < EOK) {
        line = __LINE__;
        goto ERR_0;
    }

    close_requested = 1;
    (*_hdl)->close_requested = 1;

    switch ((*_hdl)->timer_mode)
    {
   	   	case TIMER_MODE_callback:
        {
   	   		signal_fd = (*_hdl)->signal_fd;
   	    	ret = epoll_ctl(s_timeout_dist_hdl.epoll_fd, EPOLL_CTL_DEL, signal_fd, NULL);
   	    	if (ret != EOK) {
   	    		line = __LINE__;
   	        	ret = convert_std_errno(errno);
   	        	goto ERR_0;
   	    	}

   	    	ret = close((*_hdl)->signal_fd);
   	    	if (ret != EOK) {
   	        	line = __LINE__;
   	        	ret = convert_std_errno(errno);
   	        	goto ERR_0;
   	    	}
   			break;
        }
   	   	case TIMER_MODE_cyclic:
        {
   	   		lib_thread__sem_init(&((*_hdl)->close_sem),0);
   	   		itval.it_value.tv_sec		= 0;
    		itval.it_value.tv_nsec		= 100;
    		itval.it_interval.tv_sec	= 0;
    		itval.it_interval.tv_nsec	= 0;
    		timer_settime((*_hdl)->timer_id, 0, &itval, NULL);
    		lib_thread__sem_wait((*_hdl)->close_sem);
    		break;
        }
        case TIMER_MODE_shutdown:
        default:
        {
            line = __LINE__;
            ret = -ESTD_FAULT;
            goto ERR_0;
        }
   }

   ret = timer_delete((*_hdl)->timer_id);
   if (ret != EOK) {
	   line = __LINE__;
	   ret = convert_std_errno(errno);
	   goto ERR_0;
   }

   timer_signo = (*_hdl)->timer_signo;
   ret = lib_timer__shm_free_signal_timer(s_shm_signals_mmap, timer_signo);
   if (ret < EOK) {
        line = __LINE__;
        goto ERR_0;
   }
   s_local_used_signals[timer_signo] = M_LIB_TIMER_UNUSED;
 //  free(*_hdl);
   return EOK;

   ERR_0:
   msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
   if (_hdl != NULL) {
       *_hdl = NULL;
   }
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
    int line, ret;
    struct itimerspec	itval;	/* timeout value structure */

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

    if (s_shm_signals_mmap == NULL) {
        line = __LINE__;
        ret = -EEXEC_NOINIT;
        goto ERR_0;
    }

    switch (_hdl->timer_mode)
    {
    	case TIMER_MODE_callback:
    		/* set timer with the specified periodic timeout, firing immediately for the first time */
    		itval.it_value.tv_sec		= (int)_tmoms / 1000;
    		itval.it_value.tv_nsec		= ((int)_tmoms % 1000) * 1000000;
    		itval.it_interval.tv_sec	= 0;
    		itval.it_interval.tv_nsec	= 0;
    		break;
    	case TIMER_MODE_cyclic:
    		itval.it_value.tv_sec		= (int)_tmoms / 1000;
    		itval.it_value.tv_nsec		= ((int)_tmoms % 1000) * 1000000;
    		itval.it_interval.tv_sec	= (int)_tmoms / 1000;
    		itval.it_interval.tv_nsec	= ((int)_tmoms % 1000) * 1000000;
    		break;
    	default: {
    		line = __LINE__;
    		ret = -ESTD_FAULT;
    		goto ERR_0;
    	}
    }

    ret = timer_settime(_hdl->timer_id, 0, &itval, NULL);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_0;
    }

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
    struct itimerspec zero_timer = { 0 };

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

    if (s_shm_signals_mmap == NULL) {
        line = __LINE__;
        ret = -EEXEC_NOINIT;
        goto ERR_0;
    }

    ret = timer_settime(_hdl->timer_id, 0, &zero_timer, &_hdl->pause_value);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_0;
    }

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

    if (s_shm_signals_mmap == NULL) {
        line = __LINE__;
        ret = -EEXEC_NOINIT;
        goto ERR_0;
    }

    ret = timer_settime(_hdl->timer_id, 0, &_hdl->pause_value, NULL);
    if (ret != EOK) {
        line = __LINE__;
        ret = convert_std_errno(errno);
        goto ERR_0;
    }

    memset(&_hdl->pause_value, 0, sizeof(struct itimerspec));
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
    int line, ret;
    int sig;

    if (_hdl == NULL) {
        line = __LINE__;
        ret = -EPAR_NULL;
        goto ERR_0;
    }

    if(_hdl->timer_mode != TIMER_MODE_cyclic) {
    	line = __LINE__;
    	ret = -ESTD_FAULT;
    	goto ERR_0;
    }

    ret = sigwait(&(_hdl->sigset),&sig);
    if (ret != EOK) {
    	msg(LOG_LEVEL_error, M_LIB_TIMER_ID,"%s(): sigwait failed with retval %i\n",__func__, ret);
    }
    if(_hdl->close_requested) {
    	lib_thread__sem_post(_hdl->close_sem);
    	_hdl->close_requested = 0;
    	return -ESTD_INTR;
    }
    //printf("SIG %u\n",close_requested);
    return EOK;

    ERR_0:
    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
    return ret;
}

/* *******************************************************************
 * Static Functions
 * ******************************************************************/
static int lib_timer__shm_create_or_open(size_t _shm_size)
{
    int ret, shm_fd;
    char shm_name[NAME_MAX]= {0};
    int shm_oflag = 0;
    int shm_open_loop = 0;

    /* creation of a IPC object:									*/
    /*	O_CREAT : 	Create the object if not exits					*/
    /* 	O_EXCL	:	If object already exits exit with EEXIST		*/
    /*	O_RDWR	: 	Read write access 								*/
    sprintf(&shm_name[0],"/%s_%u",program_invocation_short_name,getpid());

    do {

        shm_fd = shm_open(&shm_name[0], shm_oflag | O_RDWR, S_IRUSR | S_IWUSR);
        shm_oflag = O_CREAT | O_EXCL;
        if (shm_fd < 0) {
            ret = -errno;
        }

        if ((ret == EOK) && (shm_open_loop > 0)) {
            ret = ftruncate(shm_fd, _shm_size);
            if(ret != EOK) {
                close(shm_fd);
                ret = -EEXEC_FAILINIT;
            }
        }
        shm_open_loop++;
        ret = EOK;
    } while ((shm_fd < 0)&&(shm_open_loop < 2));

    if (ret < EOK) {
        msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() error shm_open with %i",__func__,ret);
        return ret;
    }

    return shm_fd;
}

static int lib_timer__shm_initialize(struct signals_region * _shm_addr, size_t _shm_size)
{
    pthread_mutexattr_t mutex_attr;
    if (_shm_addr == NULL) {
        return -EPAR_NULL;
    }

    memset((void*)_shm_addr,0,_shm_size);
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&_shm_addr->mmap_mtx, &mutex_attr);
    _shm_addr->used_signals_max_cnt = M_LIB_TIMER_NUMBER_SIG;
    return EOK;
}

static int lib_timer__shm_request_signal_timer(struct signals_region *_shm_addr)
{
    int i, ret;
    int *used_signals;

    if (_shm_addr == NULL) {
        return -EPAR_NULL;
    }

    ret = pthread_mutex_lock(&_shm_addr->mmap_mtx);
    if (ret != EOK) {
        ret = convert_std_errno(errno);
        return ret;
    }

    used_signals = &_shm_addr->used_signals_data;
    for (i = 0; i < _shm_addr->used_signals_max_cnt; i++){
        if (used_signals[i] == M_LIB_TIMER_UNUSED){
            /* free slot found -> set signal as used and break loop */
            break;
        }
    } msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );

    if(i >= _shm_addr->used_signals_max_cnt) {
        /* maximum number of concurrently manageable signals has been reached -> return error */
        pthread_mutex_unlock(&_shm_addr->mmap_mtx);
        return -ESTD_AGAIN;
    }

    used_signals[i] = M_LIB_TIMER_REQUESTED;
    pthread_mutex_unlock(&_shm_addr->mmap_mtx);
    return i;
}

static int lib_timer__shm_confirm_signal_timer(struct signals_region *_shm_addr, int _timer_signal_number)
{
    int *used_signals;

    if (_shm_addr == NULL) {
        return -EPAR_NULL;
    }

    if (_shm_addr->used_signals_max_cnt <= _timer_signal_number) {
        return -ESTD_INVAL;
    }

    used_signals = &_shm_addr->used_signals_data;
    used_signals[_timer_signal_number] = M_LIB_TIMER_CONFIRMED;
    return EOK;
}

static int lib_timer__shm_free_signal_timer(struct signals_region *_shm_addr, int _timer_signal_number)
{
    int i, ret;
    int *used_signals;

    if (_shm_addr == NULL) {
        return -EPAR_NULL;
    }

    if (_shm_addr->used_signals_max_cnt <= _timer_signal_number) {
        return -ESTD_INVAL;
    }

    used_signals = &_shm_addr->used_signals_data;
    used_signals[_timer_signal_number] = M_LIB_TIMER_UNUSED;
    return EOK;
}

static int lib_timer__shm_timer_to_release(struct signals_region *_shm_addr)
{
    int i, ret;
    int *used_signals;

    ret = pthread_mutex_lock(&_shm_addr->mmap_mtx);
    if (ret != EOK) {
        ret = convert_std_errno(errno);
        return ret;
    }

    used_signals = &_shm_addr->used_signals_data;
    /* check for a free slot in the signal list */
    for (i = 0; i < _shm_addr->used_signals_max_cnt; i++){
        if (used_signals[i] != 0){
            /* used slot found -> break */
            break;
        }
    }

    if (i >= _shm_addr->used_signals_max_cnt) {
        /* No signal currently in use.
         * This error can actually only happen if a destroy call is executed on an actually invalid wakeup object
         * which is not detected as such. This is possible if the wakeup object has never been used before.
         */
        pthread_mutex_unlock(&_shm_addr->mmap_mtx);
        return  -ESTD_INVAL;
    }

    pthread_mutex_unlock(&_shm_addr->mmap_mtx);
    return i;
}

static int lib_timer__init_timeout_distributor(void)
{
    int ret;
    struct epoll_event epev = {0};

    ret = pipe(s_timeout_dist_hdl.unblock_fd);
    if (ret < EOK) {
        ret = convert_std_errno(errno);
        goto ERR_0;
    }

    timer_hdl_t cancelHdl = (timer_hdl_t)calloc(1,sizeof(struct internal_timer));
    if (cancelHdl == NULL) {
        ret = -EPAR_NULL;
        goto ERR_1;
    }

    cancelHdl->timer_mode = TIMER_MODE_shutdown;

    ret = epoll_create1(EPOLL_CLOEXEC);
    if (ret < EOK) {
        ret = convert_std_errno(errno);
        goto ERR_2;
    }
    s_timeout_dist_hdl.epoll_fd = ret;

    epev.events        = EPOLLIN;
    epev.data.ptr      = cancelHdl;
    ret = epoll_ctl(s_timeout_dist_hdl.epoll_fd, EPOLL_CTL_ADD, s_timeout_dist_hdl.unblock_fd[0], &epev);
    if (ret != EOK) {
        ret = convert_std_errno(errno);
        goto ERR_3;
    }

    s_timeout_dist_hdl.cancelHdl = cancelHdl;

    ret = lib_thread__create(&s_timeout_dist_hdl.timeout_th, &lib_timer__timeout_distributor_worker,&s_timeout_dist_hdl,0,"timeout_distributer");
    if (ret < EOK) {
        return ret;
    }

    return EOK;

    ERR_3:
    close(s_timeout_dist_hdl.epoll_fd);

    ERR_2:
    free(cancelHdl);

    ERR_1:
    close(s_timeout_dist_hdl.unblock_fd[0]);
    close(s_timeout_dist_hdl.unblock_fd[1]);

    ERR_0:
    return ret;

}

struct thread_hdl_attr {
    pthread_t thread_hdl;
    char *thread_name;
    size_t thread_name_len;
};

static int lib_timer__cleanup_timeout_distributor(void)
{
    int ret;
    int dummy = 1;

    s_timeout_dist_hdl.running = 0;
    write(s_timeout_dist_hdl.unblock_fd[1], &dummy, sizeof(dummy));

    ret = lib_thread__join(&s_timeout_dist_hdl.timeout_th, NULL);
    if (ret < EOK) {
        return ret;
    }

    free(s_timeout_dist_hdl.cancelHdl);

    close(s_timeout_dist_hdl.unblock_fd[0]);
    close(s_timeout_dist_hdl.unblock_fd[1]);
    close(s_timeout_dist_hdl.epoll_fd);

    return EOK;
}

static void* lib_timer__timeout_distributor_worker(void *_arg)
{
    int i, ret;
    ssize_t res;
    timer_hdl_t timer_hdl;
    struct signalfd_siginfo si;
    struct epoll_event      epev;
    struct timeout_distributer *timeout_dist_hdl = (struct timeout_distributer*)_arg;

    s_timeout_dist_hdl.running = 1;
    while (s_timeout_dist_hdl.running)
    {
        do {
            ret = epoll_wait(timeout_dist_hdl->epoll_fd, &epev, 1, -1);
            if ((ret < 0) && (errno != EINTR))  {
                 ret = convert_std_errno(errno);
                 break;
            }
            if(s_timeout_dist_hdl.running == 0) {
                ret = -EEXEC_CLEANUP;
                break;
            }
        }
        while ((ret < 0) && (errno == EINTR));

        if (ret < EOK) {
            switch(ret) {
                case -ESTD_BADF:
                    msg (LOG_LEVEL_info, M_LIB_TIMER_ID, "%s(): will shutdown \n", __func__);
                    s_timeout_dist_hdl.running = 0;
                    return NULL;
                case -EEXEC_CLEANUP:
                    msg (LOG_LEVEL_info, M_LIB_TIMER_ID, "%s(): interrupted by EINTR \n", __func__);
                    s_timeout_dist_hdl.running = 0;
                    return NULL;
                break;
                default:
                    msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): epoll error occured with %i thread terminated\n", __func__, ret);
                break;
            }
        }


        timer_hdl = (timer_hdl_t)epev.data.ptr;
        res = read (timer_hdl->signal_fd, &si, sizeof(si));

        if (res < 0) {
            msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed to read\n", __func__);
            continue;
        }

        if (res != sizeof(si)) {
            msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): size not fits\n", __func__);
            continue;
        }

        if (timer_hdl->callback != NULL){
            (*timer_hdl->callback)(timer_hdl, timer_hdl->callback_arg);
        }
    }
    return NULL;
}
