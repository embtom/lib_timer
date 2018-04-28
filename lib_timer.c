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

/* project*/
#include "lib_convention__errno.h"
#include "lib_thread.h"
#include "lib_log.h"
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
    timer_t			timer_id;	// timeout timer ID
    int				timer_signo;	// signal index, i.e. "relative" signal number
    sigset_t		sigset;		// the signal set on which to be woken up
    int             signal_fd;
    timer_cb_t      *callback;
};

struct timeout_dist_attr
{
    sigset_t timeout_sigset;
    thread_hdl_t timeout_th;
    int epoll_fd;
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
static void* lib_timer__timeout_distributor_worker(void *_arg);



/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/
static unsigned s_lib_timer_init_calls = 0;

static struct signals_region *s_shm_signals_mmap = NULL;
static ssize_t s_shm_signals_mmap_size;
static int s_shm_fd = -1;

static int *s_local_used_signals = NULL;
static struct timeout_dist_attr s_timeout_dist_hdl;

/* *******************************************************************
 * Global Functions
 * ******************************************************************/

/* *******************************************************************
 * \brief	Initialization of the lib_timer
 * ---------
 * \remark  Initilisation of the lib_timer
 *
 * ---------
 * \return	'0', if successful, < '0' if not successful
 * ******************************************************************/
int lib_timer__init(void)
{
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

    s_local_used_signals = calloc(M_LIB_TIMER_NUMBER_SIG,sizeof(int));
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

    sigemptyset(&s_timeout_dist_hdl.timeout_sigset);
    s_lib_timer_init_calls=1;
    return EOK;

    ERR_SIGADDSET:
    ERR_SIGSETEMTY:
        free(s_local_used_signals);
    ERR_SHM_INITIALZE:

    ERR_MEM_FAILED:
        close(shm_fd);

    ERR_SHM_OPEN_CREATE:
        msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"%s() failed with error %i (line: %u)",__func__ ,ret , line);
        return ret;
}

/* *******************************************************************
 * \brief	Cleanup of the lib_timer
 * ---------
 * \remark  Initilisation of the lib_timer
 *
 * ---------
 * \return	'0', if successful, < '0' if not successful
 * ******************************************************************/
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
                ret = -EBUSY;
                msg (LOG_LEVEL_error, M_LIB_TIMER_ID,"Wakeup");
                return ret;
            }
        }
        s_lib_timer_init_calls = 0;
        pthread_mutex_unlock(&s_shm_signals_mmap->mmap_mtx);
        free(s_local_used_signals);
        munmap((void*)s_shm_signals_mmap,s_shm_signals_mmap_size);
        s_shm_signals_mmap_size = 0;
        close(s_shm_fd);
        s_shm_fd = -1;
        sprintf(&shm_name[0],"/%s_%u",program_invocation_short_name,getpid());
        shm_unlink(&shm_name[0]);
        return EOK;
    }

    /* decrement initialization count */
    s_lib_timer_init_calls--;
    return EOK;
}

int lib_timer__open(timer_hdl_t *_hdl, void *_arg, timer_cb_t *_cb)
{
    struct itimerspec	itval;	/* timeout value structure */
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

    hdl = calloc(1,sizeof(struct internal_timer));
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

    if (_cb != NULL) {
        int signal_fd;
        struct epoll_event      epev;
        memset (&epev, 0, sizeof(struct epoll_event));
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
    hdl->callback = _cb;
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

int lib_timer__close(timer_hdl_t *_hdl)
{
    int line, ret;
    int timer_signo, signal_fd;

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

   ret = timer_delete((*_hdl)->timer_id);
   if (ret != EOK) {
       line = __LINE__;
       ret = convert_std_errno(errno);
       goto ERR_0;
   }

   if ((*_hdl)->callback != NULL)
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
   }

   timer_signo = (*_hdl)->timer_signo;
   ret = lib_timer__shm_free_signal_timer(s_shm_signals_mmap, timer_signo);
   if (ret < EOK) {
        line = __LINE__;
        goto ERR_0;
   }
   s_local_used_signals[timer_signo] = M_LIB_TIMER_UNUSED;
   free(*_hdl);
   return EOK;

   ERR_0:
   msg (LOG_LEVEL_error, M_LIB_TIMER_ID, "%s(): failed with retval %i\n",__func__, ret );
   if (_hdl != NULL) {
       *_hdl = NULL;
   }
   return ret;
}

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

    /* set timer with the specified periodic timeout, firing immediately for the first time */
    itval.it_value.tv_sec		= (int)_tmoms / 1000;
    itval.it_value.tv_nsec		= ((int)_tmoms % 1000) * 1000000;
    itval.it_interval.tv_sec	= 0;
    itval.it_interval.tv_nsec	= 0;

//    itval.it_value.tv_sec		= 0;
//    itval.it_value.tv_nsec		= 1;
//    itval.it_interval.tv_sec	= (int)_tmoms / 1000;
//    itval.it_interval.tv_nsec	= ((int)_tmoms % 1000) * 1000000;


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

int lib_timer__stop(timer_hdl_t _hdl)
{
    return EOK;
}

int lib_timer__reset(timer_hdl_t _hdl)
{
    return EOK;
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
    }

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

    ret = epoll_create(1);
    if (ret < EOK) {
        ret = convert_std_errno(errno);
        return ret;
    }

    s_timeout_dist_hdl.epoll_fd = ret;

    ret = lib_thread__create(&s_timeout_dist_hdl.timeout_th, &lib_timer__timeout_distributor_worker,&s_timeout_dist_hdl,0,"timeout_distributer");
    if (ret < EOK) {
        return ret;
    }

    return EOK;
}

static void* lib_timer__timeout_distributor_worker(void *_arg)
{
    int ret;
    ssize_t res;
    timer_hdl_t timer_hdl;
    struct signalfd_siginfo si;
    struct epoll_event      epev;
    struct timeout_dist_attr *timeout_dist_hdl = (struct timeout_dist_attr*)_arg;

    while (1)
    {
        do {
            ret = epoll_wait(timeout_dist_hdl->epoll_fd, &epev, 1, -1);
        }
        while ((ret < 0) && (errno == EINTR));

        timer_hdl = epev.data.ptr;
        res = read (timer_hdl->signal_fd, &si, sizeof(si));

        if (res < 0) {
            perror ("read");
            continue;
        }

        if (res != sizeof(si)) {
            fprintf (stderr, "Something wrong\n");
            continue;
        }


        //printf("%s() called\n",__func__);
        if (timer_hdl->callback != NULL){
            (*timer_hdl->callback)(timer_hdl, NULL);
        }

    }
}
