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
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>

/* project*/
#include "lib_convention__errno.h"
#include "lib_thread.h"
#include "lib_log.h"
#include "lib_timer.h"


/* *******************************************************************
 * defines
 * ******************************************************************/
#define M_LIB_TIMER_SIGMIN		SIGRTMIN
#define M_LIB_TIMER_SIGMAX		SIGRTMAX
#define M_LIB_TIMER_NUMBER_SIG  (M_LIB_TIMER_SIGMAX - M_LIB_TIMER_SIGMIN + 1)

#define M_LIB_TIMER_UNUSED			0
#define M_LIB_TIMER_CONFIRMED		1
#define M_LIB_TIMER_REQUESTED		2

#define M_LIB_TIMER_ID 		"LIB_TIMER"
#define M_LIB_MAP_INITIALZED	0xABCDABCD

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
    int				sig_ind;	// signal index, i.e. "relative" signal number
    sigset_t		sigset;		// the signal set on which to be woken up
};


/* *******************************************************************
 * Static Function Prototypes
 * ******************************************************************/
static int lib_timer__shm_create_or_open(size_t _shm_size);
static int lib_timer__shm_initialize(struct signals_region * _shm_addr, size_t _shm_size);


/* *******************************************************************
 * (static) variables declarations
 * ******************************************************************/
//static const int clock_src = CLOCK_MONOTONIC;
static unsigned s_lib_timer_init_calls = 0;
static struct signals_region *s_shm_signals_mmap = NULL;
static ssize_t s_shm_signals_mmap_size;
static int s_shm_fd = -1;
static int *s_local_used_signals = NULL;

/* *******************************************************************
 * function definition
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
