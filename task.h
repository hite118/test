/* ---------------------------------------------------------------
 * Copyright(C) 2018, BLUTEK Co., LTD. All Right Reserved.
 * ---------------------------------------------------------------
 * Author     : Yulsang Kim (E-mail: yulsang@blutek.co.kr,
 *                                   yulsang@gmail.com)
 * Filename   : task.h
 * Created on : 2018. 10. 11.
 * Description:
 * ---------------------------------------------------------------
 */

#ifndef __TASK_H__
#define __TASK_H__
#define Verson "0.0.1"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <sys/mman.h>



#ifndef __SHORT_NAME_TYPES__
typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned long  ulong;
typedef uint8_t        u8,  U8;
typedef uint16_t       u16, U16;
typedef uint32_t       u32, U32;
typedef int8_t         s8,  S8;
typedef int16_t        s16, S16;
typedef int32_t        s32, S32;
typedef float          r32, R32;
typedef double         r64, R64;
typedef float          f32, F32;
typedef double         f64, F64;
#endif /*__SHORT_NAME_TYPES__ */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)  (sizeof(x)/sizeof(x[0]))
#endif
#ifndef LOBYTE
#define LOBYTE(x) ((u8)(x&0xff))
#endif
#ifndef HIBYTE
#define HIBYTE(x) ((u8)((x>>8)&0xff))
#endif

#ifndef __IO
#define __IO volatile
#endif

#ifndef __init
#define __init __attribute__((constructor));
#endif

#ifndef __exit
#define __exit __attribute__((destructor));
#endif

#ifndef __naked
#define __naked __attribute__ ((naked))
#endif

#ifndef __weak
#define __weak  __attribute__ ((weak))
#endif

/*
 * task(thread) functions
 */

typedef void (*callback)(sigval_t sig);

typedef struct __task__ {
	pthread_t handle;
	char      name[256];
	int       quit;
	char      cmdline[1024];
	void *    arg;
	callback  event;
} task_t;

/*Update(2024-08-26) �ŷڼ��˻� : task -> task->arg ������ */
static inline int task_create(task_t *task, void *(*func) (void *), void *arg)
{
	int err;

	assert(task);
	assert(func);

	task->quit = 0;
	task->arg = arg;

	err = pthread_create(&task->handle, NULL, func, task->arg);
	if (err != 0)
	{
		printf("task_create() error!\n");
	}

	return err;
}

#if 0
static inline int task_delete(task_t *task)
{
	if (task->handle != 0UL)
	{
		task->quit = 1;
		pthread_join(task->handle, NULL);
	}
	return 0;
}
#endif

/*Update(2024-08-26) �ŷڼ��˻� : task_exit �Լ��� ������ ����  */
/*
static inline int task_exit(task_t *task)
{
	if (task->handle != 0UL)
	{
		task->quit = 1;
	}
	return 0;
}
*/

/*Update(2024-08-26) �ŷڼ��˻� : task_event �Լ��� ������ ����  */
/*Update(2024-08-26) �ŷڼ��˻� : identifier sigval�� ���� scope�� �����ϴ� ������ identifier�� ���� signal -> signal_value */
/*
static inline void task_event(task_t *task, int sig)
{
	sigval_t signal_value;

	assert(task);

	if (task->event != NULL)
	{
		signal_value.sival_int = sig;
		task->event(signal_value);
	}
}
*/

/*Update(2024-08-26) �ŷڼ��˻� : task_event_ptr �Լ��� ������ ����  */
/*Update(2024-08-26) �ŷڼ��˻� : identifier sigval�� ���� scope�� �����ϴ� ������ identifier�� ���� signal -> signal_value */
/*
static inline void task_event_ptr(task_t *task, void *sig)
{
	sigval_t signal_value;

	assert(task);

	if (task->event != NULL)
	{
		signal_value.sival_ptr = sig;
		task->event(signal_value);
	}
}
*/

static inline bool task_is_running(task_t *task)
{
	assert(task);
	return (task->quit == 0) ? true : false;
}

#if 0
static inline bool task_is_quit(task_t *task)
{
	assert(task);
	return (task->quit == 0) ? false : true;
}

static inline void *task_argument(task_t *task)
{
	assert(task);
	return task->arg;
}
#endif
/*
 * timer function
 * - callback: void timer_callback (sigval_t) { ... }
 */

/*Update(2024-08-26) �ŷڼ��˻� : identifier callback�� ���� scope�� �����ϴ� ������ identifier�� ���� callback -> p_callback */
/*Update(2024-08-26) �ŷڼ��˻� : timer_create2 �Լ��� ������ ���� */
/*
static inline int timer_create2(timer_t *handle, void *p_callback,
		void *arg, int sec, unsigned int nsec)
{
	struct itimerspec it;
	struct sigevent sev;

	int err = -1;

	memset(&sev, 0x00, sizeof(sev));

	sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_notify_function = p_callback;
	sev.sigev_value.sival_ptr = arg;

	it.it_interval.tv_sec = sec;
	it.it_interval.tv_nsec= nsec;
	it.it_value = it.it_interval;

	if (timer_create(CLOCK_REALTIME, &sev, handle) != 0)
	{
		printf("timer_create error!\n");
	}
	else if (timer_settime(*handle, 0, &it, NULL) != 0)
	{
		printf("timer_settime error!\n");
	}
	else
	{
		err = 0;
	}

	return err;
}
*/

/*
 * utility function
 * - void timer_callback (sigval_t)
 */

int  getch(void);
int  prompt(char *s);

void enter_kbnb(void);
void leave_kbnb(void);

int  hex2asc(char *dst, int dlen, unsigned char *src, int slen);
void hexline(void *p, int len);
void hexdump(void *p, int len, int line);

void* mmap2(off_t base, int size);
void  mmap2_free_all(void);

#endif /* __TASK_H__ */
