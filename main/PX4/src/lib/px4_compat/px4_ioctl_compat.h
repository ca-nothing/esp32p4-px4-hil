#pragma once
/*
 * px4_ioctl_compat.h — P4 _IO/_IOC compat (force-include, sadece uORB/cdev scope).
 *
 * PX4 defines.h: _PX4_IOC(x,y) -> _IO(x,y). drv_orb_dev.h ORBIOC* komutlarini bununla
 * tanimlar. _IO/_IOC, Linux <sys/ioctl.h>'in ioctl-kodlama makrolari; ESP-IDF/newlib
 * saglamiyor. MODERN uORB ioctl-dispatch'i BYPASS eder (Subscription/Publication ->
 * DeviceNode::copy()/write() DOGRUDAN); bu makrolar yalniz ORBIOC* sabitlerinin
 * BENZERSIZ tamsayi olarak DERLENMESI icin var (calisma-zamani gercek ioctl yok).
 */
#ifndef _IOC
#define _IOC(type, nr) (((unsigned)(type) << 8) | (unsigned)(nr))
#endif
#ifndef _IO
#define _IO(type, nr) _IOC((type), (nr))
#endif

/*
 * px4_platform_common/time.h (non-lockstep #else dali): px4_usleep -> system_usleep,
 * px4_sleep -> system_sleep, px4_pthread_cond_timedwait -> system_pthread_cond_timedwait.
 * ESP-IDF/newlib usleep()/sleep()/pthread_cond_timedwait() saglar (POSIX). Lazy makro ->
 * kullanim yerinde (unistd.h/pthread.h zaten dahil) genisler.
 */
#ifndef system_usleep
#define system_usleep usleep
#endif
#ifndef system_sleep
#define system_sleep sleep
#endif
#ifndef system_pthread_cond_timedwait
#define system_pthread_cond_timedwait pthread_cond_timedwait
#endif
