#pragma once
/*
 * <poll.h> — P4 shim. ESP-IDF/newlib <sys/poll.h>'i saglar (pollfd/POLLIN/poll())
 * ama POSIX'in <poll.h> ust-sarmalayicisini sunmaz. uORBDeviceMaster.cpp <poll.h>
 * #include eder -> gercek <sys/poll.h>'e yonlendir (icerik aynen).
 */
#include <sys/poll.h>
