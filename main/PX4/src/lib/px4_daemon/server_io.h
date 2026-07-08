#pragma once
/*
 * P4 platform shim — PX4 <px4_daemon/server_io.h> karsiligi.
 *
 * PX4'un POSIX daemon'i (nsh-shell client'a stdout pipe'lar) ESP-IDF'te YOK. px4_log.cpp
 * SADECE __PX4_POSIX altinda get_stdout() cagirir (thread-yerel stdout pipe'i icin). ESP-IDF'te
 * thread-stdout = global stdout (UART konsol; tty-pipe yok). Bu, fc_px4_tasks.cpp gibi
 * PLATFORM-KATMANI karsiligidir (simplification DEGIL; PX4'un her OS'u kendi I/O backend'ini
 * saglar — posix=daemon-pipe, nuttx=konsol, espidf=konsol).
 *
 * RULE-7 (acik-fark): gercek server_io.h sadece `__EXPORT FILE *get_stdout(bool*)` deklare eder
 * (govde px4_daemon/server_io.cpp'de = daemon). Tek uyarlama: govde = `return stdout` (inline).
 */
#include <stdio.h>

static inline FILE *get_stdout(bool *isatty_)
{
	if (isatty_) { *isatty_ = false; }   /* konsol; terminal-pipe degil */
	return stdout;
}
