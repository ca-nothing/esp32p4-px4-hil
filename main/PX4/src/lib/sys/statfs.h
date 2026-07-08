#pragma once
/*
 * P4 platform shim — <sys/statfs.h> (POSIX). ESP-IDF/newlib BUNU SAGLAMAZ.
 *
 * TEK kullanici: commander/HealthAndArmingChecks/checks/sdcardCheck.cpp — PX4_STORAGEDIR
 * tanimliyken statfs(PX4_STORAGEDIR, &buf) ile SD'nin takili olup olmadigini (f_blocks>0)
 * sorar. ESP-IDF'te SD = FATFS (VFS mount), POSIX statfs() YOK -> fc_px4_tasks.cpp /
 * server_io.h gibi PLATFORM-KATMANI karsiligi (simplification DEGIL).
 *
 * RULE-6/7 ACIK-FARK: statfs() = -1 (desteklenmiyor) -> sdcardCheck SD'yi "yok" raporlar.
 * commander DORMANT (spawn edilmemis) oldugu icin RUNTIME ETKISI YOK. Commander aktif
 * edilince GERCEK SD-tespiti = FATFS f_getfree adapter'i ile baglanacak (TODO-activation).
 */
#include <stdint.h>

struct statfs {
	uint64_t f_bsize;   /* blok boyutu (bytes) */
	uint64_t f_blocks;  /* toplam blok */
	uint64_t f_bfree;   /* bos blok */
	uint64_t f_bavail;  /* kullanilabilir blok */
};

static inline int statfs(const char *path, struct statfs *buf)
{
	(void)path;
	(void)buf;
	return -1;   /* TODO(activation): FATFS f_getfree adapter — gercek SD bos-alan */
}
