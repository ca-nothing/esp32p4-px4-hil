/*
 * fc_px4_mavlink_platform.cpp — platform globals the PX4 mavlink module needs on P4 (no fake logic).
 *
 * The PX4 mavlink module expects a few platform providers that ESP-IDF/P4 doesn't supply; these fill
 * the real gaps so the PX4 source compiles untouched. None are flight logic:
 *   1) px4 firmware/os/board version -> normally src/lib/version/version.c, which #errors on an unknown
 *      OS (version.c:326/352) and pulls Linux uname()/<sys/utsname.h> (absent on ESP-IDF), so it can't
 *      build unmodified. Returns constants for the AUTOPILOT_VERSION telemetry message (firmware=v1.15.0,
 *      rest 0). [Removable once version.c gains a __PX4_ESPIDF branch.]
 *   2) dup2 -> not provided by ESP-IDF newlib (libc gap); mavlink_main.cpp:2017 task_main calls it.
 *
 * NOTE: the mavlink_system global lives in verbatim PX4 mavlink.c:49 (CMakeLists FC_MAVLINK_SRCS), not here.
 * ---
 * fc_px4_mavlink_platform.cpp — PX4 mavlink modülünün P4'te ihtiyaç duyduğu platform global'leri (sahte mantık yok).
 *
 * PX4 mavlink modülü ESP-IDF/P4'ün sağlamadığı birkaç platform sağlayıcı bekler; bunlar PX4 kaynağı
 * dokunulmadan derlensin diye gerçek boşlukları doldurur. Hiçbiri uçuş mantığı değildir:
 *   1) px4 firmware/os/board sürümü -> normalde src/lib/version/version.c, bilinmeyen bir OS'te #error
 *      verir (version.c:326/352) ve Linux uname()/<sys/utsname.h> (ESP-IDF'de yok) çeker, bu yüzden
 *      değiştirilmeden derlenemez. AUTOPILOT_VERSION telemetri mesajı için sabitler döndürür
 *      (firmware=v1.15.0, gerisi 0). [version.c bir __PX4_ESPIDF dalı kazanınca kaldırılabilir.]
 *   2) dup2 -> ESP-IDF newlib tarafından sağlanmaz (libc boşluğu); mavlink_main.cpp:2017 task_main çağırır.
 *
 * NOT: mavlink_system global'i verbatim PX4 mavlink.c:49'da yaşar (CMakeLists FC_MAVLINK_SRCS), burada değil.
 */
#include <stdint.h>

extern "C" {

/* 1) PX4 version (real version.c #errors on ESP-IDF -> platform stub; for AUTOPILOT_VERSION uid/version).
   | 1) PX4 sürümü (gerçek version.c ESP-IDF'de #error verir -> platform kuklası; AUTOPILOT_VERSION uid/sürüm için). */
uint32_t px4_firmware_version(void)            { return 0x010F0000u; }  /* v1.15.0 */
uint32_t px4_firmware_vendor_version(void)     { return 0u; }
uint32_t px4_board_version(void)               { return 0u; }
uint32_t px4_os_version(void)                  { return 0u; }
uint64_t px4_firmware_version_binary(void)     { return 0u; }
uint64_t px4_mavlink_lib_version_binary(void)  { return 0u; }
uint64_t px4_os_version_binary(void)           { return 0u; }

/* 2) libc gap: ESP-IDF newlib lacks dup2 (mavlink_main.cpp:2017 task_main calls it).
   | 2) libc boşluğu: ESP-IDF newlib'de dup2 yok (mavlink_main.cpp:2017 task_main çağırır). */
int dup2(int oldfd, int newfd) { (void)oldfd; (void)newfd; return -1; }

} /* extern "C" */
