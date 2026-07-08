/*
 * fc_px4_mavlink_shell_stub.cpp — MavlinkShell STUB.
 *
 * PX4 mavlink_shell.cpp pulls posix px4_daemon/pxh.h -> <platforms/posix/apps.h> (absent on ESP-IDF)
 * and is a debug feature not needed for the GCS link, so it's excluded from the build. But mavlink_main
 * (get_shell) and mavlink_receiver reference MavlinkShell symbols, so this no-op stub links them with the
 * shell disabled (start()=-1, write/read=0). mavlink_shell.h has no vtable, so 6 method defs suffice.
 * ---
 * fc_px4_mavlink_shell_stub.cpp — MavlinkShell STUB (kukla).
 *
 * PX4 mavlink_shell.cpp posix px4_daemon/pxh.h -> <platforms/posix/apps.h> (ESP-IDF'de yok) çeker ve
 * GCS-bağlantısı için gerekmeyen bir hata-ayıklama özelliğidir, bu yüzden derlemeden çıkarıldı. Ama
 * mavlink_main (get_shell) ve mavlink_receiver MavlinkShell sembollerine başvurur, bu yüzden bu no-op
 * kukla onları shell devre-dışıyken bağlar (start()=-1, write/read=0). mavlink_shell.h'nin vtable'ı
 * yok, bu yüzden 6 metot tanımı yeter.
 */
#include "mavlink_shell.h"

MavlinkShell::~MavlinkShell() {}

int MavlinkShell::start() { return -1; }   /* no shell -> cannot start | shell yok -> başlatılamaz */

size_t MavlinkShell::write(uint8_t *buffer, size_t len) { (void)buffer; (void)len; return 0; }

size_t MavlinkShell::read(uint8_t *buffer, size_t len) { (void)buffer; (void)len; return 0; }

size_t MavlinkShell::available() { return 0; }

int MavlinkShell::shell_start_thread(int argc, char *argv[]) { (void)argc; (void)argv; return -1; }
