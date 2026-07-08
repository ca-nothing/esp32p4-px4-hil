#pragma once
/*
 * fc_px4_board_compat.h — board-stub for PX4 mavlink_main.cpp (force-include, mavlink TUs).
 *
 * Provides the board symbols mavlink_main.cpp normally gets from <board.h> (absent on
 * ESP32-P4), implemented P4-native: board UUID/GUID from the efuse MAC, plus CRTSCTS.
 * Platform-backend/board-stub only, not PX4 logic; PX4/ stays read-only.
 * ---
 * fc_px4_board_compat.h — PX4 mavlink_main.cpp icin board-stub (force-include, mavlink TU'lari).
 *
 * mavlink_main.cpp'nin normalde <board.h>'ten aldigi (ESP32-P4'te yok) board sembollerini
 * P4-native uygular: efuse MAC'ten board UUID/GUID, arti CRTSCTS.
 * Yalnizca platform-arka-uc/board-stub, PX4 mantigi degil; PX4/ salt-okunur kalir.
 */
#include <stdint.h>
#include <string.h>
#include <esp_mac.h>

/* ---- board UUID/GUID ---- */
#ifndef PX4_GUID_BYTE_LENGTH
#define PX4_GUID_BYTE_LENGTH 18   /* must equal sizeof(AUTOPILOT_VERSION.uid2[18]) (mavlink static_assert) | sizeof(AUTOPILOT_VERSION.uid2[18])'e esit olmali (mavlink static_assert) */
#endif
#ifndef PX4_CPU_UUID_WORD32_LENGTH
#define PX4_CPU_UUID_WORD32_LENGTH 3
#endif
#ifndef PX4_CPU_UUID_WORD32_UNIQUE_M
#define PX4_CPU_UUID_WORD32_UNIQUE_M 1   /* middle unique-word index | orta benzersiz-kelime indeksi */
#endif
#ifndef PX4_CPU_UUID_WORD32_UNIQUE_H
#define PX4_CPU_UUID_WORD32_UNIQUE_H 2   /* high unique-word index | yuksek benzersiz-kelime indeksi */
#endif

typedef uint32_t uuid_uint32_t[PX4_CPU_UUID_WORD32_LENGTH];
typedef uint8_t  px4_guid_t[PX4_GUID_BYTE_LENGTH];

/* ESP32-P4 efuse MAC (6-byte unique) -> 96-bit board UUID.
   | ESP32-P4 efuse MAC (6-bayt benzersiz) -> 96-bit board UUID. */
static inline int board_get_uuid32(uuid_uint32_t uuid_words)
{
	uint8_t mac[6] = {0};
	esp_efuse_mac_get_default(mac);
	uuid_words[0] = 0x50345f50u; /* 'P4_P' fixed signature | 'P4_P' sabit imza */
	uuid_words[1] = ((uint32_t)mac[0] << 24) | ((uint32_t)mac[1] << 16) | ((uint32_t)mac[2] << 8) | (uint32_t)mac[3];
	uuid_words[2] = ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
	return 0;
}

static inline int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t mac[6] = {0};
	esp_efuse_mac_get_default(mac);
	memset(px4_guid, 0, PX4_GUID_BYTE_LENGTH);
	memcpy(&px4_guid[PX4_GUID_BYTE_LENGTH - 6], mac, 6);
	return 0;
}

/* GUID as hex string (OpenDroneID BASIC_ID serial). PX4 signature: int(char*, int).
   | GUID'i hex dizesi olarak (OpenDroneID BASIC_ID seri-no). PX4 imzasi: int(char*, int). */
static inline int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	static const char hex[] = "0123456789abcdef";
	int offset = 0;
	for (int i = 0; i < PX4_GUID_BYTE_LENGTH && (offset + 2) < size; i++) {
		format_buffer[offset++] = hex[(px4_guid[i] >> 4) & 0x0F];
		format_buffer[offset++] = hex[px4_guid[i] & 0x0F];
	}
	if (offset < size) { format_buffer[offset] = '\0'; }
	return offset;
}

/* ---- board identity (version.h board_name/target_label/hw_version/hw_revision)
   | board kimligi (version.h board_name/target_label/hw_version/hw_revision) ---- */
#ifndef PX4_BOARD_NAME
#define PX4_BOARD_NAME "ESP32-P4-FC"
#endif
#ifndef PX4_BOARD_LABEL
#define PX4_BOARD_LABEL "default"
#endif
static inline const char *board_get_hw_type_name(void) { return "ESP32-P4"; }
static inline int board_get_hw_version(void)  { return 0; }
static inline int board_get_hw_revision(void) { return 0; }

/* ---- termios ----
 * ESP-IDF provides full termios, gated behind CONFIG_VFS_SUPPORT_TERMIOS (=y in sdkconfig),
 * so no stub here. Its only gap is CRTSCTS:
 * ---
 * ESP-IDF tam termios saglar, CONFIG_VFS_SUPPORT_TERMIOS ardinda (sdkconfig'de =y),
 * bu yuzden burada stub yok. Tek eksigi CRTSCTS: */
#ifndef CRTSCTS
#define CRTSCTS (1u << 11)   /* c_cflag HW flow-control bit; not in ESP-IDF termios, unused over UDP | c_cflag HW akis-kontrol biti; ESP-IDF termios'ta yok, UDP uzerinde kullanilmaz */
#endif
