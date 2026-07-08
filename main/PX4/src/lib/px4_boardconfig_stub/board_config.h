#pragma once
/*
 * board_config.h — P4 yeni-board STUB.
 *
 * Mekanizma: px4_boardconfig.h / micro_hal.h stub'lari ile AYNI — #include cozulsun.
 *
 * PX4'te board_config.h her board'un pin/HW/guc tanimlaridir (her board'da AYRI:
 * boards/px4/sitl/src/board_config.h, boards/px4/fmu-v5/... gibi). px4_config.h'in
 * __PX4_POSIX dali (satir 52) bunu #include eder.
 *
 * CDev/uORB board tanimi KULLANMAZ (0 board-makro). P4 yeni bir board -> kendi
 * board_config.h'i olur; moduller ihtiyac duydukca P4 pin/HW tanimlari EKLENIR.
 * Simdilik bos (kirpma degil — yeni-board dosyasi). Moduller ihtiyac duydukca P4
 * board tanimlari ASAGIYA eklenir (PX4 board_config.h'i neyi tanimliyorsa, P4 karsiligi).
 */

/* ── overload-LED indikatoru (Commander.cpp:2749/2753 — CPU asiri-yuk gostergesi).
 *    PX4 board'lari board_autoled_toggle()'a baglar; P4 board'unda AYRI overload-LED
 *    YOK -> no-op makro (kozmetik gosterge; commander DORMANT). RULE-6/7 acik-fark:
 *    TODO-activation: P4 status-LED'ine (drv_led) baglanabilir. */
#define BOARD_OVERLOAD_LED_TOGGLE() do {} while (0)
#define BOARD_OVERLOAD_LED_OFF()    do {} while (0)

/* ── PX4_STORAGEDIR kok yolu (defines.h __PX4_POSIX branch satir 100:
 *    PX4_ROOTFSDIR = CONFIG_BOARD_ROOT_PATH). PX4'te SD-storage semantigi; P4'te
 *    param=NVS, SD=FATFS, sdcardCheck statfs-stub'i yolu yoksayar. RULE-6/7 acik-fark:
 *    TODO-activation: gercek P4 storage mount (LittleFS/SD). */
#ifndef CONFIG_BOARD_ROOT_PATH
#define CONFIG_BOARD_ROOT_PATH "/sdcard"
#endif
