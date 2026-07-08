#pragma once
/*
 * micro_hal.h — empty ESP32-P4 stub for parameters.cpp's include of
 * <px4_platform_common/micro_hal.h>.
 *
 * On real PX4 this holds board-specific HW macros (px4_arch_* GPIO/SPI/clock/flash),
 * but those are only needed on the FLASH_BASED_PARAMS code path, which is disabled
 * here — so an empty stub suffices. Lives in our gen-stub dir (resolved via -isystem)
 * to keep PX4/ byte-clean. Add P4 equivalents here if some code ever needs a real macro.
 * ---
 * micro_hal.h — parameters.cpp'nin <px4_platform_common/micro_hal.h> include'u icin
 * bos ESP32-P4 stub.
 *
 * Gercek PX4'te bu, board'a-ozgu HW makrolarini tutar (px4_arch_* GPIO/SPI/clock/flash),
 * ama bunlar yalnizca burada devre-disi olan FLASH_BASED_PARAMS kod-yolunda gerekir —
 * bu yuzden bos stub yeterli. PX4/'i bayt-temiz tutmak icin gen-stub dizinimizde durur
 * (-isystem ile cozulur). Bir kod gercek bir makroya ihtiyac duyarsa P4 karsiliklarini buraya ekle.
 */
