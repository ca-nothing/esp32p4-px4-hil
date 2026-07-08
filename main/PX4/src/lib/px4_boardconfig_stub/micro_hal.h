#pragma once
/*
 * micro_hal.h — P4 yeni-board STUB.
 *
 * Mekanizma: mevcut px4_boardconfig.h stub'i ile AYNI (bkz main/CMakeLists.txt
 * "px4_boardconfig_stub" aciklamasi) — sadece #include zinciri cozulsun diye var.
 *
 * PX4'te micro_hal.h board'a-OZGU HW soyutlama makrolaridir (px4_arch_* GPIO/SPI/
 * clock...). Her PX4 board'unun KENDI micro_hal.h'i vardir (stm32f7, sitl, ...).
 * px4_config.h'in __PX4_POSIX dali (satir 51) bunu #include eder.
 *
 * CDev/uORB board-HW KULLANMAZ (dogrulama: CDev.cpp/.hpp icinde 0 board-makro).
 * Bu yuzden P4 icin simdilik bos. Bir modul gercek HW makrosu isterse bu dosyaya
 * P4 (ESP32-P4) karsiliklari EKLENIR — kirpma degil, yeni-board dosyasi, buyuyecek.
 */
