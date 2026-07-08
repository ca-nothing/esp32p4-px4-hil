/**
 * @file board_common.h — ESP32-P4 MINIMAL board-layer shim.
 *
 * PX4'un gercek platforms/common/include/px4_platform_common/board_common.h (1231 satir)
 * NuttX board-config'a SIKI bagli: PX4_NUMBER_I2C_BUSES, CONFIG_BOARD_SERIAL_RC/RC_SERIAL_PORT,
 * PX4_CPU_UUID_BYTE_LENGTH, board_get_uuid/uuid32/mfguid, uuid_byte_t/uuid_uint32_t/mfguid_t...
 * Bunlarin HEPSI bir NuttX board'unun board_config.h'inden gelir; P4'te yok -> standalone
 * derlenmez (#error PX4_NUMBER_I2C_BUSES not supported, vb.).
 *
 * RULE-7 ACIK-FARK: P4 board-soyutlamasi (px4_boardconfig_stub gibi) ince shim'dir.
 * Su an board_common.h'i SADECE AdsbConflict.cpp include eder ve tek kullandigi sembol
 * PX4_GUID_BYTE_LENGTH'tir (BOARD_HAS_NO_UUID tanimli -> guid-fonksiyon dali derlenip cikar,
 * yalniz `#else` dali `tr.uas_id[i]=0xe0+i` PX4_GUID_BYTE_LENGTH'e bakar). PX4'teki deger
 * BIREBIR korunur (board_common.h:436). Ileride baska modul daha fazlasini isterse buraya
 * o sembol PX4-degeriyle eklenir (compiler-driven; uydurma YOK).
 */
#pragma once

/* PX4 platforms/common/include/px4_platform_common/board_common.h:436 — BIREBIR deger. */
#ifndef PX4_GUID_BYTE_LENGTH
#define PX4_GUID_BYTE_LENGTH              18
#endif
