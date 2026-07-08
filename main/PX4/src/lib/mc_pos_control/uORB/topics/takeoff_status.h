/**
 * @file takeoff_status.h
 * @brief PX4 uORB takeoff_status mesajinin (TakeoffStatus.msg) sade-struct karsiligi.
 *
 * Byte-kopyalanan mc_pos_control/Takeoff (Takeoff.hpp) yalnizca TAKEOFF_STATE_*
 * sabitlerini (TakeoffState enum class baslangic degerleri) kullanir. Degerler
 * PX4 msg ile BIREBIR (UNINITIALIZED=0..FLIGHT=5). Plain-struct deseni (EKF
 * topic'leri gibi) — uORB yok, sadece tip/sabit.
 */
#pragma once

#include <stdint.h>

struct takeoff_status_s {
	static constexpr uint8_t TAKEOFF_STATE_UNINITIALIZED = 0;
	static constexpr uint8_t TAKEOFF_STATE_DISARMED = 1;
	static constexpr uint8_t TAKEOFF_STATE_SPOOLUP = 2;
	static constexpr uint8_t TAKEOFF_STATE_READY_FOR_TAKEOFF = 3;
	static constexpr uint8_t TAKEOFF_STATE_RAMPUP = 4;
	static constexpr uint8_t TAKEOFF_STATE_FLIGHT = 5;
};
