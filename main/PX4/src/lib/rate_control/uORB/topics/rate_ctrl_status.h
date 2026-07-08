/* PX4 RateCtrlStatus.msg birebir plain-struct (getRateControlStatus icin). */
#pragma once
#include <stdint.h>
struct rate_ctrl_status_s {
	uint64_t timestamp;
	float rollspeed_integ;
	float pitchspeed_integ;
	float yawspeed_integ;
};
