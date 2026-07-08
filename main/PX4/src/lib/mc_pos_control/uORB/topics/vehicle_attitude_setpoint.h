/* PX4 msg/versioned/VehicleAttitudeSetpoint.msg birebir plain-struct. */
#pragma once
#include <stdint.h>

struct vehicle_attitude_setpoint_s {
	uint64_t timestamp;
	float yaw_sp_move_rate; /* rad/s (kullanıcı) */
	float q_d[4];           /* hedef quaternion */
	float thrust_body[3];   /* normalize itki, body FRD [-1,1] */
};
