/* PX4 msg/VehicleLocalPositionSetpoint.msg birebir plain-struct. */
#pragma once
#include <stdint.h>

struct vehicle_local_position_setpoint_s {
	uint64_t timestamp;
	float x, y, z;          /* m NED */
	float vx, vy, vz;       /* m/s */
	float acceleration[3];  /* m/s^2 */
	float thrust[3];        /* normalize itki vektörü NED */
	float yaw;              /* rad NED -PI..+PI */
	float yawspeed;         /* rad/s */
};
