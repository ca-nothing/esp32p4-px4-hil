/* PX4 msg/versioned/TrajectorySetpoint.msg birebir plain-struct (EKF deseni:
 * uORB pub/sub runtime'ı yok, sadece veri struct'ı). PositionControl byte-kopyası için. */
#pragma once
#include <stdint.h>

struct trajectory_setpoint_s {
	uint64_t timestamp;
	float position[3];     /* m */
	float velocity[3];     /* m/s */
	float acceleration[3]; /* m/s^2 */
	float jerk[3];         /* m/s^3 (logging) */
	float yaw;             /* rad -PI..+PI */
	float yawspeed;        /* rad/s NED z */
};
/* C uyumu (fc_flight_task / FlightModeManager): tag -> tip. C++'ta da geçerli (redundant). */
typedef struct trajectory_setpoint_s trajectory_setpoint_s;
