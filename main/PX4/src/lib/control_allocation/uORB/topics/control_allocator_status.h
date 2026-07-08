/* PX4 ControlAllocatorStatus.msg birebir plain-struct (ActuatorEffectiveness.hpp +
 * SequentialDesaturation::updateControlAllocatorStatus için). uORB pub/sub runtime yok. */
#pragma once
#include <stdint.h>

struct control_allocator_status_s {
	uint64_t timestamp;
	bool torque_setpoint_achieved;
	float unallocated_torque[3];
	bool thrust_setpoint_achieved;
	float unallocated_thrust[3];
	int8_t actuator_saturation[16];
	uint16_t handled_motor_failure_mask;
	uint16_t motor_stop_mask;
	bool actuator_group_preflight_check_active;
};
