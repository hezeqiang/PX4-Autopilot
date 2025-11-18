/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mc_se3_geo_pos_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

using namespace matrix;

MulticopterSE3GeometricPositionControl::MulticopterSE3GeometricPositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME)),
	_loop_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME ": interval"))
{
	_pos_int.zero();
	_vel_int.zero();
}

MulticopterSE3GeometricPositionControl::~MulticopterSE3GeometricPositionControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MulticopterSE3GeometricPositionControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	return true;
}

int MulticopterSE3GeometricPositionControl::task_spawn(int argc, char *argv[])
{
	MulticopterSE3GeometricPositionControl *instance = new MulticopterSE3GeometricPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

MulticopterSE3GeometricPositionControl *MulticopterSE3GeometricPositionControl::instantiate(int argc, char *argv[])
{
	return new MulticopterSE3GeometricPositionControl();
}

int MulticopterSE3GeometricPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterSE3GeometricPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SE3 Geometric Position Controller for Multicopters

This module implements a geometric position controller on SE3 manifold for multicopter platforms.
It provides precise position and velocity tracking using geometric control methods.

### Implementation
The controller uses SE3 geometric control theory to compute desired attitude and thrust commands
based on position and velocity errors. It operates on the special Euclidean group SE(3) which
represents the configuration space of a rigid body.

### Examples
CLI usage example:
$ mc_se3_geo_pos_control start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_se3_geo_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int MulticopterSE3GeometricPositionControl::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Position integral: [%.3f, %.3f, %.3f]", (double)_pos_int(0), (double)_pos_int(1), (double)_pos_int(2));
	PX4_INFO("Velocity integral: [%.3f, %.3f, %.3f]", (double)_vel_int(0), (double)_vel_int(1), (double)_vel_int(2));
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);

	return 0;
}

void MulticopterSE3GeometricPositionControl::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void MulticopterSE3GeometricPositionControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check for parameter updates
	parameters_update();

	// Check if vehicle_local_position has updated
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position;

		if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
			// Get current vehicle state
			Vector3f current_position(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z);
			Vector3f current_velocity(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);

			// Get setpoint
			vehicle_local_position_setpoint_s pos_sp;
			Vector3f position_setpoint = current_position; // Default to current position
			Vector3f velocity_setpoint = Vector3f(0.f, 0.f, 0.f);

			if (_vehicle_local_position_setpoint_sub.update(&pos_sp)) {
				if (PX4_ISFINITE(pos_sp.x) && PX4_ISFINITE(pos_sp.y) && PX4_ISFINITE(pos_sp.z)) {
					position_setpoint = Vector3f(pos_sp.x, pos_sp.y, pos_sp.z);
				}

				if (PX4_ISFINITE(pos_sp.vx) && PX4_ISFINITE(pos_sp.vy) && PX4_ISFINITE(pos_sp.vz)) {
					velocity_setpoint = Vector3f(pos_sp.vx, pos_sp.vy, pos_sp.vz);
				}
			}

			// Check control mode
			vehicle_control_mode_s vehicle_control_mode;
			_control_mode_current = false;

			if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
				_control_mode_current = vehicle_control_mode.flag_control_position_enabled;
			}

			// Check vehicle status
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_vehicle_land_detected = vehicle_land_detected.landed;
			}

			// Control position if in position control mode and not landed
			if (_control_mode_current && !_vehicle_land_detected) {
				control_position(current_position, current_velocity, position_setpoint, velocity_setpoint);

			} else {
				// Reset integrals when not in control
				reset_pos_int();
				reset_vel_int();
			}
		}
	}

	perf_end(_loop_perf);
}

bool MulticopterSE3GeometricPositionControl::control_position(const Vector3f &curr_pos, const Vector3f &curr_vel,
		const Vector3f &pos_sp, const Vector3f &vel_sp)
{
	// Position error
	Vector3f pos_err = pos_sp - curr_pos;

	// Velocity error
	Vector3f vel_err = vel_sp - curr_vel;

	// Position control gains
	Vector3f kp_pos(0.f);
	kp_pos(0) = kp_pos(1) = _param_mpc_xy_p.get();
	kp_pos(2) = _param_mpc_z_p.get();

	// Velocity control gains
	Vector3f kp_vel(0.f);
	Vector3f ki_vel(0.f);
	Vector3f kd_vel(0.f);

	kp_vel(0) = kp_vel(1) = _param_mpc_xy_vel_p.get();
	kp_vel(2) = _param_mpc_z_vel_p.get();

	ki_vel(0) = ki_vel(1) = _param_mpc_xy_vel_i.get();
	ki_vel(2) = _param_mpc_z_vel_i.get();

	kd_vel(0) = kd_vel(1) = _param_mpc_xy_vel_d.get();
	kd_vel(2) = _param_mpc_z_vel_d.get();

	// Update integration
	const float dt = math::constrain(hrt_elapsed_time(&_time_stamp_last_loop) * 1e-6f, 0.0002f, 0.02f);
	_time_stamp_last_loop = hrt_absolute_time();

	// Position integral
	_pos_int += pos_err * dt;

	// Velocity integral
	_vel_int += vel_err * dt;

	// Compute desired velocity from position error
	Vector3f vel_sp_pos = pos_err.emult(kp_pos);

	// Total velocity setpoint
	Vector3f vel_sp_total = vel_sp + vel_sp_pos;
	Vector3f vel_err_total = vel_sp_total - curr_vel;

	// Compute thrust vector using SE3 geometric control
	Vector3f thrust_sp = vel_err_total.emult(kp_vel) + _vel_int.emult(ki_vel);

	// Add feed-forward gravity compensation
	thrust_sp(2) += CONSTANTS_ONE_G;

	// Add hover thrust
	thrust_sp(2) += _param_mpc_thr_hover.get() * CONSTANTS_ONE_G;

	// Limit thrust
	thrust_sp = limit_thrust(thrust_sp);

	// Compute desired attitude from thrust vector
	vehicle_attitude_setpoint_s attitude_setpoint{};
	attitude_setpoint.timestamp = hrt_absolute_time();

	// Normalize thrust vector to get desired z-body direction
	Vector3f body_z = thrust_sp.normalized();

	// Compute desired yaw (maintain current yaw for now)
	float yaw_sp = 0.0f; // This should come from yaw setpoint

	// Compute desired attitude using thrust vector
	Vector3f body_x(cosf(yaw_sp), sinf(yaw_sp), 0.0f);
	Vector3f body_y = body_z.cross(body_x).normalized();
	body_x = body_y.cross(body_z);

	// Create rotation matrix
	Matrix3f R_sp;
	R_sp.col(0) = body_x;
	R_sp.col(1) = body_y;
	R_sp.col(2) = body_z;

	// Convert to quaternion
	Quatf q_sp(R_sp);
	q_sp.copyTo(attitude_setpoint.q_d);

	// Set thrust
	attitude_setpoint.thrust_body[2] = -thrust_sp.length(); // Negative because thrust is in -z direction

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

	return true;
}

void MulticopterSE3GeometricPositionControl::reset_pos_int()
{
	_pos_int.zero();
}

void MulticopterSE3GeometricPositionControl::reset_vel_int()
{
	_vel_int.zero();
}

Vector3f MulticopterSE3GeometricPositionControl::limit_thrust(const Vector3f &thrust_sp) const
{
	const float thrust_max = 1.0f; // This should be a parameter
	const float thrust_mag = thrust_sp.length();

	if (thrust_mag > thrust_max) {
		return thrust_sp * (thrust_max / thrust_mag);
	}

	return thrust_sp;
}

int mc_se3_geo_pos_control_main(int argc, char *argv[])
{
	return MulticopterSE3GeometricPositionControl::main(argc, argv);
}
