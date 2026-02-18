/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

/**
 *
 * This module is a modification of the fixed wing / rover module  and it is designed for unmanned underwater vehicles  (UUV).
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * 2025: refactoring of the mode settings: attitude, rate and manual control now working.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
 * @author Pedro Roque <padr@kth.se>
 */

#include "uuv_att_control.hpp"


/**
 * UUV attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);


UUVAttitudeControl::UUVAttitudeControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

UUVAttitudeControl::~UUVAttitudeControl()
{
	perf_free(_loop_perf);
}

bool UUVAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void UUVAttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void UUVAttitudeControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u,
		float thrust_x, float thrust_y, float thrust_z)
{
	if (PX4_ISFINITE(roll_u)) {
		roll_u = math::constrain(roll_u, -_param_torque_sat.get(), _param_torque_sat.get());
		_vehicle_torque_setpoint.xyz[0] = roll_u;

	} else {
		_vehicle_torque_setpoint.xyz[0] = 0.0f;
	}

	if (PX4_ISFINITE(pitch_u)) {
		pitch_u = math::constrain(pitch_u, -_param_torque_sat.get(), _param_torque_sat.get());
		_vehicle_torque_setpoint.xyz[1] = pitch_u;

	} else {
		_vehicle_torque_setpoint.xyz[1] = 0.0f;
	}

	if (PX4_ISFINITE(yaw_u)) {
		yaw_u = math::constrain(yaw_u, -_param_torque_sat.get(), _param_torque_sat.get());
		_vehicle_torque_setpoint.xyz[2] = yaw_u;

	} else {
		_vehicle_torque_setpoint.xyz[2] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_x)) {
		thrust_x = math::constrain(thrust_x, -_param_thrust_sat.get(), _param_thrust_sat.get());
		_vehicle_thrust_setpoint.xyz[0] = thrust_x;

	} else {
		_vehicle_thrust_setpoint.xyz[0] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_y)) {
		thrust_y = math::constrain(thrust_y, -_param_thrust_sat.get(), _param_thrust_sat.get());
		_vehicle_thrust_setpoint.xyz[1] = thrust_y;

	} else {
		_vehicle_thrust_setpoint.xyz[1] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_z)) {
		thrust_z = math::constrain(thrust_z, -_param_thrust_sat.get(), _param_thrust_sat.get());
		_vehicle_thrust_setpoint.xyz[2] = thrust_z;

	} else {
		_vehicle_thrust_setpoint.xyz[2] = 0.0f;
	}
}

void UUVAttitudeControl::control_attitude_geo(const vehicle_attitude_s &attitude,
		const vehicle_attitude_setpoint_s &attitude_setpoint, const vehicle_angular_velocity_s &angular_velocity,
		const vehicle_rates_setpoint_s &rates_setpoint, bool attitude_control_enabled, float dt)
{
	/** Geometric Controller
	 *
	 * based on
	 * D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors", IEEE ICRA 2011, pp. 2520-2525.
	 * D. A. Duecker, A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow, “Micro Underwater Vehicle Hydrobatics: A SubmergedFuruta Pendulum,” IEEE ICRA 2018, pp. 7498–7503.
	 */
	Eulerf euler_angles(matrix::Quatf(attitude.q));

	const Eulerf setpoint_euler_angles(matrix::Quatf(attitude_setpoint.q_d));
	const float roll_body = setpoint_euler_angles(0);
	const float pitch_body = setpoint_euler_angles(1);
	const float yaw_body = setpoint_euler_angles(2);

	float roll_rate_desired = rates_setpoint.roll;
	float pitch_rate_desired = rates_setpoint.pitch;
	float yaw_rate_desired = rates_setpoint.yaw;

	/* get attitude setpoint rotational matrix */
	Dcmf rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	/* get current rotation matrix from control state quaternions */
	Quatf q_att(attitude.q);
	Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	Vector3f e_R_vec;
	Vector3f torques;
	torques.setZero();

	/* Compute matrix: attitude error */
	Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5;

	/* vee-map the error to get a vector instead of matrix e_R */
	e_R_vec(0) = e_R(2, 1);  /**< Roll  */
	e_R_vec(1) = e_R(0, 2);  /**< Pitch */
	e_R_vec(2) = e_R(1, 0);  /**< Yaw   */

	Vector3f omega{angular_velocity.xyz};
	omega(0) -= roll_rate_desired;
	omega(1) -= pitch_rate_desired;
	omega(2) -= yaw_rate_desired;

	/**< P-Control */
	if (attitude_control_enabled) {
		torques(0) = - e_R_vec(0) * _param_roll_p.get();	/**< Roll  */
		torques(1) = - e_R_vec(1) * _param_pitch_p.get();	/**< Pitch */
		torques(2) = - e_R_vec(2) * _param_yaw_p.get();		/**< Yaw   */

		// take thrust from attitude message
		float thrust_x = attitude_setpoint.thrust_body[0];
		float thrust_y = attitude_setpoint.thrust_body[1];
		float thrust_z = attitude_setpoint.thrust_body[2];

		/**< PD-Control */
		torques(0) -= omega(0) * _param_roll_d.get();  /**< Roll  */
		torques(1) -= omega(1) * _param_pitch_d.get(); /**< Pitch */
		torques(2) -= omega(2) * _param_yaw_d.get();   /**< Yaw   */

		float roll_u = torques(0);
		float pitch_u = torques(1);
		float yaw_u = torques(2);

		// Yaw integral: eliminates steady-state yaw error
		const float yaw_i_gain = _param_yaw_i.get();

		if (yaw_i_gain > FLT_EPSILON && dt > 0.0f) {
			// Reset on large error to prevent windup from setpoint jumps
			if (fabsf(e_R_vec(2)) > 0.5f) {
				_yaw_integral = 0.0f;
			}

			_yaw_integral += e_R_vec(2) * dt;
			_yaw_integral = math::constrain(_yaw_integral, -_param_yaw_i_max.get(), _param_yaw_i_max.get());
			yaw_u -= _yaw_integral * yaw_i_gain;

		} else {
			_yaw_integral = 0.0f;
		}

		// Feedforward: compensate pitch coupling from surge thrusters below CG
		const float surge_pff = _param_surge_pitch_ff.get();

		if (surge_pff > FLT_EPSILON) {
			pitch_u -= thrust_x * surge_pff;
		}

		// Feedforward: compensate yaw coupling from surge thruster reaction torque
		const float surge_yff = _param_surge_yaw_ff.get();

		if (fabsf(surge_yff) > FLT_EPSILON) {
			yaw_u -= thrust_x * surge_yff;
		}

		// Feedforward: compensate yaw coupling from canted thruster reaction torque
		const float heave_yff = _param_heave_yaw_ff.get();

		if (fabsf(heave_yff) > FLT_EPSILON) {
			yaw_u -= thrust_z * heave_yff;
		}

		constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);

	} else {
		// No attitude control — reset yaw integrator
		_yaw_integral = 0.0f;

		// take thrust from rates message
		float thrust_x = _rates_setpoint.thrust_body[0];
		float thrust_y = _rates_setpoint.thrust_body[1];
		float thrust_z = _rates_setpoint.thrust_body[2];

		/**< PD-Control */
		torques(0) -= omega(0) * _param_roll_d.get();  /**< Roll  */
		torques(1) -= omega(1) * _param_pitch_d.get(); /**< Pitch */
		torques(2) -= omega(2) * _param_yaw_d.get();   /**< Yaw   */

		float roll_u = torques(0);
		float pitch_u = torques(1);
		float yaw_u = torques(2);

		// Feedforward: compensate pitch coupling from surge thrusters below CG
		const float surge_pff = _param_surge_pitch_ff.get();

		if (surge_pff > FLT_EPSILON) {
			pitch_u -= thrust_x * surge_pff;
		}

		// Feedforward: compensate yaw coupling from surge thruster reaction torque
		const float surge_yff2 = _param_surge_yaw_ff.get();

		if (fabsf(surge_yff2) > FLT_EPSILON) {
			yaw_u -= thrust_x * surge_yff2;
		}

		// Feedforward: compensate yaw coupling from canted thruster reaction torque
		const float heave_yff = _param_heave_yaw_ff.get();

		if (fabsf(heave_yff) > FLT_EPSILON) {
			yaw_u -= thrust_z * heave_yff;
		}

		constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_x, thrust_y, thrust_z);
	}

	/* Geometric Controller END*/
}

void UUVAttitudeControl::generate_attitude_setpoint(float dt)
{
	// Avoid accumulating absolute yaw error with arming stick gesture
	float roll = Eulerf(matrix::Quatf(_attitude_setpoint.q_d)).phi();
	float pitch = Eulerf(matrix::Quatf(_attitude_setpoint.q_d)).theta();
	float yaw = Eulerf(matrix::Quatf(_attitude_setpoint.q_d)).psi();

	// Integrate manual control inputs
	float yaw_setpoint = yaw + _manual_control_setpoint.yaw * dt * _param_sgm_yaw.get();
	float roll_setpoint = roll + _manual_control_setpoint.roll * dt * _param_sgm_roll.get();
	float pitch_setpoint = pitch + -_manual_control_setpoint.pitch * dt * _param_sgm_pitch.get();

	// Generate target quaternion
	Eulerf euler_sp(roll_setpoint, pitch_setpoint, yaw_setpoint);
	Quatf q_sp = euler_sp;

	// Normalize the quaternion to avoid numerical issues
	q_sp.normalize();

	q_sp.copyTo(_attitude_setpoint.q_d);

	_attitude_setpoint.thrust_body[0] = _manual_control_setpoint.throttle * _param_sgm_thrtl.get();

	_attitude_setpoint.timestamp = hrt_absolute_time();
}

void UUVAttitudeControl::generate_rates_setpoint(float dt)
{
	// Integrate manual control inputs
	_rates_setpoint.roll = _manual_control_setpoint.roll * dt * _param_rgm_roll.get();
	_rates_setpoint.pitch = -_manual_control_setpoint.pitch * dt * _param_rgm_pitch.get();
	_rates_setpoint.yaw = _manual_control_setpoint.yaw * dt * _param_rgm_yaw.get();

	_rates_setpoint.thrust_body[0] = _manual_control_setpoint.throttle * _param_rgm_thrtl.get();
	_rates_setpoint.timestamp = hrt_absolute_time();

}

void UUVAttitudeControl::check_setpoint_validity(vehicle_attitude_s &v_att)
{
	const float _setpoint_age = (hrt_absolute_time() - _attitude_setpoint.timestamp) * 1e-6f;

	if (_setpoint_age < 0.0f || _setpoint_age > _param_setpoint_max_age.get()) {
		reset_attitude_setpoint(v_att);
	}
}

void UUVAttitudeControl::reset_attitude_setpoint(vehicle_attitude_s &v_att)
{
	_attitude_setpoint.timestamp = hrt_absolute_time();
	_attitude_setpoint.q_d[0] = v_att.q[0];
	_attitude_setpoint.q_d[1] = v_att.q[1];
	_attitude_setpoint.q_d[2] = v_att.q[2];
	_attitude_setpoint.q_d[3] = v_att.q[3];
	_attitude_setpoint.thrust_body[0] = 0.f;
	_attitude_setpoint.thrust_body[1] = 0.f;
	_attitude_setpoint.thrust_body[2] = 0.f;
	_yaw_integral = 0.0f;
}

bool UUVAttitudeControl::select_angular_velocity(vehicle_angular_velocity_s &out)
{
    const hrt_abstime now = hrt_absolute_time();

    // Try external source first (always, no parameter check)
    if (try_get_external_angular_velocity(out, now)) {
        transition_to_source(AngularVelocitySource::EXTERNAL, now);
        return true;
    }

    // Fallback to PX4 gyro
    if (_angular_velocity_sub.copy(&out)) {
        transition_to_source(AngularVelocitySource::PX4_GYRO, now);
        return true;
    }

    // No valid source available
    transition_to_source(AngularVelocitySource::NONE, now);

    if (now - _last_ext_vel_warning_us > WARNING_THROTTLE_US) {
        PX4_ERR("No valid angular velocity source available");
        _last_ext_vel_warning_us = now;
    }

    return false;
}

bool UUVAttitudeControl::try_get_external_angular_velocity(vehicle_angular_velocity_s &out, const hrt_abstime &now)
{
    external_vehicle_angular_velocity_s ext{};

    // Try to get external data
    if (!_ext_ang_vel_sub.update(&ext)) {
        return false;
    }

    // Validate timestamp age
    const float age = (now - ext.timestamp) * 1e-6f;

    if (age < 0.f) {
        log_validation_failure("Future timestamp", age, now);
        return false;
    }

    if (age > _param_extw_max_age_s.get()) {
        log_validation_failure("Stale data", age, now);
        return false;
    }

    // Validate finite values
    if (!PX4_ISFINITE(ext.xyz[0]) || !PX4_ISFINITE(ext.xyz[1]) || !PX4_ISFINITE(ext.xyz[2])) {
        log_validation_failure("Non-finite values", age, now);
        return false;
    }

    // All validation passed - copy data
    out.timestamp = ext.timestamp;
    out.timestamp_sample = ext.timestamp_sample ? ext.timestamp_sample : ext.timestamp;
    out.xyz[0] = ext.xyz[0];
    out.xyz[1] = ext.xyz[1];
    out.xyz[2] = ext.xyz[2];
    out.xyz_derivative[0] = ext.xyz_derivative[0];
    out.xyz_derivative[1] = ext.xyz_derivative[1];
    out.xyz_derivative[2] = ext.xyz_derivative[2];

    return true;
}

void UUVAttitudeControl::transition_to_source(AngularVelocitySource new_source, const hrt_abstime &now)
{
    if (new_source == _current_ang_vel_source) {
        return; // No change
    }

    // Add hysteresis to prevent rapid switching
    if (now - _last_source_switch_us < SOURCE_SWITCH_HYSTERESIS_US) {
        return; // Too soon to switch
    }

    // Log the transition
    const char* source_names[] = {"EXTERNAL", "PX4_GYRO", "NONE"};
    PX4_INFO("Angular velocity source: %s -> %s",
             source_names[static_cast<int>(_current_ang_vel_source)],
             source_names[static_cast<int>(new_source)]);

    _current_ang_vel_source = new_source;
    _last_source_switch_us = now;
}

void UUVAttitudeControl::log_validation_failure(const char* reason, float age, const hrt_abstime &now)
{
    if (now - _last_ext_vel_warning_us > WARNING_THROTTLE_US) {
        PX4_WARN("External angular velocity rejected: %s (age=%.3fs)",
                 reason, (double)age);
        _last_ext_vel_warning_us = now;
    }
}

// Optional: Add diagnostic method for status output
const char* UUVAttitudeControl::get_angular_velocity_source_name() const
{
    switch (_current_ang_vel_source) {
        case AngularVelocitySource::EXTERNAL: return "EXTERNAL";
        case AngularVelocitySource::PX4_GYRO: return "PX4_GYRO";
        case AngularVelocitySource::NONE: return "NONE";
        default: return "UNKNOWN";
    }
}

void UUVAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	vehicle_attitude_s attitude;

	/* only run controller if attitude changed */
	if (_vehicle_attitude_sub.update(&attitude)) {
		const float dt = math::constrain(((attitude.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = attitude.timestamp_sample;

		vehicle_angular_velocity_s angular_velocity {};

		select_angular_velocity(angular_velocity);

		// _angular_velocity_sub.copy(&angular_velocity);

		/* Check that we are not in position / velocity / altitude modes
		   and that we are using manual inputs */
		if (_vcontrol_mode.flag_control_manual_enabled
		    && !_vcontrol_mode.flag_control_position_enabled
		    && !_vcontrol_mode.flag_control_velocity_enabled
		    && !_vcontrol_mode.flag_control_altitude_enabled) {

			/* Update manual setpoints */
			_manual_control_setpoint_sub.update(&_manual_control_setpoint);

			if (_vcontrol_mode.flag_control_attitude_enabled
			    && _vcontrol_mode.flag_control_rates_enabled) {
				/* Run stabilized mode */
				_vehicle_rates_setpoint_sub.update(&_rates_setpoint);

				// Check setpoint validty
				check_setpoint_validity(attitude);

				/* Generate atttiude setpoint from sticks */
				generate_attitude_setpoint(dt);

				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint, true, dt);

			} else if (!_vcontrol_mode.flag_control_attitude_enabled
				   && _vcontrol_mode.flag_control_rates_enabled) {
				/* Run Rate mode */
				generate_rates_setpoint(dt);

				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint, false, dt);

			} else if (!_vcontrol_mode.flag_control_attitude_enabled
				   && !_vcontrol_mode.flag_control_rates_enabled) {
				/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
				constrain_actuator_commands(_manual_control_setpoint.roll * _param_mgm_roll.get(),
							    -_manual_control_setpoint.pitch * _param_mgm_pitch.get(),
							    _manual_control_setpoint.yaw * _param_mgm_yaw.get(),
							    _manual_control_setpoint.throttle * _param_mgm_thrtl.get(),
							    0.f,
							    0.f);
			}

		} else {
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				/* Get attitude and rate setpoints and control system */
				_vehicle_attitude_setpoint_sub.update(&_attitude_setpoint); // come from position controller
				_vehicle_rates_setpoint_sub.update(&_rates_setpoint); // come from an external source?
				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint, true, dt);

			} else {
				/* Get rate setpoints and control system */
				_vehicle_rates_setpoint_sub.update(&_rates_setpoint);
				control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint, false, dt);
			}
		}

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_manual_enabled ||
		    _vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled) {

			_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint.timestamp_sample = 0.f;
			_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

			_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint.timestamp_sample = 0.f;
			_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
		}
	}

	perf_end(_loop_perf);
}

int UUVAttitudeControl::task_spawn(int argc, char *argv[])
{
	UUVAttitudeControl *instance = new UUVAttitudeControl();

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

int UUVAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions

### Examples
CLI usage example:
$ uuv_att_control start
$ uuv_att_control status
$ uuv_att_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uuv_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uuv_att_control_main(int argc, char *argv[])
{
	return UUVAttitudeControl::main(argc, argv);
}
