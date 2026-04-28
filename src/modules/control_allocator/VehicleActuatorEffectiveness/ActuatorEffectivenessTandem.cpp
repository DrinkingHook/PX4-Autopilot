/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
// ActuatorEffectivenessTandem.cpp
#include "ActuatorEffectivenessTandem.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessTandem::ActuatorEffectivenessTandem(ModuleParams *parent)
	: ModuleParams(parent),
	  _rotor_front(_geo_front),
	  _rotor_rear(_geo_rear)
{
	// ── 前/后旋翼共用角度、臂长（可以扩展为独立参数）──
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles.swash_plate_servos[i].trim_front = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "C1_SV_CS%u_TRIM", i);
		_param_handles.swash_plate_servos[i].trim_rear = param_find(buffer);
	}

	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

	// 曲线设定
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve_front[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve_rear[i] = param_find(buffer);
	}

	// 其它单一参数
	_param_handles.spoolup_time   = param_find("COM_SPOOLUP_TIME");
	_param_handles.max_servo_throw = param_find("CA_MAX_SVO_THROW");
	_param_handles.pitch_scale = param_find("CA_TDM_PT_S");
	_param_handles.roll_scale  = param_find("CA_TDM_RL_S");
	_param_handles.yaw_scale   = param_find("CA_TDM_YW_S");

	updateParams();
}

void ActuatorEffectivenessTandem::updateParams()
{
	ModuleParams::updateParams();

	// ── 舵机数量 ──
	int32_t n = 3;
	param_get(_param_handles.num_swash_plate_servos, &n);
	n = math::constrain(n, (int32_t)2, (int32_t)NUM_SWASH_PLATE_SERVOS_MAX);
	_geo_front.num_swash_plate_servos = n;
	_geo_rear.num_swash_plate_servos  = n;  // 前后舵机数量相同

	// ── 舵机几何（前后共用角度/臂长，trim 独立）──
	for (int i = 0; i < n; ++i) {
		float angle_deg = 0.f;
		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
		const float angle = math::radians(angle_deg);

		float arm = 1.f;
		param_get(_param_handles.swash_plate_servos[i].arm_length, &arm);

		_geo_front.swash_plate_servos[i].angle      = angle;
		_geo_front.swash_plate_servos[i].arm_length = arm;
		param_get(_param_handles.swash_plate_servos[i].trim_front,
			  &_geo_front.swash_plate_servos[i].trim);

		_geo_rear.swash_plate_servos[i].angle      = angle;
		_geo_rear.swash_plate_servos[i].arm_length = arm;
		param_get(_param_handles.swash_plate_servos[i].trim_rear,
			  &_geo_rear.swash_plate_servos[i].trim);
	}

	// ── 曲线 ──
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i],   &_geo_front.throttle_curve[i]);
		param_get(_param_handles.pitch_curve_front[i], &_geo_front.pitch_curve[i]);
		param_get(_param_handles.pitch_curve_rear[i],  &_geo_rear.pitch_curve[i]);
		// 后旋翼共用前旋翼油门曲线（只有一个主电机）
		_geo_rear.throttle_curve[i] = _geo_front.throttle_curve[i];
	}

	// ── spoolup ──
	param_get(_param_handles.spoolup_time, &_geo_front.spoolup_time);
	_geo_rear.spoolup_time = _geo_front.spoolup_time;

	// ── 线性化（前后一致）──
	float max_servo_throw_deg = 0.f;
	param_get(_param_handles.max_servo_throw, &max_servo_throw_deg);

	if (max_servo_throw_deg > 0.f) {
		const float max_throw = math::radians(max_servo_throw_deg);
		_geo_front.linearize_servos       = 1;
		_geo_front.max_servo_height       = sinf(max_throw);
		_geo_front.inverse_max_servo_throw = 1.f / max_throw;

	} else {
		_geo_front.linearize_servos        = 0;
		_geo_front.max_servo_height        = 0.f;
		_geo_front.inverse_max_servo_throw = 0.f;
	}

	// 后旋翼同步
	_geo_rear.linearize_servos        = _geo_front.linearize_servos;
	_geo_rear.max_servo_height        = _geo_front.max_servo_height;
	_geo_rear.inverse_max_servo_throw = _geo_front.inverse_max_servo_throw;

	// ── 可调增益 ──
	param_get(_param_handles.pitch_scale, &_pitch_scale);
	param_get(_param_handles.roll_scale,  &_roll_scale);
	param_get(_param_handles.yaw_scale,   &_yaw_scale);

	// 同步到 RotorHead
	_rotor_front.setGeometry(_geo_front);
	_rotor_rear.setGeometry(_geo_rear);
}

bool ActuatorEffectivenessTandem::getEffectivenessMatrix(
	Configuration &configuration, EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// 主电机
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// 前旋翼舵机
	_first_swash_servo_index = configuration.num_actuators_matrix[0];

	for (int i = 0; i < _geo_front.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		configuration.trim[configuration.selected_matrix](
			_first_swash_servo_index + i) = _geo_front.swash_plate_servos[i].trim;
	}

	// 后旋翼舵机（trim 索引必须偏移）
	const int rear_start = _first_swash_servo_index + _geo_front.num_swash_plate_servos;

	for (int i = 0; i < _geo_rear.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		configuration.trim[configuration.selected_matrix](
			rear_start + i) = _geo_rear.swash_plate_servos[i].trim;
	}

	return true;
}

void ActuatorEffectivenessTandem::updateSetpoint(
	const matrix::Vector<float, NUM_AXES> &control_sp,
	int matrix_index,
	ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min,
	const ActuatorVector &actuator_max)
{
	_sat_front = {};
	_sat_rear  = {};

	const float spoolup = throttleSpoolupProgress();
	const float throttle = math::interpolateN(
				       -control_sp(ControlAxis::THRUST_Z), _geo_front.throttle_curve) * spoolup;

	const float collective_front = math::interpolateN(
					       -control_sp(ControlAxis::THRUST_Z), _geo_front.pitch_curve);
	const float collective_rear  = math::interpolateN(
					       -control_sp(ControlAxis::THRUST_Z), _geo_rear.pitch_curve);

	// 主电机
	actuator_sp(0) = mainMotorEngaged() ? throttle : NAN;

	const float roll_sp  = control_sp(ControlAxis::ROLL)  * _roll_scale;
	const float pitch_sp = control_sp(ControlAxis::PITCH) * _pitch_scale;
	const float yaw_sp   = control_sp(ControlAxis::YAW)   * _yaw_scale;

	// 前旋翼：collective + pitch抬头分量
	_sat_front = _rotor_front.computeSetpoints(
			     collective_front + pitch_sp,   // 直接加到总距
			     roll_sp,
			     0.f,                           // pitch_coeff无效，传0
			     +yaw_sp,
			     _first_swash_servo_index,
			     actuator_sp, actuator_min, actuator_max);
	const int rear_start = _first_swash_servo_index + _geo_front.num_swash_plate_servos;
	// 后旋翼：collective - pitch抬头分量
	_sat_rear = _rotor_rear.computeSetpoints(
			    collective_rear - pitch_sp,    // 后旋翼反向
			    roll_sp,
			    0.f,
			    -yaw_sp,
			    rear_start,
			    actuator_sp, actuator_min, actuator_max);
}

void ActuatorEffectivenessTandem::getUnallocatedControl(
	int matrix_index, control_allocator_status_s &status)
{
	// 先清零，再合并两个旋翼头
	status.unallocated_torque[0] = 0.f;
	status.unallocated_torque[1] = 0.f;
	status.unallocated_torque[2] = 0.f;
	status.unallocated_thrust[2] = 0.f;

	HelicopterRotorHead::mergeUnallocatedControl(_sat_front, status);
	HelicopterRotorHead::mergeUnallocatedControl(_sat_rear,  status);
}

float ActuatorEffectivenessTandem::throttleSpoolupProgress()
{
	vehicle_status_s vs;

	if (_vehicle_status_sub.update(&vs)) {
		_armed      = vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vs.armed_time;
	}

	const float elapsed  = (hrt_absolute_time() - _armed_time) / 1e6f;
	const float progress = elapsed / _geo_front.spoolup_time;
	return (_armed && progress < 1.f) ? progress : 1.f;
}

bool ActuatorEffectivenessTandem::mainMotorEngaged()
{
	manual_control_switches_s sw;

	if (_manual_control_switches_sub.update(&sw)) {
		_main_motor_engaged =
			sw.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
			|| sw.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return _main_motor_engaged;
}
