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

// ActuatorEffectivenessTandem.hpp
#pragma once

#include "ActuatorEffectiveness.hpp"
#include "HelicopterRotorHead.hpp"
#include "HelicopterGeometry.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>

class ActuatorEffectivenessTandem : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_ROTORS = 2;  // 前旋翼 + 后旋翼

	explicit ActuatorEffectivenessTandem(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTandem() = default;

	bool getEffectivenessMatrix(Configuration &configuration,
				    EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
			    int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index,
				   control_allocator_status_s &status) override;

	const char *name() const override { return "Tandem Helicopter"; }

private:
	void updateParams() override;
	float throttleSpoolupProgress();
	bool  mainMotorEngaged();

	// ── 参数句柄 ────────────────────────────────────────────────
	struct ParamHandlesSwashPlate {
		param_t angle;
		param_t arm_length;
		param_t trim_front;   // CA_SV_CS%u_TRIM
		param_t trim_rear;    // C1_SV_CS%u_TRIM  (后旋翼独立 trim)
	};

	struct ParamHandles {
		ParamHandlesSwashPlate swash_plate_servos[NUM_SWASH_PLATE_SERVOS_MAX];
		param_t num_swash_plate_servos;
		param_t throttle_curve[NUM_CURVE_POINTS];
		param_t pitch_curve_front[NUM_CURVE_POINTS];  // CA_HELI_PITCH_C%u
		param_t pitch_curve_rear[NUM_CURVE_POINTS];   // CA2_HLP_C%u
		param_t spoolup_time;
		param_t max_servo_throw;
		// 偏航/横滚增益（建议独立注册）
		param_t pitch_scale;  // CA_TANDEM_PTCH_S
		param_t roll_scale;   // CA_TANDEM_ROLL_S
		param_t yaw_scale;    // CA_TANDEM_YAW_S
	} _param_handles{};

	// ── 两个旋翼头 ───────────────────────────────────────────────
	RotorGeometry      _geo_front{};
	RotorGeometry      _geo_rear{};
	HelicopterRotorHead _rotor_front{_geo_front};
	HelicopterRotorHead _rotor_rear{_geo_rear};

	// 可调增益（替代硬编码的 0.5f / 0.9f）
	float _pitch_scale{0.9f};
	float _roll_scale{0.9f};
	float _yaw_scale{0.5f};

	int _first_swash_servo_index{0};

	// 两个旋翼头各自的饱和标志
	RotorSaturationFlags _sat_front{};
	RotorSaturationFlags _sat_rear{};

	// spoolup / arm 状态
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	bool     _armed{false};
	uint64_t _armed_time{0};
	bool     _main_motor_engaged{true};
};