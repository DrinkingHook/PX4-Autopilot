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

// HelicopterRotorHead.cpp
#include "HelicopterRotorHead.hpp"
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/control_allocator_status.h>

using namespace matrix;

/**
 * @brief 计算设定点
 * @param pitch_sp 此参数设置为0 是因为Tandem机型pitch轴运动时斜盘一起运动所以直接叠加到油门总距上
 */
RotorSaturationFlags HelicopterRotorHead::computeSetpoints(
	float collective_pitch,
	float roll_sp,
	float pitch_sp,
	float yaw_sp,
	int   start,
	ActuatorVector &actuator_sp,
	const ActuatorVector &actuator_min,
	const ActuatorVector &actuator_max) const
{
	RotorSaturationFlags flags{};

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		const float roll_coeff  = sinf(_geometry.swash_plate_servos[i].angle)
					  * _geometry.swash_plate_servos[i].arm_length;
		const float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle)
					  * _geometry.swash_plate_servos[i].arm_length;

		float sp = collective_pitch          // 已经包含了pitch分量
			   - roll_sp * roll_coeff      // 横滚仍然用roll_coeff
			   + yaw_sp  * (-roll_coeff)   // 偏航用roll_coeff差动
			   + _geometry.swash_plate_servos[i].trim;

		// pitch_coeff * pitch_sp 这一项完全删掉

		if (_geometry.linearize_servos) {
			sp = getLinearServoOutput(sp);
		}

		actuator_sp(start + i) = sp;

		// 饱和检测
		if (sp < actuator_min(start + i)) {
			setSaturationFlag(roll_coeff,  flags.roll_pos,  flags.roll_neg);
			setSaturationFlag(pitch_coeff, flags.pitch_neg, flags.pitch_pos);

		} else if (sp > actuator_max(start + i)) {
			setSaturationFlag(roll_coeff,  flags.roll_neg,  flags.roll_pos);
			setSaturationFlag(pitch_coeff, flags.pitch_pos, flags.pitch_neg);
		}
	}

	return flags;
}

float HelicopterRotorHead::getLinearServoOutput(float input) const
{
	input = math::constrain(input, -1.f, 1.f);
	float servo_height = _geometry.max_servo_height * input;
	return _geometry.inverse_max_servo_throw * asinf(servo_height);
}

void HelicopterRotorHead::setSaturationFlag(
	float coeff, bool &pos_flag, bool &neg_flag)
{
	if (coeff > 0.f) { pos_flag = true; }

	else if (coeff < 0.f) { neg_flag = true; }
}

void HelicopterRotorHead::mergeUnallocatedControl(
	const RotorSaturationFlags &flags,
	control_allocator_status_s &status)
{
	// 用 |= 合并，两个旋翼头都可能贡献饱和
	auto set = [](float & field, bool pos, bool neg) {
		if (pos) { field =  1.f; }

		else if (neg) { field = -1.f; }

		// 不清零：另一个旋翼头可能已经写过
	};

	if (flags.roll_pos || flags.roll_neg) {
		set(status.unallocated_torque[0], flags.roll_pos, flags.roll_neg);
	}

	if (flags.pitch_pos || flags.pitch_neg) {
		set(status.unallocated_torque[1], flags.pitch_pos, flags.pitch_neg);
	}

	if (flags.yaw_pos || flags.yaw_neg) {
		set(status.unallocated_torque[2], flags.yaw_pos, flags.yaw_neg);
	}

	if (flags.thrust_pos || flags.thrust_neg) {
		set(status.unallocated_thrust[2], flags.thrust_pos, flags.thrust_neg);
	}
}
