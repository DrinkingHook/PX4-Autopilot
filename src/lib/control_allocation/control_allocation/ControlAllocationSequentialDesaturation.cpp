/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationSequentialDesaturation.cpp
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "ControlAllocationSequentialDesaturation.hpp"


// 分配
void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	switch (_param_mc_airmode.get()) {
	case 1:
		mixAirmodeRP();
		break;

	case 2:
		mixAirmodeRPY();
		break;

	default:
		mixAirmodeDisabled();
		break;
	}
}

/**
 * @brief 去饱和执行器：沿 desaturation_vector 方向调整执行器输出，消除饱和
 *
 * 核心思想：分配器混出来的输出向量 actuator_sp 中，某些执行器可能超出 [min, max] 边界（饱和）。
 * 我们沿着一个"不影响指定轴控制效果"的方向 desaturation_vector（例如只沿推力轴、或只沿 yaw 轴）
 * 给输出叠加一个修正量，用牺牲该轴的方式把饱和的执行器拉回边界内。
 *
 * @param actuator_sp         执行器输出向量，会被原地修改（既是输入也是输出）
 * @param desaturation_vector 去饱和方向向量，叠加它不会改变目标轴上的力矩/推力
 * @param increase_only       为 true 时只允许沿该方向"增大"输出，不允许减小
 */
void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	// 计算去饱和增益：找到把"最饱和"的执行器恰好拉回边界所需的修正系数 k
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	// 若要求"只增不减"，但算出的增益为负（即必须减小输出才能去饱和），则本次不做任何调整
	if (increase_only && gain < 0.f) {
		return;
	}

	// 第一次去饱和：把增益乘以方向向量，叠加到每个执行器的输出上，消除最严重的饱和
	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	// 第二次去饱和：用减半的增益再算一次，处理第一次调整后新出现的轻微饱和（起到收敛效果）
	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
		// 以一半的力度再叠加一次，继续去饱和
		actuator_sp(i) += gain * desaturation_vector(i);
	}
}

float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRP()
{
	// Airmode for roll and pitch, but not yaw
	// 翻译：该空中模式支持横滚和俯仰，但不支持偏航

	// Mix without yaw
	// 翻译：不考虑偏航，直接分配
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Mix yaw independently
	mixYaw();
}

// mixAirmodeRPY（完整 Airmode）是一开始就把 roll + pitch + yaw + 推力全部混进去
// 因为 yaw 提前占用了执行器输出空间，所以更容易触顶。此时算法会优先用调整推力的方式去饱和（允许升高或降低总推力），如果还不够，再削减 yaw，从而优先保证 roll/pitch
void
ControlAllocationSequentialDesaturation::mixAirmodeRPY()
{
	// Airmode for roll, pitch and yaw

	// Do full mixing
	ActuatorVector thrust_z;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		yaw(i) = _mix(i, ControlAxis::YAW);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Unsaturate yaw (in case upper and lower bounds are exceeded)
	// to prioritize roll/pitch over yaw.
	desaturateActuators(_actuator_sp, yaw);
}

// mixAirmodeDisabled（关闭 Airmode）是先只混 roll + pitch + 推力，yaw 故意放在最后
// 去饱和时绝对不允许抬高推力，只能降低推力或削减 roll/pitch。最后才尝试加入 yaw，并且最多只允许再降低 15% 的推力来给 yaw 留一点余量。因此在高推力或接近饱和时，yaw 经常会被大幅甚至完全牺牲
void
ControlAllocationSequentialDesaturation::mixAirmodeDisabled()
{
	// Airmode disabled: never allow to increase the thrust to unsaturate a motor
	// 翻译：空中模式已禁用：绝不允许增加推力以使电机去饱和

	// Mix without yaw
	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		roll(i) = _mix(i, ControlAxis::ROLL);
		pitch(i) = _mix(i, ControlAxis::PITCH);
	}

	// only reduce thrust
	// 翻译：仅减少推力
	desaturateActuators(_actuator_sp, thrust_z, true);

	// Reduce roll/pitch acceleration if needed to unsaturate
	// 翻译：如果需要解除饱和，则减少横滚/俯仰加速度
	desaturateActuators(_actuator_sp, roll);
	desaturateActuators(_actuator_sp, pitch);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixYaw()
{
	// Add yaw to outputs
	ActuatorVector yaw;
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		yaw(i) = _mix(i, ControlAxis::YAW);
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	// Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
	// and allow some yaw response at maximum thrust
	// 翻译：如有需要，调整偏航加速度以使输出不饱和（不要改变滚转/俯仰），并在最大推力下允许一定的偏航响应
	ActuatorVector max_prev = _actuator_max;
	_actuator_max += (_actuator_max - _actuator_min) * MINIMUM_YAW_MARGIN;
	desaturateActuators(_actuator_sp, yaw);
	_actuator_max = max_prev;

	// reduce thrust only
	desaturateActuators(_actuator_sp, thrust_z, true);
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
