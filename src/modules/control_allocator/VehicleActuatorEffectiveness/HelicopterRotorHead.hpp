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

#pragma once

#include "HelicopterGeometry.hpp"
#include <matrix/matrix/math.hpp>
#include <uORB/topics/control_allocator_status.h>

// ActuatorVector 的原始定义
static constexpr int NUM_ACTUATORS = 16;
using ActuatorVector = matrix::Vector<float, NUM_ACTUATORS>;

class HelicopterRotorHead
{
public:
	explicit HelicopterRotorHead(const RotorGeometry &geometry)
		: _geometry(geometry) {}

	void setGeometry(const RotorGeometry &g) { _geometry = g; }
	const RotorGeometry &geometry() const    { return _geometry; }

	RotorSaturationFlags computeSetpoints(
		float collective_pitch,
		float roll_sp,
		float pitch_sp,
		float yaw_sp,
		int   start,
		ActuatorVector &actuator_sp,
		const ActuatorVector &actuator_min,
		const ActuatorVector &actuator_max) const;

	static void mergeUnallocatedControl(
		const RotorSaturationFlags &flags,
		control_allocator_status_s &status);

	float getLinearServoOutput(float input) const;

private:
	static void setSaturationFlag(float coeff, bool &pos_flag, bool &neg_flag);
	RotorGeometry _geometry{};
};
