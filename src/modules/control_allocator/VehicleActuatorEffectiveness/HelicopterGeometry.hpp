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

// HelicopterGeometry.hpp
#pragma once
#include <stdint.h>

static constexpr int NUM_SWASH_PLATE_SERVOS_MAX = 4;
static constexpr int NUM_CURVE_POINTS           = 5;

struct SwashPlateGeometry {
	float angle{0.f};
	float arm_length{1.f};
	float trim{0.f};
};

struct RotorGeometry {
	SwashPlateGeometry swash_plate_servos[NUM_SWASH_PLATE_SERVOS_MAX];
	int32_t num_swash_plate_servos{3};
	float   throttle_curve[NUM_CURVE_POINTS] {};
	float   pitch_curve[NUM_CURVE_POINTS] {};
	float   yaw_collective_pitch_scale{0.f};
	float   yaw_collective_pitch_offset{0.f};
	float   yaw_throttle_scale{0.f};
	float   yaw_sign{1.f};
	float   spoolup_time{1.f};
	int     linearize_servos{0};
	float   max_servo_height{0.f};
	float   inverse_max_servo_throw{0.f};
};

struct RotorSaturationFlags {
	bool roll_pos{false};
	bool roll_neg{false};
	bool pitch_pos{false};
	bool pitch_neg{false};
	bool yaw_pos{false};
	bool yaw_neg{false};
	bool thrust_pos{false};
	bool thrust_neg{false};
};
