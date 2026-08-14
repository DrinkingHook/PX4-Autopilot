/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "FirstOrderHoldAltitude.hpp"

#include <math.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

float calculateFirstOrderHoldAltitude(const double target_lat, const double target_lon, const float target_altitude,
				      const double current_lat, const double current_lon, const float current_altitude,
				      const float acc_rad, FirstOrderHoldAltitudeState &state)
{
	// The target altitude is considered reached within the acceptance radius of the target.
	// 翻译：当飞行器进入目标点的允许误差半径(acceptance radius)内时，即视为已达到目标高度
	const float completion_radius = acc_rad;

	const float d_curr = get_distance_to_next_waypoint(target_lat, target_lon, current_lat, current_lon);

	// Start a new ramp whenever the target altitude changes (a genuinely new altitude setpoint). Ramp updates
	// that keep the same target altitude leave the ongoing ramp untouched so it keeps progressing smoothly.
	// 翻译：每当目标高度发生变化(即出现真正新的高度设定点)时，启动新的斜坡计算。若更新指令保持目标高度不变，则不改变当前斜坡状态，使其继续平滑进行
	const bool new_target = !PX4_ISFINITE(state.target_altitude)
				|| fabsf(target_altitude - state.target_altitude) > FLT_EPSILON;

	if (new_target) {
		state.target_altitude = target_altitude;
		// Always start the ramp from the current (measured) altitude.
		state.ramp_start_altitude = current_altitude;
		state.ramp_start_distance = d_curr;
		state.min_distance = d_curr;
	}

	// Track the closest horizontal approach so the ramp only ever progresses toward the target.
	// 翻译：追踪最近的水平接近点，确保斜坡变化始终朝着目标方向进行
	state.min_distance = math::min(state.min_distance, d_curr);

	float position_sp_alt = target_altitude;

	// Only ramp if the target was still outside the completion radius when the ramp started, otherwise there is
	// no meaningful distance to interpolate over and we command the target altitude directly.
	// 翻译：仅在斜坡启动时目标位于完成半径(completion radius)之外的情况下才执行斜坡变化；否则，因无有效的插值距离，直接指令目标高度即可
	if (state.ramp_start_distance > completion_radius) {
		// The setpoint is interpolated linearly from the ramp start altitude (at the ramp start distance) to the
		// target altitude (reached at the completion radius around the target).
		// 翻译：设定值通过线性插值计算得出：起点为斜坡启动时的高度(对应启动距离)，终点为目标高度(在目标点周围的完成半径处达到)
		const float grad = (target_altitude - state.ramp_start_altitude) / (completion_radius - state.ramp_start_distance);
		const float progress_distance = math::constrain(state.min_distance, completion_radius, state.ramp_start_distance);
		position_sp_alt = target_altitude + grad * (progress_distance - completion_radius);
	}

	return position_sp_alt;
}
