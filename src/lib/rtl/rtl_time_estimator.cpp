/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file rtl_time_estimator.cpp
 *
 * Helper class to calculate the remaining time estimate to go to RTL landing point.
 *
 */

#include "rtl_time_estimator.h"

#include <float.h>
#include <math.h>

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_status.h>

RtlTimeEstimator::RtlTimeEstimator() : ModuleParams(nullptr)
{
	_param_mpc_z_v_auto_up = param_find("MPC_Z_V_AUTO_UP");
	_param_mpc_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
	_param_fw_climb_rate = param_find("FW_T_CLMB_R_SP");
	_param_fw_sink_rate = param_find("FW_T_SINK_R_SP");
	_param_fw_airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_param_mpc_xy_cruise = param_find("MPC_XY_CRUISE");
	_param_rover_cruise_speed = param_find("RO_SPEED_LIM");
};

rtl_time_estimate_s RtlTimeEstimator::getEstimate() const
{
	rtl_time_estimate_s time_estimate{};

	if (_is_valid && PX4_ISFINITE(_time_estimate)) {
		time_estimate.valid = true;
		time_estimate.time_estimate = _time_estimate;
		// Use actual time estimate to compute the safer time estimate with additional scale factor and a margin
		// 翻译：使用实际时间估计来计算更安全的时间估计，使用额外的缩放因子和余量
		time_estimate.safe_time_estimate = _param_rtl_time_factor.get() * _time_estimate + _param_rtl_time_margin.get();

	} else {
		time_estimate.valid = false;
	}

	time_estimate.timestamp = hrt_absolute_time();
	return time_estimate;
}

void RtlTimeEstimator::update()
{
	_wind_sub.update();

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		// 翻译：如果任何参数更新，调用updateParams()检查是否需要更新此类属性（并执行更新）。
		updateParams();
	}
}

/**
 * @brief 添加垂直距离估计
 * @param alt 垂直距离
 */
void RtlTimeEstimator::addVertDistance(float alt)
{
	if (PX4_ISFINITE(alt)) {
		_is_valid = true;

		_time_estimate += calcVertTimeEstimate(alt);
	}
}

/**
 * @brief 添加水平距离估计
 * @param hor_dist 水平距离
 * @param direction 方向
 * @param vert_dist 垂直距离
 */
void RtlTimeEstimator::addDistance(float hor_dist, const matrix::Vector2f &direction, float vert_dist)
{
	if (PX4_ISFINITE(hor_dist) && PX4_ISFINITE(vert_dist)) {
		_is_valid = true;

		float hor_time_estimate{0.f};

		if (hor_dist > FLT_EPSILON && PX4_ISFINITE(hor_dist)) {
			hor_time_estimate = hor_dist / getCruiseGroundSpeed(direction.normalized());
		}

		float ver_time_estimate{calcVertTimeEstimate(vert_dist)};

		_time_estimate += math::max(hor_time_estimate, ver_time_estimate);

	}
}

void RtlTimeEstimator::addWait(float time_s)
{
	if (PX4_ISFINITE(time_s)) {
		_is_valid = true;

		if (time_s > FLT_EPSILON) {
			_time_estimate += time_s;
		}
	}
}

/**
 * @brief 计算巡航速度
 * @param direction_norm 方向向量的单位向量
 * @return float 巡航速度
 */
float RtlTimeEstimator::getCruiseGroundSpeed(const matrix::Vector2f &direction_norm)
{
	// 根据不同机型获取巡航速度
	float cruise_speed = getCruiseSpeed();

	// 固定翼机型特殊处理
	if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		matrix::Vector2f wind = get_wind();

		const float wind_along_dir = wind.dot(direction_norm);
		const float wind_across_dir = matrix::Vector2f(wind - direction_norm * wind_along_dir).norm();

		// Note: use fminf so that we don't _rely_ on tailwind towards direction to make RTL more efficient
		// 翻译：Note: 使用fminf，以便我们不要依赖于方向上的逆风来使RTL更有效率
		const float ground_speed = sqrtf(cruise_speed * cruise_speed - wind_across_dir * wind_across_dir) + fminf(
						   0.f, wind_along_dir);

		cruise_speed = ground_speed;
	}

	return cruise_speed;
}

/**
 * @brief 计算垂直时间估计值
 * @param alt 高度
 * @return float 垂直时间估计值
 */
float RtlTimeEstimator::calcVertTimeEstimate(float alt)
{
	float vertical_rate{0.1f};
	float time_estimate{0.f};

	if (alt > FLT_EPSILON) {
		vertical_rate = getClimbRate();

	} else {
		vertical_rate = getDescendRate();
	}

	float abs_alt = fabsf(alt);

	if (abs_alt > FLT_EPSILON) {
		time_estimate = abs_alt / vertical_rate;
	}

	return time_estimate;
}

/**
 * @brief 计算巡航速度
 * @return float 巡航速度
 */
float RtlTimeEstimator::getCruiseSpeed()
{
	float ret = 1e6f;

	if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_xy_cruise == PARAM_INVALID || param_get(_param_mpc_xy_cruise, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_airspeed_trim == PARAM_INVALID || param_get(_param_fw_airspeed_trim, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		if (_param_rover_cruise_speed == PARAM_INVALID || param_get(_param_rover_cruise_speed, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

/**
 * @brief 计算风速
 * @return matrix::Vector2f 风速向量
 */
matrix::Vector2f RtlTimeEstimator::get_wind()
{
	_wind_sub.update();
	matrix::Vector2f wind{};

	if (hrt_absolute_time() - _wind_sub.get().timestamp < 1_s) {
		wind(0) = _wind_sub.get().windspeed_north;
		wind(1) = _wind_sub.get().windspeed_east;
	}

	return wind;
}

/**
 * @brief 计算下降速度
 * @return float 下降速度
 */
float RtlTimeEstimator::getClimbRate()
{
	float ret = 1e6f;

	if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_up == PARAM_INVALID || param_get(_param_mpc_z_v_auto_up, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		if (_param_fw_climb_rate == PARAM_INVALID || param_get(_param_fw_climb_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}

/**
 * @brief 计算下降速度
 * @return float 下降速度
 */
float RtlTimeEstimator::getDescendRate()
{
	float ret = 1e6f;

	if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_param_mpc_z_v_auto_dn == PARAM_INVALID || param_get(_param_mpc_z_v_auto_dn, &ret) != PX4_OK) {
			ret = 1e6f;
		}

	} else if (_vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		if (_param_fw_sink_rate == PARAM_INVALID || param_get(_param_fw_sink_rate, &ret) != PX4_OK) {
			ret = 1e6f;
		}
	}

	return ret;
}
