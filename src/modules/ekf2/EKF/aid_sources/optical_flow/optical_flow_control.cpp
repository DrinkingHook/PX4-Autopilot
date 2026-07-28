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

/**
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	_fc.of.available = (_params.ekf2_of_ctrl != 0);

	if (!_flow_buffer || !_fc.of.intended()) {
		stopFlowFusion();
		return;
	}

	VectorState H;

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	// 翻译：当样本中点落后于融合时间范围时，新的光流数据即可使用并准备融合
	if (_flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed)) {

		// flow gyro has opposite sign convention
		// 翻译：光流陀螺仪的角速率符号相反。
		_ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - getGyroBias());

		// ensure valid flow sample gyro rate before proceeding
		// 翻译：在继续操作之前，请确保有效的流量采样陀螺仪速率
		switch (static_cast<FlowGyroSource>(_params.ekf2_of_gyr_src)) {
		default:

		/* FALLTHROUGH */
		case FlowGyroSource::Auto:
			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_rate(1))) {
				_flow_sample_delayed.gyro_rate = _ref_body_rate;
			}

			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(2))) {
				// Some flow modules only provide X ind Y angular rates. If this is the case, complete the vector with our own Z gyro
				// 翻译：某些流速模块仅提供 X 和 Y 方向的角速率。如果遇到这种情况，请使用我们自己的 Z 方向陀螺仪数据补全向量。
				_flow_sample_delayed.gyro_rate(2) = _ref_body_rate(2);
			}

			break;

		case FlowGyroSource::Internal:
			_flow_sample_delayed.gyro_rate = _ref_body_rate;
			break;
		}

		const flowSample &flow_sample = _flow_sample_delayed;

		const int32_t min_quality = _control_status.flags.in_air
					    ? _params.ekf2_of_qmin
					    : _params.ekf2_of_qmin_gnd;

		const bool is_quality_good = (flow_sample.quality >= min_quality);

		bool is_tilt_good = true;

#if defined(CONFIG_EKF2_RANGE_FINDER)
		is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);
#endif // CONFIG_EKF2_RANGE_FINDER

		calcOptFlowBodyRateComp(flow_sample);

		// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
		// correct for gyro bias errors in the data used to do the motion compensation
		// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
		// 翻译：使用已去除机体角速率贡献的光流速率计算光学视线速率，并校正用于运动补偿的数据中的陀螺仪偏差误差。
		// 注意符号约定：正的视线速率表示场景绕该轴向右旋转。
		// 流速陀螺仪修正
		const Vector3f flow_gyro_corrected = flow_sample.gyro_rate - _flow_gyro_bias;
		// 光流补偿
		const Vector2f flow_compensated = flow_sample.flow_rate - flow_gyro_corrected.xy();

		// calculate the optical flow observation variance
		// 翻译：计算光流观测误差
		const float R_LOS = calcOptFlowMeasVar(flow_sample);

		const float epsilon = 1e-3f;
		Vector2f innov_var;
		sym::ComputeFlowXyInnovVarAndHx(_state.vector(), P, R_LOS, epsilon, &innov_var, &H);

		// run the innovation consistency check and record result
		// 翻译：运行创新一致性检查并记录结果
		updateAidSourceStatus(_aid_src_optical_flow,
				      flow_sample.time_us,                                 // sample timestamp
				      flow_compensated,                                    // observation
				      Vector2f{R_LOS, R_LOS},                              // observation variance
				      predictFlow(flow_gyro_corrected) - flow_compensated, // innovation
				      innov_var,                                           // innovation variance
				      math::max(_params.ekf2_of_gate, 1.f));            // innovation gate

		// logging
		// 翻译：日志记录
		// 记录光流补偿速率
		_flow_rate_compensated = flow_compensated;

		// compute the velocities in body and local frames from corrected optical flow measurement for logging only
		// 翻译：计算校正后的光流测量在身体和本地框架中的速度，仅用于日志记录
		const float range = predictFlowRange();
		_flow_vel_body(0) = -flow_compensated(1) * range;
		_flow_vel_body(1) =  flow_compensated(0) * range;

		if (_flow_counter == 0) {
			_flow_vel_body_lpf.reset(_flow_vel_body);
			_flow_rate_compensated_lpf.reset(_flow_rate_compensated);
			_flow_counter = 1;

		} else {

			_flow_vel_body_lpf.update(_flow_vel_body);
			_flow_rate_compensated_lpf.update(_flow_rate_compensated);
			_flow_counter++;
		}

		// Check if we are in-air and require optical flow to control position drift
		// 翻译：检查我们是否在空中，需要光流来控制位置漂移
		bool is_flow_required = _control_status.flags.in_air
					&& (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
					    || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow));

		// 判断是否在传感器范围内
		const bool is_within_sensor_dist = (getHagl() >= _flow_min_distance) && (getHagl() <= _flow_max_distance);

		// 幅度良好 (流样本速率合法 && 流样本速率未超最大值 && 流补偿未超最大值)
		const bool is_magnitude_good = flow_sample.flow_rate.isAllFinite()
					       && !flow_sample.flow_rate.longerThan(_flow_max_rate)
					       && !flow_compensated.longerThan(_flow_max_rate);

		const bool continuing_conditions_passing = _fc.of.intended()
				&& _control_status.flags.tilt_align
				&& is_within_sensor_dist;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_quality_good
				&& is_magnitude_good
				&& is_tilt_good
				&& (_flow_counter > 10)
				&& (isHeightAboveGroundEstimateValid() || isHorizontalAidingActive())
				&& isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)2e6); // Prevent rapid switching

		// If the height is relative to the ground, terrain height cannot be observed.
		// 翻译：如果高度相对于地面，则无法观察地形高度。
		_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE);

		if (_control_status.flags.opt_flow) {
			if (continuing_conditions_passing) {

				if (is_quality_good && is_magnitude_good && is_tilt_good) {
					fuseOptFlow(H, _control_status.flags.opt_flow_terrain);
				}

				// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
				// 翻译：处理我们有光学流，依赖它，但没有使用它很长时间的情况。
				if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)) {
					if (is_flow_required && is_quality_good && is_magnitude_good) {
						resetFlowFusion(flow_sample);

						if (_control_status.flags.opt_flow_terrain && !isTerrainEstimateValid()) {
							resetTerrainToFlow();
						}

					} else {
						stopFlowFusion();
					}
				}

			} else {
				stopFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// If the height is relative to the ground, terrain height cannot be observed.
				// 翻译：如果高度相对于地面，地形高度不能被观察到。
				_control_status.flags.opt_flow_terrain = (_height_sensor_ref != HeightSensor::RANGE);

				if (isHorizontalAidingActive()) {
					if (fuseOptFlow(H, _control_status.flags.opt_flow_terrain)) {
						ECL_INFO("starting optical flow");
						_control_status.flags.opt_flow = true;

					} else if (_control_status.flags.opt_flow_terrain && !_control_status.flags.rng_terrain) {
						ECL_INFO("starting optical flow, resetting terrain");
						resetTerrainToFlow();
						_control_status.flags.opt_flow = true;
					}

				} else {
					if (isHeightAboveGroundEstimateValid()) {
						ECL_INFO("starting optical flow, resetting");
						resetFlowFusion(flow_sample);
						_control_status.flags.opt_flow = true;

					} else if (_control_status.flags.opt_flow_terrain) {
						ECL_INFO("starting optical flow, resetting terrain");
						resetTerrainToFlow();
						_control_status.flags.opt_flow = true;
					}
				}

				_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE);
			}
		}

	} else if (_control_status.flags.opt_flow && isTimedOut(_flow_sample_delayed.time_us, _params.reset_timeout_max)) {
		stopFlowFusion();
	}
}

void Ekf::resetFlowFusion(const flowSample &flow_sample)
{
	ECL_INFO("reset velocity to flow");
	_information_events.flags.reset_vel_to_flow = true;

	const float flow_vel_var = sq(predictFlowRange()) * calcOptFlowMeasVar(flow_sample);
	resetHorizontalVelocityTo(getFilteredFlowVelNE(), flow_vel_var);

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow);
}

/**
 * @brief 重置地形到流
 */
void Ekf::resetTerrainToFlow()
{
	ECL_INFO("reset hagl to flow");

	float new_terrain = -_gpos.altitude() + _params.ekf2_min_rng;

	if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {
		// ||vel_NE|| = ||( R * flow_body * range).xy()||
		// range = ||vel_NE|| / ||P * R * flow_body||
		constexpr float kProjXY[2][3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
		const matrix::Matrix<float, 2, 3> proj(kProjXY);

		const Vector3f flow_body(-_flow_rate_compensated_lpf.getState()(1), _flow_rate_compensated_lpf.getState()(0), 0.f);
		const float denom = Vector2f(proj * _R_to_earth * flow_body).norm();

		if (denom > 1e-6f) {
			const float range = _state.vel.xy().norm() / denom;
			new_terrain = -_gpos.altitude() + max(range, _params.ekf2_min_rng);
		}
	}

	const float delta_terrain = new_terrain - _state.terrain;
	_state.terrain = new_terrain;
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 100.f);

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow);

	// record the state change
	// 翻译：记录状态变化
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		// 翻译：已经有一个重置，累积总增量
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("stopping optical flow fusion");
		_control_status.flags.opt_flow = false;
		_control_status.flags.opt_flow_terrain = false;

		_fault_status.flags.bad_optflow_X = false;
		_fault_status.flags.bad_optflow_Y = false;

		_flow_counter = 0;
	}
}

/**
 * @brief 计算光学流的陀螺偏置估计
 * @param flow_sample 光学流样本
 */
void Ekf::calcOptFlowBodyRateComp(const flowSample &flow_sample)
{
	// calculate the bias estimate using a combined LPF and spike filter
	// 翻译：使用结合LPF和尖峰滤波器的计算偏差估计
	_flow_gyro_bias = 0.99f * _flow_gyro_bias
			  + 0.01f * matrix::constrain(flow_sample.gyro_rate - _ref_body_rate, -0.1f, 0.1f);
}
