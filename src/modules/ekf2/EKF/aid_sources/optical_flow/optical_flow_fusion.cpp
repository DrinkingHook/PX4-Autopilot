/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file optical_flow_fusion.cpp
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_flow_y_innov_var_and_h.h>

/**
 * @brief 融合光流
 * 
 */
bool Ekf::fuseOptFlow(VectorState &H, const bool update_terrain)
{
	const auto state_vector = _state.vector();

	// if either axis fails we abort the fusion
	// 翻译：如果任意轴融合失败则终止
	if (_aid_src_optical_flow.innovation_rejected) {
		return false;
	}

	// fuse observation axes sequentially
	// 翻译：按顺序融合观测轴
	// 只处理XY轴
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed before
			// 翻译：所有内容之前都已经计算好了

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			// 翻译：重新计算创新方差，因为状态协方差因之前的融合而发生变化（线性化，所有轴使用相同的初始状态）
			const float R_LOS = _aid_src_optical_flow.observation_variance[1];
			const float epsilon = 1e-3f;
			sym::ComputeFlowYInnovVarAndH(state_vector, P, R_LOS, epsilon, &_aid_src_optical_flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			// 翻译：使用更新后的状态重新计算创新
			const Vector3f flow_gyro_corrected = _flow_sample_delayed.gyro_rate - _flow_gyro_bias;
			_aid_src_optical_flow.innovation[1] = predictFlow(flow_gyro_corrected)(1) - static_cast<float>
							      (_aid_src_optical_flow.observation[1]);

			// recalculate the test ratio as the measurement jacobian is highly non linear
			// when close to the ground (singularity at 0) and the innovation can suddenly become really
			// large and destabilize the filter
			// 翻译：重新计算测试比率，因为测量雅可比矩阵在接近地面时具有高度非线性（奇异点位于 0），创新可能突然变得非常大，导致滤波器不稳定。
			_aid_src_optical_flow.test_ratio[1] = sq(_aid_src_optical_flow.innovation[1]) / (sq(
					_params.ekf2_of_gate) * _aid_src_optical_flow.innovation_variance[1]);

			if (_aid_src_optical_flow.test_ratio[1] > 1.f) {
				continue;
			}
		}

		if (_aid_src_optical_flow.innovation_variance[index] < _aid_src_optical_flow.observation_variance[index]) {
			// we need to reinitialise the covariance matrix and abort this fusion step
			// 翻译：我们需要重新初始化协方差矩阵并中止此融合步骤
			ECL_ERR("Opt flow error - covariance reset");
			initialiseCovariance();
			return false;
		}

		VectorState Kfusion = P * H / _aid_src_optical_flow.innovation_variance[index];

		if (!update_terrain) {
			Kfusion(State::terrain.idx) = 0.f;
		}

		measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index],
				  _aid_src_optical_flow.innovation[index]);
	}

	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
	_aid_src_optical_flow.fused = true;

	_time_last_hor_vel_fuse = _time_delayed_us;

	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true;
}

/**
 * @brief 预测光流高度
 * @return 预测光流高度
 */
float Ekf::predictFlowHagl() const
{
	// calculate the sensor position relative to the IMU
	// 翻译：计算传感器相对于 IMU 的位置
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the sensor position relative to the IMU in earth frame
	// 翻译：计算传感器相对于 IMU 在地球坐标系中的位置
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	// 翻译：计算光流相机的离地高度。由于地球坐标系是非均匀分布的（NED），因此地球坐标系中的正偏移会导致离地高度减小。
	const float height_above_gnd_est = fabsf(getHagl() - pos_offset_earth(2));

	// Never return a really small value to avoid generating insanely large flow innovations
	// that could destabilize the filter
	// 翻译：切勿返回过小的值，以免产生过大的流创新，从而导致滤波器不稳定。
	constexpr float min_hagl = 1e-2f;

	return fmaxf(height_above_gnd_est, min_hagl);
}
/**
 * @brief 预测光流范围
 * @return 预测光流范围
 */
float Ekf::predictFlowRange() const
{
	// calculate range from focal point to centre of image
	// absolute distance to the frame region in view
	// 翻译：计算焦点到图像中心到画面区域的绝对距离
	return predictFlowHagl() / _R_to_earth(2, 2);
}

/**
 * @brief 预测光流
 * @param flow_gyro 光流陀螺仪数据
 * @return 预测光流
 */
Vector2f Ekf::predictFlow(const Vector3f &flow_gyro) const
{
	// calculate the sensor position relative to the IMU
	// 翻译：计算传感器相对于 IMU 的位置
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: flow gyro is the negative of the body angular velocity, thus use minus sign
	// 翻译：计算传感器相对于 IMU 在机体坐标系中的速度
	// 注意：流陀螺仪角速度是机体角速度的负值，因此使用负号。
	const Vector3f vel_rel_imu_body = -flow_gyro % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	// 翻译：计算传感器在地球坐标系中的速度
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	// 翻译：旋转到机体坐标系中
	const Vector2f vel_body = _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();

	// calculate range from focal point to centre of image
	// 翻译：计算焦点到图像中心的距离
	const float scale = _R_to_earth(2, 2) / predictFlowHagl();

	return Vector2f(vel_body(1) * scale, -vel_body(0) * scale);
}

/**
 * @brief 计算光流观测误差
 * @param flow_sample 光流样本
 * @return 光流观测误差
 */
float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample) const
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	// 翻译：计算观测噪声方差 - 在光流质量范围内线性缩放噪声
	const float R_LOS_best = fmaxf(_params.ekf2_of_n_min, 0.05f);
	const float R_LOS_worst = fmaxf(_params.ekf2_of_n_max, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	// 翻译：计算一个权重，该权重在流质量最佳时为 1，在流质量最差时为 0计算一个权重，该权重在流质量最佳时为 1，在流质量最差时为 0
	float weighting = (255.f - (float)_params.ekf2_of_qmin);

	if (weighting >= 1.f) {
		weighting = math::constrain((float)(flow_sample.quality - _params.ekf2_of_qmin) / weighting, 0.f, 1.f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	// 翻译：取最佳和最差流质量下观测噪声的加权平均值
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS;
}
