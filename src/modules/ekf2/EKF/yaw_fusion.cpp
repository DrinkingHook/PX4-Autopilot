/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

#include <ekf_derivation/generated/compute_yaw_innov_var_and_h.h>

#include <mathlib/mathlib.h>

/**
 * @brief 融合航向
 * @param aid_src_status 融合状态
 * @param H_YAW 状态向量
 * @param reset 是否重置
 */
bool Ekf::fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW, bool reset)
{
	// check if the innovation variance calculation is badly conditioned
	// 翻译：检查创新方差计算是否条件不良
	if (aid_src_status.innovation_variance >= aid_src_status.observation_variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		// 翻译：创新方差贡献来自状态协方差不是负数，没有故障
		_fault_status.flags.bad_hdg = false;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		// 翻译：创新方差贡献来自状态协方差是负数，这意味着协方差矩阵条件不良
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		// 翻译：我们重新初始化协方差矩阵并放弃此融合步骤
		initialiseCovariance();
		ECL_ERR("yaw fusion numerical error - covariance reset");

		return false;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion;
	const float heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;

	for (uint8_t row = 0; row < State::size; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (reset && fabsf(H_YAW(State::quat_nominal.idx + 2)) > FLT_EPSILON) {
		// Reset the yaw estimate by forcing the measurement into the state
		Kfusion(State::quat_nominal.idx + 2) = 1.f / H_YAW(State::quat_nominal.idx + 2);
	}

	// set the heading unhealthy if the test fails
	if (aid_src_status.innovation_rejected) {
		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air
		    && isTimedOut(_time_last_in_air, (uint64_t)5e6)
		    && isTimedOut(aid_src_status.time_last_fuse, (uint64_t)1e6)
		   ) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			const float gate_sigma = math::max(_params.ekf2_hdg_gate, 1.f);
			const float gate_limit = sqrtf((sq(gate_sigma) * aid_src_status.innovation_variance));
			aid_src_status.innovation = math::constrain(aid_src_status.innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			// 翻译：此外，还要重置偏航陀螺仪方差，以加快收敛速度，避免卡在之前的错误估计值上
			resetGyroBiasZCov();

		} else {
			return false;
		}
	}

	measurementUpdate(Kfusion, H_YAW, aid_src_status.observation_variance, aid_src_status.innovation);

	_time_last_heading_fuse = _time_delayed_us;

	aid_src_status.time_last_fuse = _time_delayed_us;
	aid_src_status.fused = true;

	_fault_status.flags.bad_hdg = false;

	return true;
}

/**
 * @brief 计算偏航角的创新方差和观测矩阵
 * @param observation_variance 观测方差
 * @param innovation_variance 创新方差
 * @param H_YAW 观测矩阵
 */
void Ekf::computeYawInnovVarAndH(float observation_variance, float &innovation_variance, VectorState &H_YAW) const
{
	sym::ComputeYawInnovVarAndH(_state.vector(), P, observation_variance, &innovation_variance, &H_YAW);
}

/**
 * @brief 重置姿态状态的偏航角
 * @param yaw 偏航角
 * @param yaw_variance 偏航角方差
 */
void Ekf::resetQuatStateYaw(const float yaw, const float yaw_variance)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	// 翻译：保存当前的四元数状态，以便稍后计算重置变化量
	const Quatf quat_before_reset = _state.quat_nominal;

	// update the yaw angle variance
	// 翻译：更新偏航角方差
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		P.uncorrelateCovarianceSetVariance<1>(2, yaw_variance);
	}

	// update transformation matrix from body to world frame using the current estimate
	// update the rotation matrix using the new yaw value
	// 翻译：使用当前估计值更新从本体坐标系到世界坐标系的变换矩阵
	// 	使用新的偏航值更新旋转矩阵
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// update quaternion states
	// 翻译：更新四元数状态
	_state.quat_nominal = Quatf(_R_to_earth);

	_time_last_heading_fuse = _time_delayed_us;

	// 对滤波器状态和协方差进行一致性传播
	propagateQuatReset(quat_before_reset);

	// rotate horizontal velocity by the yaw change
	// 翻译：旋转水平速度以匹配偏航角变化
	const float yaw_diff = wrap_pi(yaw - getEulerYaw(quat_before_reset));
	resetHorizontalVelocityToMatchYaw(yaw_diff);
}

/**
 * @brief 在四元数被外部重置后，对滤波器状态和协方差进行一致性传播
 *
 * 当姿态四元数被外部模块（通常是视觉系统、回环、初始化、对齐等）突然强制修改（reset）时，
 * 本函数负责将这个突变合理地传播到EKF的协方差矩阵以及相关的状态量（如速度、位置、偏置等），
 * 以保证滤波器在重置前后保持统计一致性，避免协方差过度乐观或滤波器发散。
 *
 * @param quat_before_reset 重置发生之前的状态四元数（通常是调用重置前的备份值）
 *
 * @note
 * - 该函数通常在视觉前端或全局优化完成后、直接对状态四元数赋值之后调用
 * - 常见的调用场景包括：VIO关键帧切换、重定位成功、回环闭合、IMU初始化对齐、重力/磁场对齐等
 * - 函数内部会计算重置前后的四元数差（delta rotation），并将其作用于协方差和部分状态
 *
 * @warning
 * - 调用本函数前，必须已经将新的四元数写入滤波器状态（state.quat）
 * - 不应在没有发生四元数跳变的情况下调用此函数
 */
void Ekf::propagateQuatReset(const Quatf &quat_before_reset)
{
	const Quatf q_error((_state.quat_nominal * quat_before_reset.inversed()).normalized());

	// add the reset amount to the output observer buffered data
	// 翻译：将重置量添加到输出观察器缓冲数据中
	_output_predictor.resetQuaternion(q_error);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	// update EV attitude error filter
	// 翻译：更新EV姿态误差滤波器
	if (_ev_q_error_initialized) {
		const Quatf ev_q_error_updated = (q_error * _ev_q_error_filt.getState()).normalized();
		_ev_q_error_filt.reset(ev_q_error_updated);
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// record the state change
	// 翻译：记录状态变化
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;

	} else {
		// there's already a reset this update, accumulate total delta
		// 翻译：已有重置，累积总差值
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;
		_state_reset_status.quat_change.normalize();
	}

	_state_reset_status.reset_count.quat++;
}

void Ekf::resetYawByFusion(const float yaw, const float yaw_variance)
{
	const Quatf quat_before_reset = _state.quat_nominal;

	estimator_aid_source1d_s aid_src_status{};
	aid_src_status.observation = yaw;
	aid_src_status.observation_variance = yaw_variance;
	aid_src_status.innovation = wrap_pi(getEulerYaw(_state.quat_nominal) - yaw);

	VectorState H_YAW;

	computeYawInnovVarAndH(aid_src_status.observation_variance, aid_src_status.innovation_variance, H_YAW);

	const bool reset_yaw = true;
	fuseYaw(aid_src_status, H_YAW, reset_yaw);

	propagateQuatReset(quat_before_reset);

	resetHorizontalVelocityToMatchYaw(-aid_src_status.innovation);
}

void Ekf::resetHorizontalVelocityToMatchYaw(const float delta_yaw)
{
	if (!isNorthEastAidingActive() && fabsf(delta_yaw) > 0.3f) {
		const matrix::Dcm2f R_yaw(delta_yaw);
		const Vector2f vel_rotated = R_yaw * Vector2f(_state.vel);
		const float vel_var = fmaxf(P(State::vel.idx, State::vel.idx), P(State::vel.idx + 1, State::vel.idx + 1));
		resetHorizontalVelocityTo(vel_rotated, vel_var);
	}
}
