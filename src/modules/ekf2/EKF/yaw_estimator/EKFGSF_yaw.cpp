/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
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

#include "EKFGSF_yaw.h"

#include <cstdlib>

#include <lib/geo/geo.h> // CONSTANTS_ONE_G

#include "derivation/generated/yaw_est_predict_covariance.h"
#include "derivation/generated/yaw_est_compute_measurement_update.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;
using math::Utilities::getEulerYaw;
using math::Utilities::updateYawInRotMat;

EKFGSF_yaw::EKFGSF_yaw()
{
	reset();
}

void EKFGSF_yaw::reset()
{
	_ekf_gsf_vel_fuse_started = false;

	_gsf_yaw_variance = INFINITY;
}

/**
 * @brief 预测
 * @param delta_ang 角度变化
 * @param delta_ang_dt 角度变化速率
 * @param delta_vel 速度变化
 * @param delta_vel_dt 速度变化速率
 * @param in_air 是否在空中
 */
void EKFGSF_yaw::predict(const matrix::Vector3f &delta_ang, const float delta_ang_dt, const matrix::Vector3f &delta_vel,
			 const float delta_vel_dt, bool in_air)
{
	const Vector3f accel = delta_vel / delta_vel_dt;

	if (delta_vel_dt > 0.001f) {
		// to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
		// 翻译：为降低振动的影响，使用低通滤波器进行滤波，其时间常数为AHRS倾斜校正时间常数的1/10。
		const float filter_coef = fminf(10.f * delta_vel_dt * _tilt_gain, 1.f);
		_ahrs_accel = _ahrs_accel * (1.f - filter_coef) + accel * filter_coef;

	} else {
		return;
	}

	// Initialise states first time
	// 翻译：首次初始化状态
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		// due to vehicle movement
		// 翻译：检查加速度是否过大，以降低因飞行器运动导致初始横滚/俯仰误差过大的可能性
		const float accel_norm_sq = accel.norm_squared();
		const float accel_lpf_norm_sq = _ahrs_accel.norm_squared();

		static constexpr float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		static constexpr float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit))
					 && (accel_lpf_norm_sq > sq(lower_accel_limit)) && (accel_lpf_norm_sq < sq(upper_accel_limit));

		if (ok_to_align) {
			ahrsAlignTilt(delta_vel);
			_ahrs_ekf_gsf_tilt_aligned = true;

		} else {
			return;
		}
	}

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index, delta_ang, delta_ang_dt, delta_vel, delta_vel_dt, in_air);
	}
}

/**
 * @brief 融合速度观测值
 * @param vel_NE 观测速度向量
 * @param vel_accuracy 观测速度精度
 * @param in_air 是否在空中
 */
void EKFGSF_yaw::fuseVelocity(const Vector2f &vel_NE, const float vel_accuracy, const bool in_air)
{
	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	// 翻译：我们不会开始运行EKF部分的算法，直到有常规的观测值
	if (!_ekf_gsf_vel_fuse_started) {

		initialiseEKFGSF(vel_NE, vel_accuracy);

		ahrsAlignYaw();

		// don't start until in air or velocity is not negligible
		// 翻译：只有在空中或速度不可忽略时才启动
		if (in_air || vel_NE.longerThan(vel_accuracy)) {
			_ekf_gsf_vel_fuse_started = true;
		}

	} else {
		bool bad_update = false;

		// 循环更新所有模型的EKF，如果有一个模型的EKF更新失败，那么bad_update = true
		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			// subsequent measurements are fused as direct state observations
			// 翻译：后续测量结果将作为直接状态观测值进行融合
			if (!updateEKF(model_index, vel_NE, vel_accuracy)) {
				bad_update = true;
			}
		}

		// 如果为true 那么则放弃 本轮的权重计算
		if (!bad_update) {
			float total_weight = 0.0f;
			// calculate weighting for each model assuming a normal distribution
			// 翻译：假设正态分布，计算每个模型的权重。
			const float min_weight = 1e-5f;

			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index);

				if (_model_weights(model_index) < min_weight) {
					_model_weights(model_index) = min_weight;
				}

				total_weight += _model_weights(model_index);
			}

			// normalise the weighting function
			// 翻译：归一化权重函数
			_model_weights /= total_weight;
		}

		// Calculate a composite yaw vector as a weighted average of the states for each model.
		// To avoid issues with angle wrapping, the yaw state is converted to a vector with length
		// equal to the weighting value before it is summed.
		// 翻译：计算每个模型状态的加权平均值，得到复合偏航向量。
		// 	为避免角度缠绕问题，在求和之前，将偏航状态转换为长度等于权重值的向量。
		Vector2f yaw_vector;

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
			yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
			yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
		}

		_gsf_yaw = atan2f(yaw_vector(1), yaw_vector(0));

		// calculate a composite variance for the yaw state from a weighted average of the variance for each model
		// models with larger innovations are weighted less
		// 翻译：根据每个模型方差的加权平均值，计算偏航状态的复合方差。创新值较大的模型权重较小
		_gsf_yaw_variance = 0.0f;

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
			const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
			_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2, 2) + yaw_delta * yaw_delta);
		}

		if (_gsf_yaw_variance <= 0.f || !PX4_ISFINITE(_gsf_yaw_variance)) {
			reset();
		}
	}
}

/**
 * @brief AHRS预测
 * @param model_index 模型索引
 * @param delta_ang 角度变化量
 * @param delta_ang_dt 角度变化时间
 */
void EKFGSF_yaw::ahrsPredict(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt)
{
	// generate attitude solution using simple complementary filter for the selected model
	// 翻译：生成姿态解决方案，使用简单的互补滤波器为选定的模型
	const Vector3f ang_rate = delta_ang / fmaxf(delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	const Vector3f gravity_direction_bf = _ahrs_ekf_gsf[model_index].q.inversed().dcm_z();

	const float ahrs_accel_norm = _ahrs_accel.norm();

	// gain from accel vector tilt error to rate gyro correction used by AHRS calculation
	// 翻译：加速度向量倾斜误差到速率陀螺校正的增益，用于AHRS计算
	const float ahrs_accel_fusion_gain = ahrsCalcAccelGain();

	// Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	// During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	// 翻译：使用加速度数据进行角速度校正，并随着加速度幅值偏离 1g 而减小校正量（减少车辆被抬起和移动时的漂移）。
	// 	在固定翼飞行期间，假设协调转弯且 X 轴向前，补偿向心加速度。
	Vector3f tilt_correction{};

	if (ahrs_accel_fusion_gain > 0.f) {

		Vector3f accel = _ahrs_accel;

		if (PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON)) {
			// Calculate body frame centripetal acceleration with assumption X axis is aligned with the airspeed vector
			// Use cross product of body rate and body frame airspeed vector
			// 翻译：假设 X 轴与空速矢量对齐，计算机体坐标系下的向心加速度。
			// 	使用机体速率与机体坐标系下空速矢量的叉积。
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// correct measured accel for centripetal acceleration
			// 翻译：校正向心加速度测量值
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * ahrs_accel_fusion_gain / ahrs_accel_norm;
	}

	// Gyro bias estimation
	// 翻译：陀螺仪偏置估计
	constexpr float ekf2_gyr_b_limit = 0.05f;
	const float spin_rate = ang_rate.length();

	if (spin_rate < math::radians(10.f)) {
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias,
						       -ekf2_gyr_b_limit, ekf2_gyr_b_limit);
	}

	// delta angle from previous to current frame
	// 翻译：从上一帧到当前帧的角位移
	const Vector3f delta_angle_corrected = delta_ang
					       + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) * delta_ang_dt;

	// Apply delta angle to attitude
	// 翻译：将角位移应用于姿态
	const Quatf dq(AxisAnglef{delta_angle_corrected});
	_ahrs_ekf_gsf[model_index].q = (_ahrs_ekf_gsf[model_index].q * dq).normalized();
}

/**
 * @brief AHRS 对齐 倾斜
 * @param delta_vel 速度增量
 */
void EKFGSF_yaw::ahrsAlignTilt(const Vector3f &delta_vel)
{
	// Rotation matrix is constructed directly from acceleration measurement and will be the same for
	// all models so only need to calculate it once. Assumptions are:
	// 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
	// 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.
	// 翻译：旋转矩阵直接由加速度测量值构建，且对于所有模型均相同，
	// 	因此只需计算一次。假设如下：
	// 	1) 偏航角为零 - 偏航角将在速度融合开始时针对每个模型进行调整。
	// 	2) 车辆未加速，因此所有测量的加速度均由重力引起。

	// The tilt is simply the rotation between the measured gravity and the vertical axis
	// 翻译：倾斜角是指测得的重力方向与垂直轴之间的旋转角度
	Quatf q(delta_vel, Vector3f(0.f, 0.f, -1.f));

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].q = q;
	}
}

/**
 * @brief AHRS 对齐 偏航角
 */
void EKFGSF_yaw::ahrsAlignYaw()
{
	// Align yaw angle for each model
	// 翻译：调整每个模型的偏航角
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		const float yaw = wrap_pi(_ekf_gsf[model_index].X(2));
		const Dcmf R(_ahrs_ekf_gsf[model_index].q);
		_ahrs_ekf_gsf[model_index].q = Quatf(updateYawInRotMat(yaw, R));
	}
}

/**
 * @brief 预测 EKF
 * @param model_index 模型索引
 * @param delta_ang 角度变化量
 * @param delta_ang_dt 角度变化率
 * @param delta_vel 速度变化量
 * @param delta_vel_dt 速度变化率
 * @param in_air 是否在空中
 */
void EKFGSF_yaw::predictEKF(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt,
			    const Vector3f &delta_vel, const float delta_vel_dt, bool in_air)
{
	// generate an attitude reference using IMU data
	// 翻译：使用 IMU 数据生成姿态参考
	ahrsPredict(model_index, delta_ang, delta_ang_dt);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	// 翻译：我们不开始运行 EKF 算法的后半部分，直到有常规的加速度观测
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	const Dcmf R(_ahrs_ekf_gsf[model_index].q);

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	// 翻译：使用水平投影计算偏航状态，以避免万向节锁死
	_ekf_gsf[model_index].X(2) = getEulerYaw(R);

	// calculate delta velocity in a horizontal front-right frame
	// 翻译：计算水平前右坐标系中的速度增量
	const Vector3f del_vel_NED = R * delta_vel;
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2));
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2));
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw;
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw;
	const float daz = Vector3f(R * delta_ang)(2);

	// delta velocity process noise double if we're not in air
	// 翻译：如果车辆不在空中，则速度增量过程噪声加倍
	const float accel_noise = in_air ? _accel_noise : 2.f * _accel_noise;
	const float d_vel_var = sq(accel_noise * delta_vel_dt);

	// Use fixed values for delta angle process noise variances
	// 翻译：使用固定值作为角度增量过程噪声方差
	const float d_ang_var = sq(_gyro_noise * delta_ang_dt);

	_ekf_gsf[model_index].P = sym::YawEstPredictCovariance(_ekf_gsf[model_index].X, _ekf_gsf[model_index].P, Vector2f(dvx,
				  dvy), d_vel_var, daz, d_ang_var);

	// covariance matrix is symmetrical, so copy upper half to lower half
	// 翻译：协方差矩阵是对称的，因此将上半部分复制到下半部分
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1);
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2);
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2);

	// constrain variances
	// 翻译：约束方差
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

	// sum delta velocities in earth frame:
	// 翻译：在地球坐标系中求和速度增量：
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);
}

/**
 * @brief 更新 EKF
 * @param model_index 模型索引
 * @param vel_NE 北-东方向的水平速度向量
 * @param vel_accuracy 速度精度
 * @return 是否成功
 */
bool EKFGSF_yaw::updateEKF(const uint8_t model_index, const Vector2f &vel_NE, const float vel_accuracy)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	// 翻译：设置观测方差与 GPS 提供的精度估计值之间的偏差，并应用最小合理性检查
	const float vel_obs_var = sq(fmaxf(vel_accuracy, 0.01f));

	// calculate velocity observation innovations
	// 翻译：计算速度观测新息
	_ekf_gsf[model_index].innov = _ekf_gsf[model_index].X.xy() - vel_NE;

	matrix::Matrix<float, 3, 2> K;
	matrix::SquareMatrix<float, 3> P_new;
	matrix::SquareMatrix<float, 2> S_inverse;

	sym::YawEstComputeMeasurementUpdate(_ekf_gsf[model_index].P,
					    vel_obs_var,
					    FLT_EPSILON,
					    &S_inverse,
					    &_ekf_gsf[model_index].S_det_inverse,
					    &K,
					    &P_new);

	_ekf_gsf[model_index].P = P_new;

	// copy upper to lower diagonal
	// 翻译：将上对角线复制到下对角线
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1);
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2);
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

	// normalized innovation squared = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	_ekf_gsf[model_index].nis = _ekf_gsf[model_index].innov * (S_inverse * _ekf_gsf[model_index].innov);

	// Perform a chi-square innovation consistency test and calculate a compression scale factor
	// that limits the magnitude of innovations to 5-sigma
	// If the normalized innovation squared is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	// This protects from large measurement spikes
	if (_ekf_gsf[model_index].nis > sq(5.f)) {
		_ekf_gsf[model_index].innov *= sqrtf(sq(5.f) / _ekf_gsf[model_index].nis);
		_ekf_gsf[model_index].nis = sq(5.f);
	}

	// Correct the state vector
	// 翻译：修正状态向量
	const Vector3f delta_state = -K * _ekf_gsf[model_index].innov;
	const float yawDelta = delta_state(2);

	_ekf_gsf[model_index].X.xy() += delta_state.xy();
	_ekf_gsf[model_index].X(2) = wrap_pi(_ekf_gsf[model_index].X(2) + yawDelta);

	// Apply the change in yaw angle to the AHRS using left multiplication to rotate
	// the attitude around the earth Down axis
	// 翻译：使用左乘法将偏航角的变化应用于姿态航向参考系统 (AHRS)，以绕地球下轴旋转姿态
	const Quatf dq(cosf(yawDelta / 2.f), 0.f, 0.f, sinf(yawDelta / 2.f));
	_ahrs_ekf_gsf[model_index].q = (dq * _ahrs_ekf_gsf[model_index].q).normalized();

	return true;
}

/**
 * @brief 初始化 EKF
 * @param vel_NE 速度
 * @param vel_accuracy 速度精度
 */
void EKFGSF_yaw::initialiseEKFGSF(const Vector2f &vel_NE, const float vel_accuracy)
{
	_gsf_yaw = 0.0f;
	_gsf_yaw_variance = sq(M_PI_F / 2.f);
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // All filter models start with the same weight

	const float yaw_increment = 2.f * M_PI_F / (float)N_MODELS_EKFGSF;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ekf_gsf[model_index] = {};

		// evenly space initial yaw estimates in the region between +-Pi
		_ekf_gsf[model_index].X(2) = -M_PI_F + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// take velocity states and corresponding variance from last measurement
		_ekf_gsf[model_index].X(0) = vel_NE(0);
		_ekf_gsf[model_index].X(1) = vel_NE(1);

		_ekf_gsf[model_index].P(0, 0) = sq(fmaxf(vel_accuracy, 0.01f));
		_ekf_gsf[model_index].P(1, 1) = _ekf_gsf[model_index].P(0, 0);

		// use half yaw interval for yaw uncertainty
		_ekf_gsf[model_index].P(2, 2) = sq(0.5f * yaw_increment);
	}
}

/**
 * @brief 计算高斯密度函数值
 * @param model_index 模型索引
 * @return 高斯密度函数值
 */
float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	return (1.f / (2.f * M_PI_F)) * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * _ekf_gsf[model_index].nis);
}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			    float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw_composite = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			yaw[model_index] = _ekf_gsf[model_index].X(2);
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0);
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1);
			weight[model_index] = _model_weights(model_index);
		}

		return true;
	}

	return false;
}

/**
 * @brief ahrs 计算加速度增益
 */
float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// Calculate the acceleration fusion gain using a continuous function that is unity at 1g and zero
	// at the min and max g value. Allow for more acceleration when flying as a fixed wing vehicle using centripetal
	// acceleration correction as higher and more sustained g will be experienced.
	// Use a quadratic instead of linear function to prevent vibration around 1g reducing the tilt correction effectiveness.
	// see https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON);

	const float ahrs_accel_norm = _ahrs_accel.norm();

	if (centripetal_accel_compensation_enabled && (ahrs_accel_norm > CONSTANTS_ONE_G)) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}
