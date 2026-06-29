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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

static __attribute__((noinline)) Quatf qmul(const Quatf &a, const Quatf &b) { return a * b; }
static __attribute__((noinline)) Quatf qinv(const Quatf &q) { return q.inversed(); }
static __attribute__((noinline)) Vector3f qzaxis(const Quatf &q) { return q.dcm_z(); }

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	// 翻译：补偿由于yaw权重缩放输出的效果
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

void AttitudeControl::setRefModelFrequency(float omega_n)
{
	_omega_n = math::max(omega_n, 0.1f);
	_kq      = _omega_n * _omega_n;
}

void AttitudeControl::setAttitudeSetpoint(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	Quatf qd_normalized = qd;
	qd_normalized.normalize();

	if (_ref_initialized && dt > 0.f) {
		propagateReferenceModel(qd_normalized, yawspeed_setpoint, dt);

	} else {
		// First call (or dt out of range): snap reference to the current setpoint.
		// 翻译：首次调用(或 dt 超出范围): 捕捉对当前设定点的引用
		_q_ref = qd_normalized;
		_omega_correction.zero();
		_omega_command.zero();
		_ref_initialized = true;
	}
}

void AttitudeControl::propagateReferenceModel(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	// 2nd-order critically damped ref model with exact (ZOH) discretisation.
	// Repeated eigenvalue at s = -_omega_n; unconditionally stable for any dt.
	// 翻译：具有精确(ZOH)离散化的二阶临界阻尼参考模型
	// 	在 s = -_omega_n 处重复出现特征值；对于任何 dt 无条件稳定

	// Tangent-space inputs: rotate the analytical yaw rate into q_ref's body
	//    frame, and form the small-angle error vector from q_ref to q_d.
	// 翻译：切空间输入：将解析偏航角速度旋转到 q_ref 的本体坐标系中
	// 	并形成从 q_ref 到 q_d 的小角度误差向量
	const Quatf q_ref_inv = qinv(_q_ref);
	const Vector3f yaw_axis_body = qzaxis(q_ref_inv); // world yaw axis expressed in q_ref's body frame
	const Vector3f omega_command = PX4_ISFINITE(yawspeed_setpoint)
				       ? yaw_axis_body * yawspeed_setpoint
				       : Vector3f{};

	Quatf q_err = qmul(q_ref_inv, qd);
	q_err.canonicalize();
	const Vector3f e = 2.f * q_err.imag();

	// Entries of exp(A*dt) for A = [0 -1; _kq -2*_omega_n]. A has the repeated eigenvalue lambda = -_omega_n.
	// The matrix N = A - lambda*I is then nilpotent (a matrix is nilpotent when
	// N^k = 0 for some k, and here N^2 = 0). Writing exp(A*dt) = e^(lambda*dt) * exp(N*dt) and expanding the
	// series for exp(N*dt) = I + N*dt + (N*dt)^2/2! + ... , every term from (N*dt)^2
	// onward vanishes, so the exponential truncates to the exact closed form
	//     exp(A*dt) = e^(-_omega_n*dt) * [ (1 + _omega_n*dt) I + dt*A ]
	//               = emt * [ a  -b ;  gamma  delta ].
	const float w_dt  = _omega_n * dt;
	const float emt   = expf(-w_dt);
	const float a     = (1.f + w_dt) * emt;
	const float b     = dt * emt;
	const float gamma = _kq * dt * emt;
	const float delta = (1.f - w_dt) * emt;

	// Propagate the error-driven correction in tangent space (the 2nd-order state). delta_phi is the integral
	//    of omega over [0, dt]; the correction part collapses to e(0) - e(dt) since e_dot = -correction.
	// 翻译：在切空间(二阶状态)中传播误差驱动的修正。delta_phi 是 ω 在 [0, dt] 上的积分；由于 e_dot = -correction，修正部分退化为 e(0) - e(dt)
	const Vector3f delta_phi = (1.f - a) * e + b * _omega_correction + omega_command * dt;
	_omega_correction = gamma * e + delta * _omega_correction;

	// Yaw-rate command: the heading setpoint just follows the measured yaw, so feeding the error-driven
	// rate forward closes a positive-feedback loop. Keep only the commanded rate (omega_command) on the yaw axis.
	// 翻译：偏航角速率指令：航向设定点仅跟随测量的偏航角，因此将误差驱动的速率向前反馈可以闭合正反馈回路。仅保留偏航轴上的指令速率(omega_command)
	if (PX4_ISFINITE(yawspeed_setpoint) && (fabsf(yawspeed_setpoint) > FLT_EPSILON)) {
		_omega_correction -= _omega_correction.dot(yaw_axis_body) * yaw_axis_body;
	}

	// Commanded (analytical) reference rate, kept separate so update() can exempt it from the feedforward limit.
	// 翻译：指令(解析)参考速率单独保留，以便 update() 函数可以将其排除在前馈限制之外
	_omega_command = omega_command;

	_q_ref     = qmul(_q_ref, Quatf(AxisAnglef(delta_phi)));
	_q_ref.normalize();
}

void AttitudeControl::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	// Apply the world-frame delta to the reference attitude. _omega_correction and _omega_command are
	// in the reference body frame and physically invariant under a world relabeling.
	// 翻译：将世界坐标系下的 delta 应用于参考姿态。_omega_correction 和 _omega_command 位于参考机体坐标系中，并且在世界坐标系重新标记下具有物理不变性
	_q_ref = qmul(q_delta, _q_ref);
	_q_ref.normalize();
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	// The P controller always tracks the reference-model attitude.
	// 翻译：P 控制器始终跟踪参考模型的姿态
	Quatf qd = _q_ref;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// 翻译：计算简化后的期望姿态时忽略车辆的偏航，以优先考虑横滚和俯仰
	const Vector3f e_z = qzaxis(q);
	const Vector3f e_z_d = qzaxis(qd);
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		// 翻译：在飞行器和推力方向完全相反的极端情况下，全姿态控制不会产生偏航输入，而是直接利用横滚和俯仰的组合来实现所需的偏航角。忽略这种情况仍然完全安全稳定
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame
		// 翻译：将当前推力矢量到期望推力矢量的旋转变换到世界坐标系下的期望姿态。
		// 	这是一个右乘运算，因为倾斜误差四元数是由世界坐标系下的两个 Z 矢量得到的。
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	// 翻译：使用由 qd = qd_red * qd_dyaw 给出的完整期望姿态，提取偏航角 delta 分量
	// 	根据定义，偏航角 delta 四元数的形式为 (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qmul(qinv(qd_red), qd);
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	// 翻译：捕捉acosf和asinf函数域的数值问题
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	// 翻译：调整偏航角增量并重新组合所需姿态
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	// 翻译：四元数姿态控制律，qe 是从 q 到 qd 的旋转。
	const Quatf qe = qmul(qinv(q), qd);

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// 翻译：使用 sin(alpha/2) 缩放的旋转轴作为姿态误差(参见四元数定义，按轴角，同时考虑对跖点单位四元数的歧义性)
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	// 翻译：计算角速率设定点
	// 乘上比例增益 _proportional_gain
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Map reference-frame rates into the current body frame.
	// 翻译：将参考系速率映射到当前身体坐标系
	const Quatf q_rel = qmul(qinv(q), _q_ref);

	// The commanded reference rate (e.g. manual/auto yaw rate) is a setpoint, not a model prediction, so it
	// bypasses the reference model: it is fed forward at unity regardless of the feedforward gain and limit.
	// 翻译：指令参考速率(例如手动/自动偏航速率)是一个设定值，而不是模型预测值，因此它绕过了参考模型：无论前馈增益和限制如何，它都会以单位值进行前馈
	rate_setpoint += q_rel.rotateVector(_omega_command);

	// the gain scales and the limit caps the model's error-driven anticipation (zero at gain 0)
	// 翻译：增益缩放，限制模型误差驱动的预测能力(增益为 0 时预测能力为零)
	Vector3f omega_ff = _ff_gain * q_rel.rotateVector(_omega_correction);

	if (_ff_max > 0.f) {
		for (int i = 0; i < 3; i++) {
			omega_ff(i) = math::constrain(omega_ff(i), -_ff_max, _ff_max);
		}
	}

	rate_setpoint += omega_ff;

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
