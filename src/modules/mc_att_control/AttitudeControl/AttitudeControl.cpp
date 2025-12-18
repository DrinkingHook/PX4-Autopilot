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

/**
 * @brief 姿态控制更新
 * @param q 当前的姿态四元数
 * @param qd 期望的姿态四元数
 * @return 控制输出
 */
matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// 翻译：计算减少的期望姿态，忽略车辆的yaw，优先考虑roll和pitch
	// 四元数对应一个完整的旋转，可提取三个正交轴（x,y,z）在世界系中的方向。
	// 从四元数中取出任意一个轴的数据都不包含完整的机体的xyz轴上的姿态数据。
	// 只取一个轴（如e_z），只能确定roll/pitch（倾斜），无法唯一确定yaw（偏航）。一个轴只提供两个自由度（方向向量），不足以表达完整三自由度姿态。
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	// 这里生成的四元数是从完整的四元数中提取的Z轴数据重新生成的，所以只有两个自由度，丢弃了yaw轴数据。
	// 它也是一个合法的单位四元数（规范化的），数学上完整表示一个旋转。只是这个旋转绕Z轴的自由度未定义（yaw任意），所以姿态控制上不完整（缺yaw。
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		// 翻译：在车辆和推力完全相反的方向的无穷小角落案例中，
		// 全姿态控制仍然生成没有yaw输入，并直接采取roll和pitch的组合，
		// 导致正确的期望yaw。忽略这个案例仍然完全安全和稳定。
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		// 翻译：从当前到期望推力矢量的旋转转换到世界框架的减少期望姿态。
		// 这是一个右乘法，因为倾斜误差四元数是从世界框架中的两个Z矢量获得的。
		// 先应用当前实际姿态 q（把世界坐标系转到当前机体坐标系）。
		// 再在当前机体坐标系上应用那个“最小扶正旋转” qd_red。
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// 翻译：给定完全期望姿态：qd = qd_red * qd_dyaw，提取偏航角分量。
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	// 翻译：通过定义，delta yaw四元数具有形式(cos(angle/2)，0，0，sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	// 翻译：捕捉acosf和asinf函数域的数值问题
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	// 翻译：调整偏航角增量并重新组合所需姿态
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	// 翻译：四元数姿态控制律，qe 是从 q 到 qd 的旋转
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// 翻译：使用 sin(alpha/2) 缩放的旋转轴作为姿态误差（参见四元数定义，按轴角，同时考虑对跖点单位四元数的歧义性）。
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	// 翻译：计算角速率设定点
	// 乘上比例增益 _proportional_gain
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	// 翻译：前馈偏航设定值速率。
	// yawspeed_setpoint 是前馈的绕世界坐标系 z 轴的指令旋转，
	// 但我们需要将其应用到机体坐标系中（因为 _rates_sp 是在机体坐标系中表示的）。
	// 因此，我们通过取 R.transposed（== q.inversed）的最后一列来推断世界坐标系 z 轴（在机体坐标系中表示）。
	// 并将其乘以偏航设定值速率 (yawspeed_setpoint)。
	// 这将产生一个向量，表示在机体坐标系中表示的绕世界坐标系 z 轴的指令旋转，
	// 以便将其添加到速率设定值中。
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
