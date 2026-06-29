/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

/**
* 设置前馈和位置控制执行的最大速度
* @param vel_horizontal 水平速度限制
* @param vel_up 向上速度限制
* @param vel_down 向下速度限制
*/
void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

/**
* 设置控制器可输出的最小和最大集体归一化推力[0,1]
* @param min 最小推力 例如 0.1或0
* @param max 最大推力 例如 0.9或1
*/
void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

/**
* 更新悬停推力而不立即影响输出通过调整积分器。
* 这可以防止传播动态悬停推力信号直接输出到控制器。
*/
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	// 翻译：鉴于推力的方程为 T = a_sp * Th / g - Th
	// 	其中 a_sp = 期望加速度, Th = 悬停推力, g = 重力常数,
	// 	我们希望找到需要添加到积分器中的加速度，以在用新的悬停推力替换当前悬停推力后获得相同的推力。
	// 	T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// 	因此 a_sp' = (a_sp - g) * Th / Th' + g
	// 	然后，我们可以将 a_sp' - a_sp 添加到当前积分器中，以吸收 Th 变为 Th' 的影响
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	if (PX4_ISFINITE(_acc_sp(2))) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
			       + CONSTANTS_ONE_G - _acc_sp(2);
	}

}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

/**
* 通过所需的设定值
* 注意：NAN 值意味着如果没有更高阶设定值，则不会出现前馈/离开不受控制的状态。
* @param setpoint 设定点，包括在 update() 中执行的前馈
*/
void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	// 翻译：必须有有效的输出加速度和推力设定点，否则会出现错误
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

/**
 * @brief 位置控制
 */
void PositionControl::_positionControl()
{
	// P-position controller
	// 翻译：位置控制器
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	// 翻译：位置和前馈速度设定点或位置状态为NAN时，它们不会产生影响
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	// 翻译：确保在进一步约束时没有NAN元素
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	// 翻译：水平速度约束优先考虑沿所需位置设定点的速度分量，而不是前馈项
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	// 翻译：约束垂直速度方向
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

/**
 * @brief 速度控制
 */
void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	// 翻译：约束垂直速度积分
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	// 翻译：PID速度控制
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	// 翻译：没有来自设置点或相应状态的控制输入，如果它们是NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// 加速度控制
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	// 翻译：垂直方向积分器抗饱和
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	// 翻译：优先考虑垂直控制，同时保持水平边距
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	// 翻译：确定在保持水平余量的情况下还剩下多少垂直推力
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	// 翻译：饱和最大垂直推力
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	// 翻译：确定水平推力，优先垂直控制
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	// 翻译：饱和最大垂直推力
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// 翻译：对水平方向使用跟踪抗饱和：在饱和期间，使用积分器来消除输出饱和。参见 L. Rundqwist 于 1990 年发表的《PID 控制器的抗复位饱和》
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	// 翻译：由于饱和和实际垂直推力（独立计算），产生的加速度可能大于或小于期望加速度。仅当信号饱和时才需要运行 ARW 回路
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	// 翻译：确保积分不为NAN。
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	// 翻译：更新速度控制的积分部分。
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

/**
 * @brief 加速度控制
 */
void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	// 翻译：假设垂直方向重力产生的标准加速度用于生成姿态
	// 初始化竖直方向的“比力”（specific force），从机体加速度计视角
	// 物理意义: 如果不控制，机体自由落体时 z 轴感受到 -g (负号因为机体 z 向下，重力表现为向上惯性力)
	// 范围: 固定 -9.81 m/s² (CONSTANTS_ONE_G ≈ 9.81)
	float z_specific_force = -CONSTANTS_ONE_G;

	// 不忽略垂直加速度设定值以消除其对倾斜设定值的影响
	// 如果不解耦水平/竖直加速度（默认模式），叠加竖直期望加速度
	// 物理意义: 加入控制器想要的竖直加速度 _acc_sp(2) (NED: 向上为负，向下为正)
	// 示例: 悬停 _acc_sp(2)=0 → z_specific_force=-9.81
	//       向上加速 _acc_sp(2)=-5 → z_specific_force=-9.81-5=-14.81
	// 范围: 一般 -20 ~ -5 (取决于 _acc_sp(2) ≈ -10 ~ +5 m/s²)
	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		// 翻译：包括垂直加速度设定点以更好地跟踪水平加速度。
		z_specific_force += _acc_sp(2);
	}

	// 期望的机体坐标系
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	// 限制倾斜角，防止侧翻
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	// 翻译：假设悬停推力产生标准重力
	// collective_thrust_normalized = ( _acc_sp(2) / g ) × hover_thrust  -  hover_thrust
	//                              = hover_thrust × ( _acc_sp(2)/g  -  1 )
	// 这里是真正的推力计算， body_z只是期望的机体坐标系用作推力映射
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	// 翻译：将推力投影到计划的机身姿态。
	// 计算 NED z 与 body_z 的夹角余弦 (投影因子)
	// 检测机体 Z 轴（body_z）与 NED 向下方向（0,0,1）之间的对齐程度
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	// 计算最终集体推力 (collective_thrust)，限幅避免过大/过小
	// 物理意义: 先投影 thrust_ned_z / cos_ned_body (补偿倾斜)，然后取 min(..., -_lim_thr_min)
	//           -_lim_thr_min ≈ -0.9 ~ -1.0 (最大向上推力)
	//           确保 collective_thrust ≤ -_lim_thr_min (更负=更大推力)，但不低于某个最小 (避免坠落)
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	// 翻译：每个轴x、y、z都需要有某些设定点
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	// 翻译：x和y输入设定点总是成对出现
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	// 翻译：对于每个受控状态，估计值必须有效
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

/**
 * @brief 获取本地位置设定点
 */
void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

/**
 * @brief 获取姿态设定点
 */
void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
