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
 * @file TrajMath.hpp
 *
 * collection of functions used for trajectory generation
 */

#pragma once

namespace math
{

namespace trajectory
{

/* Compute the maximum possible speed on the track given the desired speed,
 * remaining distance, the maximum acceleration and the maximum jerk.
 * We assume a constant acceleration profile with a delay of 2*accel/jerk
 * (time to reach the desired acceleration from opposite max acceleration)
 * Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param jerk maximum jerk
 * @param accel maximum acceleration
 * @param braking_distance distance to the desired point
 * @param final_speed the still-remaining speed of the vehicle when it reaches the braking_distance
 *
 * @return maximum speed
 */
 
 /* 翻译：
  * 计算给定期望速度、剩余距离、最大加速度和最大加加速度时，赛道上的最大可能速度.
  * 我们假设加速度曲线恒定，且存在 2*accel/jerk 的延迟（从相反方向的最大加速度达到期望加速度所需的时间）。
  * 待求解的方程：vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
  *
  * @param jerk 最大加加速度
  * @param accel 最大加速度
  * @param braking_distance 到目标点的距离
  * @param final_speed 车辆到达制动距离时的剩余速度
  *
  * @return maximum speed
  */
inline float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
		const float final_speed)
{
	auto sqr = [](float f) {return f * f;};
	float b =  4.0f * sqr(accel) / jerk;
	float c = - 2.0f * accel * braking_distance - sqr(final_speed);
	float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

	// don't slow down more than the end speed, even if the conservative accel ramp time requests it
	return fmaxf(max_speed, final_speed);
}

/* Compute the maximum tangential speed in a circle defined by two line segments of length "d"
 * forming a V shape, opened by an angle "alpha". The circle is tangent to the end of the
 * two segments as shown below:
 *      \\
 *      | \ d
 *      /  \
 *  __='___a\
 *      d
 *  @param alpha angle between the two line segments
 *  @param accel maximum lateral acceleration
 *  @param d length of the two line segments
 *
 *  @return maximum tangential speed
 */
 
 /* 翻译：
  * 计算由两条长度为“d”的线段构成的 V 形圆内的最大切向速度，该 V 形圆的开口角度为“alpha”。
  * 该圆与两条线段的端点相切，如下图所示：
  *      \\
  *      | \ d
  *      /  \
  *  __='___a\
  *      d
  *  @param alpha 两个线段之间的夹角
  *  @param accel 最大横向加速度
  *  @param d 两个线段的长度
  *
  *  @return maximum tangential speed
  */
inline float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
{
	float tan_alpha = tanf(alpha / 2.0f);
	float max_speed_in_turn = sqrtf(accel * d * tan_alpha);

	return max_speed_in_turn;
}

/* Compute the braking distance given a maximum acceleration, maximum jerk and a maximum delay acceleration.
 * We assume a constant acceleration profile with a delay of accel_delay_max/jerk
 * (time to reach the desired acceleration from opposite max acceleration)
 * Equation to solve: vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param velocity initial velocity
 * @param jerk maximum jerk
 * @param accel maximum target acceleration during the braking maneuver
 * @param accel_delay_max the acceleration defining the delay described above
 *
 * @return braking distance
 */
 
 /* 翻译：
  * 已知最大加速度、最大加加速度和最大延迟加速度，计算制动距离。
  * 我们假设加速度曲线恒定，存在一个延迟，延迟量为 accel_delay_max/jerk
  *（从相反方向的最大加速度达到目标加速度所需的时间）
  * 待求解的方程：vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
  *
  * @param velocity 速度初始值
  * @param jerk 最大加加速度
  * @param accel 制动过程中最大目标加速度
  * @param accel_delay_max 上述延迟所定义的加速度
  *
  * @return braking distance
  */
inline float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
{
	return velocity * (velocity / (2.0f * accel) + accel_delay_max / jerk);
}

/* Compute the maximum distance between a point and a circle given a direction vector pointing from the point
 * towards the circle. The point can be inside or outside the circle.
 *                  _
 *               ,=' '=,               __
 *    P-->------/-------A   Distance = PA
 *       Dir   |    x    |
 *              \       /
 *               "=,_,="
 * Equation to solve: ||(point - circle_pos) + direction_unit * distance_to_circle|| = radius
 *
 * @param pos position of the point
 * @param circle_pos position of the center of the circle
 * @param radius radius of the circle
 * @param direction vector pointing from the point towards the circle
 *
 * @return longest distance between the point to the circle in the direction indicated by the vector or NAN if the
 * vector does not point towards the circle
 */
 
 /* 翻译：
  * 计算给定指向圆的方向向量时，点与圆之间的最大距离。点可以位于圆内或圆外。
  *                  _
  *               ,=' '=,               __
  *    P-->------/-------A   Distance = PA
  *       Dir   |    x    |
  *              \       /
  *               "=,_,="
  * 待求解方程: ||(point - circle_pos) + direction_unit * distance_to_circle|| = radius
  *
  * @param pos 点的位置
  * @param circle_pos 圆心的位置
  * @param radius 圆的半径
  * @param direction 从点指向圆的方向向量
  *
  * @return 沿向量方向，点到圆的最大距离；如果向量不指向圆，则结果为 NaN。
  */
inline float getMaxDistanceToCircle(const matrix::Vector2f &pos, const matrix::Vector2f &circle_pos, float radius,
				    const matrix::Vector2f &direction)
{
	// 计算从圆心指向当前位置的向量
	matrix::Vector2f center_to_pos = pos - circle_pos;
	// 计算二次方程的 b 系数
	//  b = 2(v·d)
	// direction.unit_or_zero() 确保是单位向量（如果为零向量则返回零）
	const float b = 2.f * center_to_pos.dot(direction.unit_or_zero());
	// 计算二次方程的 c 系数
	// c = |v|² - r²
	// norm_squared() = v·v (向量长度的平方)
	const float c = center_to_pos.norm_squared() - radius * radius;
	// 计算判别式
	// Δ = b² - 4c
	const float delta = b * b - 4.f * c;

	float distance_to_circle;

	// 检查是否有有效解
	// delta >= 0: 射线与圆有交点
    	// direction.longerThan(0.f): 方向向量不为零
	if (delta >= 0.f && direction.longerThan(0.f)) {
		// 计算距离（取较大的根)
		// t = (-b + √Δ) / 2
    		// fmaxf(..., 0.f) 确保非负
		distance_to_circle = fmaxf((-b + sqrtf(delta)) / 2.f, 0.f);

	} else {
		// Never intersecting the circle
		// 翻译：没有交点：射线不会与圆相交
		distance_to_circle = NAN;
	}

	return distance_to_circle;
}

} /* namespace traj */
} /* namespace math */
