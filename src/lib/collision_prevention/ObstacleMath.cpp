/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "ObstacleMath.hpp"
#include <mathlib/math/Limits.hpp>

using namespace matrix;

namespace ObstacleMath
{

/**
 * @brief 将距离投影到水平平面上。
 * @param distance 距离
 * @param yaw 面向角度
 * @param q_world_vehicle 车辆在世界坐标系下的四元数
 */
void project_distance_on_horizontal_plane(float &distance, const float yaw, const matrix::Quatf &q_world_vehicle)
{
	const Quatf q_vehicle_sensor(Quatf(cosf(yaw / 2.f), 0.f, 0.f, sinf(yaw / 2.f)));
	const Quatf q_world_sensor = q_world_vehicle * q_vehicle_sensor;
	const Vector3f forward(1.f, 0.f, 0.f);
	const Vector3f sensor_direction_in_world = q_world_sensor.rotateVector(forward);

	float horizontal_projection_scale = sensor_direction_in_world.xy().norm();
	horizontal_projection_scale = math::constrain(horizontal_projection_scale, FLT_EPSILON, 1.0f);
	distance *= horizontal_projection_scale;
}

/**
 * @brief 获取角度所在的bin索引。
 * @param bin_width 每个bin的宽度。
 * @param angle 角度。
 * @return 角度所在的bin索引。
 */
int get_bin_at_angle(float bin_width, float angle)
{
	int bin_at_angle = (int)round(matrix::wrap(angle, 0.f, 360.f) / bin_width);
	return wrap_bin(bin_at_angle, 360 / bin_width);
}

/**
 * @brief 获取角度的下界。
 * @param bin_width 每个bin的宽度。
 * @param angle 角度。
 * @param angle_offset 偏航角偏移量。
 * @return 角度的下界。
 */
float get_lower_bound_angle(int bin, float bin_width, float angle_offset)
{
	bin = wrap_bin(bin, 360 / bin_width);
	return wrap_360(bin * bin_width + angle_offset - bin_width / 2.f);
}

/**
 * @brief 获取偏移量的bin索引。
 *
 * @param bin 原始bin索引。
 * @param bin_width 每个bin的宽度。
 * @param angle_offset 偏航角偏移量。
 * @return 偏移量的bin索引。
 */
int get_offset_bin_index(int bin, float bin_width, float angle_offset)
{
	int offset = get_bin_at_angle(bin_width, angle_offset);
	return wrap_bin(bin - offset, 360 / bin_width);
}

/**
 * @brief 将传感器方向转换为偏航角偏移量。
 *
 * @param orientation 传感器方向。
 * @param q 传感器方向的四元数。
 * @return 偏航角偏移量。
 */
float sensor_orientation_to_yaw_offset(const SensorOrientation orientation, const float q[4])
{
	float offset = 0.0f;

	switch (orientation) {
	case SensorOrientation::ROTATION_YAW_0:
		offset = 0.0f;
		break;

	case SensorOrientation::ROTATION_YAW_45:
		offset = M_PI_F / 4.0f;
		break;

	case SensorOrientation::ROTATION_YAW_90:
		offset = M_PI_F / 2.0f;
		break;

	case SensorOrientation::ROTATION_YAW_135:
		offset = 3.0f * M_PI_F / 4.0f;
		break;

	case SensorOrientation::ROTATION_YAW_180:
		offset = M_PI_F;
		break;

	case SensorOrientation::ROTATION_YAW_225:
		offset = -3.0f * M_PI_F / 4.0f;
		break;

	case SensorOrientation::ROTATION_YAW_270:
		offset = -M_PI_F / 2.0f;
		break;

	case SensorOrientation::ROTATION_YAW_315:
		offset = -M_PI_F / 4.0f;
		break;

	case SensorOrientation::ROTATION_CUSTOM:
		if (q != nullptr) {
			offset = Eulerf(Quatf(q)).psi();
		}

		break;
	}

	return offset;
}

/**
 * @brief 将bin索引转换为角度。
 * @param bin bin索引。
 * @param bin_width 每个bin的宽度。
 * @return 角度。
 */
int wrap_bin(int bin, int bin_count)
{
	return (bin + bin_count) % bin_count;
}

/**
 * @brief 将角度转换为bin索引。
 * @param angle 角度。
 * @param bin_width 每个bin的宽度。
 * @return bin索引。
 */
float wrap_360(const float angle)
{
	return matrix::wrap(angle, 0.0f, 360.0f);
}

} // ObstacleMath
