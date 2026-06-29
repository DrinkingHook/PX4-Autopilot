/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file geo.cpp
 *
 * Geo / math functions to perform geodesic calculations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "geo.h"

#include <float.h>

using matrix::wrap_pi;
using matrix::wrap_2pi;

/*
 * Azimuthal Equidistant Projection
 * formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html
 */

/**
 * @brief Initializes the reference point for map projection. 初始化地图投影的参考点。
 *
 * Sets the reference latitude and longitude (in degrees) for the map projection,
 * converting them to radians and precomputing sine and cosine values for efficiency.
 * The timestamp is stored to track when the reference point was set.
 * This function must be called before performing any coordinate transformations
 * using the MapProjection class.
 * 设置地图投影的参考纬度和经度（以度为单位），将其转换为弧度，并预先计算正弦和余弦值以提高效率。
 * 存储时间戳以跟踪参考点的设置时间。在使用 MapProjection 类执行任何坐标转换之前，必须调用此函数。
 * @param lat_0 Reference latitude in degrees.
 * @param lon_0 Reference longitude in degrees.
 * @param timestamp Timestamp of the reference point initialization (in microseconds).
 */
void MapProjection::initReference(double lat_0, double lon_0, uint64_t timestamp)
{
	// 存储时间戳
	_ref_timestamp = timestamp;
	// 将纬度从度转换为弧度
	_ref_lat = math::radians(lat_0);
	// 将经度从度转换为弧度
	_ref_lon = math::radians(lon_0);
	// 计算参考纬度的正弦值
	_ref_sin_lat = sin(_ref_lat);
	// 计算参考纬度的余弦值
	_ref_cos_lat = cos(_ref_lat);
	// 标记参考点初始化完成
	_ref_init_done = true;
}

void MapProjection::project(double lat, double lon, float &x, float &y) const
{
	const double lat_rad = math::radians(lat);
	const double lon_rad = math::radians(lon);

	const double sin_lat = sin(lat_rad);
	const double cos_lat = cos(lat_rad);

	const double cos_d_lon = cos(lon_rad - _ref_lon);

	const double arg = math::constrain(_ref_sin_lat * sin_lat + _ref_cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	const double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	x = static_cast<float>(k * (_ref_cos_lat * sin_lat - _ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	y = static_cast<float>(k * cos_lat * sin(lon_rad - _ref_lon) * CONSTANTS_RADIUS_OF_EARTH);
}

/**
 * @brief 将平面坐标转换为经纬度。
 * @param x 平面坐标x。
 * @param y 平面坐标y。
 * @param lat 经度。
 * @param lon 纬度。
 */
void MapProjection::reproject(float x, float y, double &lat, double &lon) const
{
	const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
	const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
	const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

	if (fabs(c) > 0) {
		const double sin_c = sin(c);
		const double cos_c = cos(c);

		const double lat_rad = asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
		const double lon_rad = (_ref_lon + atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c - x_rad * _ref_sin_lat * sin_c));

		lat = math::degrees(lat_rad);
		lon = math::degrees(lon_rad);

	} else {
		lat = math::degrees(_ref_lat);
		lon = math::degrees(_ref_lon);
	}
}

/**
 * @brief 计算当前位置到下一个航点的距离。
 * @param lat_now 当前纬度。
 * @param lon_now 当前经度。
 * @param lat_next 下一个航点的纬度。
 * @param lon_next 下一个航点的经度。
 * @return 当前位置到下一个航点的距离。
 */
float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);

	const double d_lat = lat_next_rad - lat_now_rad;
	const double d_lon = math::radians(lon_next) - math::radians(lon_now);

	const double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(lat_now_rad) * cos(
				 lat_next_rad);

	const double c = atan2(sqrt(a), sqrt(1.0 - a));

	return static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * 2.0 * c);
}

/**
 * @brief 根据起始点、终点和距离生成航点。
 * @param lat_A 起始点纬度。
 * @param lon_A 起始点经度。
 * @param lat_B 终点纬度。
 * @param lon_B 终点经度。
 * @param dist 距离。
 * @param lat_target 目标点纬度。
 * @param lon_target 目标点经度。
 */
void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
					double *lat_target, double *lon_target)
{
	if (fabsf(dist) < FLT_EPSILON) {
		*lat_target = lat_A;
		*lon_target = lon_A;

	} else {
		float heading = get_bearing_to_next_waypoint(lat_A, lon_A, lat_B, lon_B);
		waypoint_from_heading_and_distance(lat_A, lon_A, heading, dist, lat_target, lon_target);
	}
}

/**
 * @brief 根据起始点、航向和距离生成航点。
 * @param lat_start 起始点纬度。
 * @param lon_start 起始点经度。
 * @param bearing 航向。
 * @param dist 距离。
 * @param lat_target 目标点纬度。
 * @param lon_target 目标点经度。
 */
void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
					double *lat_target, double *lon_target)
{
	bearing = wrap_2pi(bearing);

	double radius_ratio = static_cast<double>(dist) / CONSTANTS_RADIUS_OF_EARTH;

	double lat_start_rad = math::radians(lat_start);
	double lon_start_rad = math::radians(lon_start);

	*lat_target = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos((
				   double)bearing));
	*lon_target = lon_start_rad + atan2(sin((double)bearing) * sin(radius_ratio) * cos(lat_start_rad),
					    cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_target));

	*lat_target = math::degrees(*lat_target);
	*lon_target = math::degrees(*lon_target);
}

/**
 * @brief Get the bearing to next waypoint object 获取下一个航点的方位
 *
 * @param lat_now 现在的经度
 * @param lon_now 现在的纬度
 * @param lat_next 下一个的纬度
 * @param lon_next 下一个经度
 * @return float
 */
float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);

	const double cos_lat_next = cos(lat_next_rad);
	const double d_lon = math::radians(lon_next - lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */

	const float y = static_cast<float>(sin(d_lon) * cos_lat_next);
	const float x = static_cast<float>(cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos_lat_next * cos(d_lon));

	return wrap_pi(atan2f(y, x));
}

/**
 * @brief 计算从当前位置到下一个航点的向量
 * @param lat_now 当前纬度
 * @param lon_now 当前经度
 * @param lat_next 下一个纬度
 * @param lon_next 下一个经度
 * @param v_n 向北分量
 * @param v_e 向东分量
 */
void
get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e)
{
	const double lat_now_rad = math::radians(lat_now);
	const double lat_next_rad = math::radians(lat_next);
	const double d_lon = math::radians(lon_next) - math::radians(lon_now);

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * (cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(
					  lat_next_rad) * cos(d_lon)));
	*v_e = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * sin(d_lon) * cos(lat_next_rad));
}

/**
 * @brief 快速获取到下一个航点的向量
 * @param lat_now 当前纬度
 * @param lon_now 当前经度
 * @param lat_next 下一个纬度
 * @param lon_next 下一个经度
 * @param v_n 指向北向量的指针
 * @param v_e 指向东向量的指针
 */
void
get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
				 float *v_e)
{
	double lat_now_rad = math::radians(lat_now);
	double lon_now_rad = math::radians(lon_now);
	double lat_next_rad = math::radians(lat_next);
	double lon_next_rad = math::radians(lon_next);

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	*v_n = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * d_lat);
	*v_e = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * d_lon * cos(lat_now_rad));
}

/**
 * @brief 将向量添加到全局位置
 * @param lat_now 当前纬度。
 * @param lon_now 当前经度。
 * @param v_n 向北距离。
 * @param v_e 向东距离。
 * @param lat_res 结果纬度。
 * @param lon_res 结果经度。
 */
void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res,
				   double *lon_res)
{
	double lat_now_rad = math::radians(lat_now);
	double lon_now_rad = math::radians(lon_now);

	*lat_res = math::degrees(lat_now_rad + (double)v_n / CONSTANTS_RADIUS_OF_EARTH);
	*lon_res = math::degrees(lon_now_rad + (double)v_e / (CONSTANTS_RADIUS_OF_EARTH * cos(lat_now_rad)));
}

// Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>

/**
 * @brief 计算当前位置到线段的距离
 * @param crosstrack_error 结果结构体
 * @param lat_now 当前纬度。
 * @param lon_now 当前经度。
 * @param lat_start 线段起点纬度。
 * @param lon_start 线段起点经度。
 * @param lat_end 线段终点纬度。
 * @param lon_end 线段终点经度。
 * @return 返回值
 */
int get_distance_to_line(struct crosstrack_error_s &crosstrack_error, double lat_now, double lon_now,
			 double lat_start, double lon_start, double lat_end, double lon_end)
{
	// This function returns the distance to the nearest point on the track line.  Distance is positive if current
	// position is right of the track and negative if left of the track as seen from a point on the track line
	// headed towards the end point.
	// 翻译：此函数返回到轨道线上最近点的距离。如果当前位置位于轨道右侧，则距离为正；
	// 如果位于轨道左侧，则距离为负（从轨道线上朝向终点的方向观察）。

	int return_value = -1;	// Set error flag, cleared when valid result calculated.
	crosstrack_error.past_end = false;
	crosstrack_error.distance = 0.0f;
	crosstrack_error.bearing = 0.0f;

	float dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

	// Return error if arguments are bad
	if (dist_to_end < 0.1f) {
		return -1;
	}

	float bearing_end = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
	float bearing_track = get_bearing_to_next_waypoint(lat_start, lon_start, lat_end, lon_end);
	float bearing_diff = wrap_pi(bearing_track - bearing_end);

	// Return past_end = true if past end point of line
	if (bearing_diff > M_PI_2_F || bearing_diff < -M_PI_2_F) {
		crosstrack_error.past_end = true;
		return_value = 0;
		return return_value;
	}

	crosstrack_error.distance = (dist_to_end) * sinf(bearing_diff);

	if (sinf(bearing_diff) >= 0) {
		crosstrack_error.bearing = wrap_pi(bearing_track - M_PI_2_F);

	} else {
		crosstrack_error.bearing = wrap_pi(bearing_track + M_PI_2_F);
	}

	return_value = 0;

	return return_value;
}

/**
 * @brief 计算从当前位置到弧线的最短距离。
 *
 * @param crosstrack_error 结构体，用于存储计算结果。
 * @param lat_now 当前位置的纬度。
 * @param lon_now 当前位置的经度。
 * @param lat_center 弧线中心的纬度。
 * @param lon_center 弧线中心的经度。
 * @param radius 弧线的半径。
 * @param arc_start_bearing 弧线起始角度。
 * @param arc_sweep 弧线扫过的角度。
 *
 * @return 返回值。
 */
int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
			double lat_center, double lon_center,
			float radius, float arc_start_bearing, float arc_sweep)
{
	// This function returns the distance to the nearest point on the track arc.  Distance is positive if current
	// position is right of the arc and negative if left of the arc as seen from the closest point on the arc and
	// headed towards the end point.
	// 翻译：此函数返回到轨迹弧上最近点的距离。如果当前位置位于弧线右侧，则距离为正；
	// 如果位于弧线左侧，则距离为负（从弧线上的最近点看，并朝向终点方向）。

	// Determine if the current position is inside or outside the sector between the line from the center
	// to the arc start and the line from the center to the arc end
	const float bearing_now = get_bearing_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

	int return_value = -1;		// Set error flag, cleared when valid result calculated.
	crosstrack_error->past_end = false;
	crosstrack_error->distance = 0.0f;
	crosstrack_error->bearing = 0.0f;

	// Return error if arguments are bad
	if (radius < 0.1f) {
		return return_value;
	}

	// arc_start_bearing and arc_sweep are referenced to the center, so test membership with the
	// center->vehicle bearing. Measuring it in the direction of the sweep handles either sign and
	// any zero crossing without special cases.
	// NOLINTNEXTLINE(readability-suspicious-call-argument) intentional reverse bearing: center -> vehicle
	const float bearing_center_to_now = get_bearing_to_next_waypoint(lat_center, lon_center, lat_now, lon_now);
	const float bearing_from_start = wrap_2pi((bearing_center_to_now - arc_start_bearing) * (arc_sweep < 0.0f ? -1.0f :
					 1.0f));
	const bool in_sector = bearing_from_start <= fabsf(arc_sweep);

	// If in the sector then calculate distance and bearing to closest point
	if (in_sector) {
		crosstrack_error->past_end = false;
		float dist_to_center = get_distance_to_next_waypoint(lat_now, lon_now, lat_center, lon_center);

		if (dist_to_center <= radius) {
			crosstrack_error->distance = radius - dist_to_center;
			crosstrack_error->bearing = bearing_now + M_PI_F;

		} else {
			crosstrack_error->distance = dist_to_center - radius;
			crosstrack_error->bearing = bearing_now;
		}

		// If out of the sector then calculate dist and bearing to start or end point

	} else {

		// Use the approximation  that 111,111 meters in the y direction is 1 degree (of latitude)
		// and 111,111 * cos(latitude) meters in the x direction is 1 degree (of longitude) to
		// calculate the position of the start and end points.  We should not be doing this often
		// as this function generally will not be called repeatedly when we are out of the sector.

		double start_disp_x = (double)radius * sin((double)arc_start_bearing);
		double start_disp_y = (double)radius * cos((double)arc_start_bearing);
		double end_disp_x = (double)radius * sin((double)wrap_pi(arc_start_bearing + arc_sweep));
		double end_disp_y = (double)radius * cos((double)wrap_pi(arc_start_bearing + arc_sweep));
		const double cos_lat_center = cos(math::radians(lat_center));
		double lon_start = lon_center + start_disp_x / (111111.0 * cos_lat_center);
		double lat_start = lat_center + start_disp_y / 111111.0;
		double lon_end = lon_center + end_disp_x / (111111.0 * cos_lat_center);
		double lat_end = lat_center + end_disp_y / 111111.0;
		float dist_to_start = get_distance_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);
		float dist_to_end = get_distance_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);

		if (dist_to_start < dist_to_end) {
			crosstrack_error->distance = dist_to_start;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_start, lon_start);

		} else {
			crosstrack_error->past_end = true;
			crosstrack_error->distance = dist_to_end;
			crosstrack_error->bearing = get_bearing_to_next_waypoint(lat_now, lon_now, lat_end, lon_end);
		}
	}

	crosstrack_error->bearing = wrap_pi(crosstrack_error->bearing);
	return_value = 0;

	return return_value;
}

/**
 * @brief 计算从当前位置到下一个位置的水平距离和垂直距离(3D距离)
 * 	   WGS84坐标系(地球椭球体模型gps数据)
 * @param lat_now 当前纬度
 * @param lon_now 当前经度
 * @param alt_now 当前高度
 * @param lat_next 下一个位置的纬度
 * @param lon_next 下一个位置的经度
 * @param alt_next 下一个位置的高度
 * @param dist_xy 水平距离
 * @param dist_z 垂直距离
 */
float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z)
{
	double current_x_rad = math::radians(lat_next);
	double current_y_rad = math::radians(lon_next);
	double x_rad = math::radians(lat_now);
	double y_rad = math::radians(lon_now);

	double d_lat = x_rad - current_x_rad;
	double d_lon = y_rad - current_y_rad;

	double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0) * cos(current_x_rad) * cos(x_rad);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	const float dxy = static_cast<float>(CONSTANTS_RADIUS_OF_EARTH * c);
	const float dz = static_cast<float>(alt_now - alt_next);

	*dist_xy = fabsf(dxy);
	*dist_z = fabsf(dz);

	return sqrtf(dxy * dxy + dz * dz);
}

/**
 * @brief mavlink航点到本地点距离
 * 	  一般传入参数是NED坐标系
 * @param x_now 当前X坐标
 * @param y_now 当前Y坐标
 * @param z_now 当前Z坐标
 * @param x_next 下一个位置的X坐标
 * @param y_next 下一个位置的Y坐标
 * @param z_next 下一个位置的Z坐标
 * @param dist_xy 水平距离
 * @param dist_z 垂直距离
 */
float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
		float x_next, float y_next, float z_next,
		float *dist_xy, float *dist_z)
{
	float dx = x_now - x_next;
	float dy = y_now - y_next;
	float dz = z_now - z_next;

	*dist_xy = sqrtf(dx * dx + dy * dy);
	*dist_z = fabsf(dz);

	return sqrtf(dx * dx + dy * dy + dz * dz);
}
