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
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <cstdlib>

/**
 * @brief 判断是否需要重置高度。
 */
bool Ekf::isHeightResetRequired() const
{
	// check if height is continuously failing because of accel errors
	// 翻译：检查高度是否连续失败，因为加速度错误
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	// 翻译注释：检查高度是否连续失败，因为惯性死区
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

/**
 * @brief 计算地球自转速度在NED坐标系下的向量。
 * @param lat_rad 纬度的弧度表示。
 */
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

/**
 * @brief 获取EKF的全局原点信息，包括时间、纬度、经度和海拔高度。
 *
 * @param origin_time 输出参数，表示原点的时间戳。
 * @param latitude 输出参数，表示原点的纬度。
 * @param longitude 输出参数，表示原点的经度。
 * @param origin_alt 输出参数，表示原点的海拔高度。
 */
void Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _local_origin_lat_lon.getProjectionReferenceTimestamp();
	latitude = _local_origin_lat_lon.getProjectionReferenceLat();
	longitude = _local_origin_lat_lon.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
}

/**
 * @brief 检查纬度和经度的有效性。
 *
 * @param latitude 输入参数，表示要检查的纬度。
 * @param longitude 输入参数，表示要检查的经度。
 *
 * @return true 如果纬度和经度有效，false 否则。
 */
bool Ekf::checkLatLonValidity(const double latitude, const double longitude)
{
	const bool lat_valid = (PX4_ISFINITE(latitude) && (abs(latitude) <= 90));
	const bool lon_valid = (PX4_ISFINITE(longitude) && (abs(longitude) <= 180));

	return (lat_valid && lon_valid);
}

/**
 * @brief 检查高度的有效性。
 *
 * @param altitude 输入参数，表示要检查的高度。
 *
 * @return true 如果高度有效，false 否则。
 */
bool Ekf::checkAltitudeValidity(const float altitude)
{
	// sanity check valid altitude anywhere between the Mariana Trench and edge of Space
	// 翻译:合理性检查有效高度，介于马里亚纳海沟和太空边缘之间的任何高度
	return (PX4_ISFINITE(altitude) && ((altitude > -12'000.f) && (altitude < 100'000.f)));
}

/**
 * @brief 设置EKF的全局原点。
 *
 * @param latitude 输入参数，表示要设置的纬度。
 * @param longitude 输入参数，表示要设置的经度。
 * @param altitude 输入参数，表示要设置的高度。
 * @param hpos_var 输入参数，表示水平位置方差。
 * @param vpos_var 输入参数，表示垂直位置方差。
 *
 * @return true 如果设置成功，false 否则。
 */
bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude, const float hpos_var,
			     const float vpos_var)
{
	if (!setLatLonOrigin(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	setAltOrigin(altitude, vpos_var);

	return true;
}

/**
 * @brief 设置EKF的纬度和经度原点。
 *
 * @param latitude 输入参数，表示要设置的纬度。
 * @param longitude 输入参数，表示要设置的经度。
 * @param hpos_var 输入参数，表示水平位置方差。
 *
 * @return true 如果设置成功，false 否则。
 */
bool Ekf::setLatLonOrigin(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid()) {
		// Already navigating in a local frame, use the origin to initialize global position
		// 翻译：如果已经处于局部坐标系中，使用原点来初始化全局位置
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		double new_latitude;
		double new_longitude;
		_local_origin_lat_lon.reproject(pos_prev(0), pos_prev(1), new_latitude, new_longitude);
		resetHorizontalPositionTo(new_latitude, new_longitude, hpos_var);

	} else {
		// Simply move the origin and compute the change in local position
		// 翻译：如果尚未处于局部坐标系中，直接移动原点并计算局部位置的变化
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		const Vector2f pos_new = getLocalHorizontalPosition();
		const Vector2f delta_pos = pos_new - pos_prev;
		updateHorizontalPositionResetStatus(delta_pos);
	}

	return true;
}

/**
 * @brief 设置新的高度原点，更新高度状态
 * @param altitude 新的高度原点
 * @param vpos_var 高度方差
 * @return 是否成功设置新的高度原点
 */
bool Ekf::setAltOrigin(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	ECL_INFO("EKF origin altitude %.1fm -> %.1fm", (double)_local_origin_alt,
		 (double)altitude);

	if (!PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid()) {
		const float local_alt_prev = _gpos.altitude();
		_local_origin_alt = altitude;
		resetAltitudeTo(local_alt_prev + _local_origin_alt);

	} else {
		const float delta_origin_alt = altitude - _local_origin_alt;
		_local_origin_alt = altitude;
		updateVerticalPositionResetStatus(-delta_origin_alt);

#if defined(CONFIG_EKF2_TERRAIN)
		updateTerrainResetStatus(-delta_origin_alt);
#endif // CONFIG_EKF2_TERRAIN
	}

	return true;
}

/**
 * @brief 重置全局位置到指定的经纬度和高度。
 *
 * @param latitude  纬度（度）
 * @param longitude 经度（度）
 * @param altitude  高度（米）
 * @param hpos_var  水平位置方差（米^2）
 * @param vpos_var  垂直位置方差（米^2）
 *
 * @return true 如果重置成功，否则返回false。
 */
bool Ekf::resetGlobalPositionTo(const double latitude, const double longitude, const float altitude,
				const float hpos_var, const float vpos_var)
{
	if (!resetLatLonTo(latitude, longitude, hpos_var)) {
		return false;
	}

	// altitude is optional
	initialiseAltitudeTo(altitude, vpos_var);

	return true;
}

/**
 * @brief 重置纬度和经度到指定的值。
 *
 * @param latitude  纬度（度）
 * @param longitude 经度（度）
 * @param hpos_var  水平位置方差（米^2）
 *
 * @return true 如果重置成功，否则返回false。
 */
bool Ekf::resetLatLonTo(const double latitude, const double longitude, const float hpos_var)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	Vector2f pos_prev;

	if (!_local_origin_lat_lon.isInitialized()) {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		pos_prev = zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());

		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);

		// if we are already doing aiding, correct for the change in position since the EKF started navigating
		if (isLocalHorizontalPositionValid()) {
			double est_lat;
			double est_lon;
			_local_origin_lat_lon.reproject(-pos_prev(0), -pos_prev(1), est_lat, est_lon);
			_local_origin_lat_lon.initReference(est_lat, est_lon, _time_delayed_us);
		}

		ECL_INFO("Origin set to lat=%.6f, lon=%.6f",
			 _local_origin_lat_lon.getProjectionReferenceLat(), _local_origin_lat_lon.getProjectionReferenceLon());

	} else {
		pos_prev = _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}

	_gpos.setLatLonDeg(latitude, longitude);
	_output_predictor.resetLatLonTo(latitude, longitude);

	const Vector2f delta_horz_pos = getLocalHorizontalPosition() - pos_prev;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_pos) {
		_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	updateHorizontalPositionResetStatus(delta_horz_pos);

	if (PX4_ISFINITE(hpos_var)) {
		P.uncorrelateCovarianceSetVariance<2>(State::pos.idx, math::max(sq(0.01f), hpos_var));
	}

	// Reset the timout timer
	_time_last_hor_pos_fuse = _time_delayed_us;

	return true;
}

/**
 * @brief 初始化高度到指定值
 *
 * @param altitude 高度值
 * @param vpos_var 高度方差
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool Ekf::initialiseAltitudeTo(const float altitude, const float vpos_var)
{
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	if (!PX4_ISFINITE(_local_origin_alt)) {
		const float local_alt_prev = _gpos.altitude();

		if (isLocalVerticalPositionValid()) {
			_local_origin_alt = altitude - local_alt_prev;

		} else {
			_local_origin_alt = altitude;
		}

		ECL_INFO("Origin alt=%.3f", (double)_local_origin_alt);
	}

	resetAltitudeTo(altitude, vpos_var);

	return true;
}

/**
 * @brief 获取ekf的地理位置精度
 *
 * @param ekf_eph 横向误差
 * @param ekf_epv 纵向误差
 */
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	if (global_origin_valid()) {
		get_ekf_lpos_accuracy(ekf_eph, ekf_epv);

	} else {
		*ekf_eph = INFINITY;
		*ekf_epv = INFINITY;
	}
}

/**
 * @brief 获取ekf的水平位置精度
 *
 * @param ekf_eph 横向误差
 * @param ekf_epv 纵向误差
 */
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P.trace<2>(State::pos.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	// 翻译：如果航位推算时间过长，则应使用创新点作为水平位置误差的保守替代指标。
	// 原因是，完全拒绝测量结果通常是由航向偏差或惯性传感误差引起的。
	// 在这些情况下，使用状态方差来报告精度过于乐观。
	if (_horizontal_deadreckon_time_exceeded) {
#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gnss_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2));
}

/**
 * @brief 获取EKF估计的速度精度
 *
 * @param ekf_eph 横向误差
 * @param ekf_epv 纵向误差
 */
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P.trace<2>(State::vel.idx));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	// 翻译：如果我们航位推算时间太长，请使用这些创新作为水平速度误差的保守替代测量方法
	// 原因是完全拒绝测量通常是由航向未对准或惯性传感误差引起的
	// 在这些情况下使用状态差异进行准确性报告过于乐观
	if (_horizontal_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.ekf2_min_rng, 0.1f);
			vel_err_conservative = math::max(getHagl(), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)

		if (_control_status.flags.gnss_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}

		if (_control_status.flags.gnss_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_vel.innovation).norm());
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_vel.innovation).norm());
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(State::vel.idx + 2, State::vel.idx + 2));
}

void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max_z,
			      float *hagl_max_xy) const
{
	// Do not require limiting by default
	// 翻译：默认不强制限制
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max_z = NAN;
	*hagl_max_xy = NAN;

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// Calculate range finder limits
	// 翻译：计算测距仪极限
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();

	// Allow use of 90% of rangefinder maximum range to allow for angular motion
	// 翻译：允许使用90%的测距仪最大范围来允许角运动
	const float rangefinder_hagl_max = 0.9f * _range_sensor.getValidMaxVal();

	// TODO : calculate visual odometry limits
	// 翻译：计算视觉里程计极限
	const bool relying_on_rangefinder = isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.rng_hgt);

	if (relying_on_rangefinder) {
		*hagl_min = rangefinder_hagl_min;
		*hagl_max_z = rangefinder_hagl_max;
	}

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// Keep within flow AND range sensor limits when exclusively using optical flow
	// 翻译：当仅使用光学流时，保持在流和测距仪限制内
	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	if (relying_on_optical_flow) {
		// Calculate optical flow limits
		// 翻译：计算光学流限制
		float flow_hagl_min = _flow_min_distance;
		float flow_hagl_max = _flow_max_distance;

		// only limit optical flow height is dependent on range finder or terrain estimate invalid (precaution)
		// 翻译：仅在测距仪或地形估计无效时限制光学流高度（预防措施）
		if ((!_control_status.flags.opt_flow_terrain && _control_status.flags.rng_terrain)
		    || !isTerrainEstimateValid()
		   ) {
			flow_hagl_min = math::max(flow_hagl_min, rangefinder_hagl_min);
			flow_hagl_max = math::min(flow_hagl_max, rangefinder_hagl_max);
		}

		const float flow_constrained_height = math::constrain(getHagl(), flow_hagl_min, flow_hagl_max);

		// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
		// 翻译：允许地面相对速度使用可用流传感器范围的50％，以允许角运动
		float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;
		flow_hagl_max = math::max(flow_hagl_max * 0.9f, flow_hagl_max - 1.0f);

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max_xy = flow_hagl_max;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_RANGE_FINDER
}

/**
 * @brief 重置Gyro偏置
 */
void Ekf::resetGyroBias()
{
	// Zero the gyro bias states
	_state.gyro_bias.zero();

	resetGyroBiasCov();
}

/**
 * @brief 重置Accel偏置
 */
void Ekf::resetAccelBias()
{
	// Zero the accel bias states
	_state.accel_bias.zero();

	resetAccelBiasCov();
}

/**
 * @brief 获取航向的创新测试比率（Innovation Test Ratio）
 */
float Ekf::getHeadingInnovationTestRatio() const
{
	// return the largest heading innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		for (auto &test_ratio_filtered : _aid_src_mag.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_yaw.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

/**
 * @brief 获取水平速度的创新测试比率（Innovation Test Ratio）
 */
float Ekf::getHorizontalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_vel) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		for (int i = 0; i < 2; i++) { // only xy
			test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[i]));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		for (auto &test_ratio_filtered : _aid_src_optical_flow.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

/**
 * @brief 获取垂直速度的创新测试比率（Innovation Test Ratio）
 */
float Ekf::getVerticalVelocityInnovationTestRatio() const
{
	// return the largest velocity innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_vel) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_vel) {
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[2]));
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

/**
 * @brief 获取水平位置的创新测试比率（Innovation Test Ratio）
 */
float Ekf::getHorizontalPositionInnovationTestRatio() const
{
	// return the largest position innovation test ratio
	float test_ratio = -1.f;

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gnss_pos) {
		for (auto &test_ratio_filtered : _aid_src_gnss_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_pos) {
		for (auto &test_ratio_filtered : _aid_src_ev_pos.test_ratio_filtered) {
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

	if (_control_status.flags.aux_gpos) {
		test_ratio = math::max(test_ratio, fabsf(_aux_global_position.testRatioFiltered()));
	}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		return sqrtf(test_ratio);
	}

	return NAN;
}

/**
 * @brief 获取垂直位置的创新测试比率（Innovation Test Ratio）
 *
 * 该函数计算垂直位置测量（通常为气压高度、GPS高度、范围传感器或EV位置Z）的创新测试比率，
 * 用于评估本次测量的创新（残差）是否在合理范围内（创新门限检查/gating）。
 */
float Ekf::getVerticalPositionInnovationTestRatio() const
{
	// return the combined vertical position innovation test ratio
	float hgt_sum = 0.f;
	int n_hgt_sources = 0;

#if defined(CONFIG_EKF2_BAROMETER)

	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_baro_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_gnss_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_ev_hgt.test_ratio_filtered));
		n_hgt_sources++;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (n_hgt_sources > 0) {
		return math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);
	}

	return NAN;
}

/**
 * @brief 返回气压高度的创新测试比率
 */
float Ekf::getAirspeedInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_AIRSPEED)

	if (_control_status.flags.fuse_aspd) {
		// return the airspeed fusion innovation test ratio
		return sqrtf(fabsf(_aid_src_airspeed.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_AIRSPEED

	return NAN;
}

float Ekf::getSyntheticSideslipInnovationTestRatio() const
{
#if defined(CONFIG_EKF2_SIDESLIP)

	if (_control_status.flags.fuse_beta) {
		// return the synthetic sideslip innovation test ratio
		return sqrtf(fabsf(_aid_src_sideslip.test_ratio_filtered));
	}

#endif // CONFIG_EKF2_SIDESLIP

	return NAN;
}

/**
 * @brief 返回高度上方地面的创新测试比率
 */
float Ekf::getHeightAboveGroundInnovationTestRatio() const
{
	// return the combined HAGL innovation test ratio
	float hagl_sum = 0.f;
	int n_hagl_sources = 0;

#if defined(CONFIG_EKF2_TERRAIN)

# if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_control_status.flags.opt_flow_terrain) {
		hagl_sum += sqrtf(math::max(fabsf(_aid_src_optical_flow.test_ratio_filtered[0]),
					    _aid_src_optical_flow.test_ratio_filtered[1]));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

# if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_terrain) {
		hagl_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered));
		n_hagl_sources++;
	}

# endif // CONFIG_EKF2_RANGE_FINDER

#endif // CONFIG_EKF2_TERRAIN

	if (n_hagl_sources > 0) {
		return math::max(hagl_sum / static_cast<float>(n_hagl_sources), FLT_MIN);
	}

	return NAN;
}

uint16_t Ekf::get_ekf_soln_status() const
{
	// LEGACY Mavlink bitmask containing state of estimator solution (see Mavlink ESTIMATOR_STATUS_FLAGS)
	union ekf_solution_status_u {
		struct {
			uint16_t attitude           : 1;
			uint16_t velocity_horiz     : 1;
			uint16_t velocity_vert      : 1;
			uint16_t pos_horiz_rel      : 1;
			uint16_t pos_horiz_abs      : 1;
			uint16_t pos_vert_abs       : 1;
			uint16_t pos_vert_agl       : 1;
			uint16_t const_pos_mode     : 1;
			uint16_t pred_pos_horiz_rel : 1;
			uint16_t pred_pos_horiz_abs : 1;
			uint16_t gps_glitch         : 1;
			uint16_t accel_error        : 1;
		} flags;
		uint16_t value;
	} soln_status{};

	// 1	ESTIMATOR_ATTITUDE	True if the attitude estimate is good
	soln_status.flags.attitude = attitude_valid();

	// 2	ESTIMATOR_VELOCITY_HORIZ	True if the horizontal velocity estimate is good
	soln_status.flags.velocity_horiz = isLocalHorizontalPositionValid();

	// 4	ESTIMATOR_VELOCITY_VERT	True if the vertical velocity estimate is good
	soln_status.flags.velocity_vert = isLocalVerticalVelocityValid() || isLocalVerticalPositionValid();

	// 8	ESTIMATOR_POS_HORIZ_REL	True if the horizontal position (relative) estimate is good
	soln_status.flags.pos_horiz_rel = isLocalHorizontalPositionValid();

	// 16	ESTIMATOR_POS_HORIZ_ABS	True if the horizontal position (absolute) estimate is good
	soln_status.flags.pos_horiz_abs = isGlobalHorizontalPositionValid();

	// 32	ESTIMATOR_POS_VERT_ABS	True if the vertical position (absolute) estimate is good
	soln_status.flags.pos_vert_abs = isVerticalAidingActive();

	// 64	ESTIMATOR_POS_VERT_AGL	True if the vertical position (above ground) estimate is good
#if defined(CONFIG_EKF2_TERRAIN)
	soln_status.flags.pos_vert_agl = isHeightAboveGroundEstimateValid();
#endif // CONFIG_EKF2_TERRAIN

	// 128	ESTIMATOR_CONST_POS_MODE	True if the EKF is in a constant position mode and is not using external measurements (eg GNSS or optical flow)
	soln_status.flags.const_pos_mode = _control_status.flags.fake_pos || _control_status.flags.valid_fake_pos
					   || _control_status.flags.vehicle_at_rest;

	// 256	ESTIMATOR_PRED_POS_HORIZ_REL	True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	soln_status.flags.pred_pos_horiz_rel = isHorizontalAidingActive();

	// 512	ESTIMATOR_PRED_POS_HORIZ_ABS	True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	soln_status.flags.pred_pos_horiz_abs = _control_status.flags.gnss_pos || _control_status.flags.aux_gpos;

	// 1024	ESTIMATOR_GPS_GLITCH	True if the EKF has detected a GNSS glitch
#if defined(CONFIG_EKF2_GNSS)
	const bool gnss_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gnss_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;
	soln_status.flags.gps_glitch = (gnss_vel_innov_bad || gnss_pos_innov_bad);
#endif // CONFIG_EKF2_GNSS

	// 2048	ESTIMATOR_ACCEL_ERROR	True if the EKF has detected bad accelerometer data
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical || _fault_status.flags.bad_acc_clipping;

	return soln_status.value;
}

/**
 * @brief 执行卡尔曼滤波的状态修正步骤（应用增益和创新更新状态向量）
 *
 * 该函数仅负责测量更新中的状态估计修正部分：
 * 使用预先计算的卡尔曼增益和创新值，直接更新状态向量。
 * 不更新协方差矩阵P（协方差更新通常在调用者或measurementUpdate中完成）。
 *
 * 典型公式：x = x + K * innovation
 *
 * @param[in] K           卡尔曼增益向量（状态维度×1）
 *                        通常由 measurementUpdate() 或类似函数计算得到
 * @param[in] innovation  创新值（标量），即 measurement - H * x_pred
 *
 * @return void           无返回值，直接就地修改成员变量 _state（或等效状态向量）
 *
 * @note
 *       - 该函数仅更新状态估计，不涉及协方差更新。
 *       - 常用于标量测量场景，与计算增益和协方差更新的函数配合使用。
 *       - 在某些直接状态测量融合（如fuseDirectStateMeasurement）内部也会调用类似逻辑。
 *
 * @see measurementUpdate()     完整标量测量更新（计算K并更新状态和协方差）
 * @see fuseDirectStateMeasurement()  直接状态测量的融合函数（内部可能调用fuse）
 */
void Ekf::fuse(const VectorState &K, float innovation)
{
	// quat_nominal
	// 翻译：名义四元数
	Quatf delta_quat(matrix::AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx,
					    0) * (-1.f * innovation)));
	_state.quat_nominal = delta_quat * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// vel
	_state.vel = matrix::constrain(_state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation, -1.e3f, 1.e3f);

	// pos
	const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);

	// Accumulate position in global coordinates
	// 翻译：累积位置在全局坐标中
	_gpos += pos_correction;
	_state.pos.zero();
	// Also store altitude in the state vector as this is used for optical flow fusion
	// 翻译：也存储高度在状态向量中，因为这是用于光学流融合的
	_state.pos(2) = -_gpos.altitude();

	// gyro_bias
	_state.gyro_bias = matrix::constrain(_state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx,
					     0) * innovation,
					     -getGyroBiasLimit(), getGyroBiasLimit());

	// accel_bias
	_state.accel_bias = matrix::constrain(_state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx,
					      0) * innovation,
					      -getAccelBiasLimit(), getAccelBiasLimit());

#if defined(CONFIG_EKF2_MAGNETOMETER)

	// mag_I, mag_B
	if (_control_status.flags.mag) {
		_state.mag_I = matrix::constrain(_state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation, -1.f,
						 1.f);
		_state.mag_B = matrix::constrain(_state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
						 -getMagBiasLimit(), getMagBiasLimit());
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)

	// wind_vel
	if (_control_status.flags.wind) {
		_state.wind_vel = matrix::constrain(_state.wind_vel - K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx,
						    0) * innovation, -1.e2f, 1.e2f);
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	_state.terrain = math::constrain(_state.terrain - K(State::terrain.idx) * innovation, -1e4f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

/**
 * @brief 获取死区状态更新
 */
void Ekf::updateDeadReckoningStatus()
{
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

/**
 * @brief 获取水平方向上的死区状态更新
 */
void Ekf::updateHorizontalDeadReckoningstatus()
{
	bool inertial_dead_reckoning = true;
	bool aiding_expected_in_air = false;

	// velocity aiding active
	// 翻译：速度辅助激活
	if ((_control_status.flags.gnss_vel || _control_status.flags.ev_vel)
	    && isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

	// position aiding active
	if ((_control_status.flags.gnss_pos || _control_status.flags.ev_pos
	     || _control_status.flags.aux_gpos || _control_status.flags.rngbcn_fusion)
	    && isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// optical flow active
	// 翻译：光流激活
	if (_control_status.flags.opt_flow
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;

	} else {
		if (!_control_status.flags.in_air && _fc.of.intended()
		    && isRecent(_aid_src_optical_flow.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but optical flow aiding should be possible once in air
			// 翻译：目前着陆，但光流辅助应该在空中可用
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)

	// air data aiding active
	// 翻译：空气数据辅助激活
	if ((_control_status.flags.fuse_aspd && isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max))
	    && (_control_status.flags.fuse_beta && isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max))
	   ) {
		// wind_dead_reckoning: no other aiding but air data
		// 翻译：风死记：没有其他辅助但空气数据
		_control_status.flags.wind_dead_reckoning = inertial_dead_reckoning;

		// air data aiding is active, we're not inertial dead reckoning
		// 翻译：空气数据辅助激活，我们不是惯性死记
		inertial_dead_reckoning = false;

	} else {
		_control_status.flags.wind_dead_reckoning = false;

		if (!_control_status.flags.in_air && _control_status.flags.fixed_wing
		    && (_params.ekf2_fuse_beta == 1)
		    && _fc.aspd.intended() && isRecent(_aid_src_airspeed.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			// currently landed, but air data aiding should be possible once in air
			// 翻译：目前着陆，但空气数据辅助应该在空中可用
			aiding_expected_in_air = true;
		}
	}

#endif // CONFIG_EKF2_AIRSPEED

	// zero velocity update
	// 翻译：零速度更新
	if (isRecent(_zero_velocity_update.time_last_fuse(), _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		// 翻译：只有在我们期望在空中有另一个有效的来源时才尊重它作为有效的辅助来源
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (_control_status.flags.valid_fake_pos && isRecent(_aid_src_fake_pos.time_last_fuse, _params.no_aid_timeout_max)) {
		// only respect as a valid aiding source now if we expect to have another valid source once in air
		// 翻译：只有在我们期望在空中有另一个有效的来源时才尊重它作为有效的辅助来源
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	if (inertial_dead_reckoning) {
		if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.ekf2_noaid_tout)) {
			// deadreckon time exceeded
			// 翻译：死区时间超时
			if (!_horizontal_deadreckon_time_exceeded) {
				ECL_WARN("horizontal dead reckon time exceeded");
				_horizontal_deadreckon_time_exceeded = true;
			}
		}

	} else {
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}

		_horizontal_deadreckon_time_exceeded = false;

	}

	_control_status.flags.inertial_dead_reckoning = inertial_dead_reckoning;
}

/**
 * @brief 获取垂直方向上的死区状态更新
 */
void Ekf::updateVerticalDeadReckoningStatus()
{
	if (isVerticalPositionAidingActive()) {
		_time_last_v_pos_aiding = _time_last_hgt_fuse;
		_vertical_position_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_pos_aiding, (uint64_t)_params.ekf2_noaid_tout)) {
		_vertical_position_deadreckon_time_exceeded = true;
	}

	if (isVerticalVelocityAidingActive()) {
		_time_last_v_vel_aiding = _time_last_ver_vel_fuse;
		_vertical_velocity_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_vel_aiding, (uint64_t)_params.ekf2_noaid_tout)
		   && _vertical_position_deadreckon_time_exceeded) {

		_vertical_velocity_deadreckon_time_exceeded = true;
	}
}

/**
 * @brief 获取机体坐标系（Body frame）下的姿态旋转误差方差（对角元素）
 *
 * 该函数从状态协方差矩阵中提取四元数（quat_nominal）对应的3x3协方差子块，
 * 表示姿态误差在本地导航坐标系（NED/Earth frame）下的协方差，
 * 然后通过当前姿态旋转矩阵将该协方差旋转到机体坐标系（Body frame），
 * 最后返回旋转后协方差矩阵的对角元素，即roll、pitch、yaw三个轴在机体坐标系下的方差。
 *
 * @return Vector3f 机体坐标系下三个旋转轴的误差方差（rad²）
 *         - x: roll轴方差
 *         - y: pitch轴方差
 *         - z: yaw轴方差
 *
 * @note
 *       - 姿态误差协方差最初定义在NED坐标系中（tilt error + yaw error）。
 *       - 通过相似变换 _R_to_earth^T * cov_ned * _R_to_earth 得到机体坐标系下的协方差。
 *       - 仅返回对角元素（即各轴独立方差），不返回完整协方差矩阵。
 *       - 常用于机载传感器（如磁力计、外部视觉姿态）的创新门限检查或诊断输出。
 *
 * @see getRotVarNed() 获取NED坐标系下的姿态方差
 */
Vector3f Ekf::getRotVarBody() const
{
	const matrix::SquareMatrix3f rot_cov_body = getStateCovariance<State::quat_nominal>();
	return matrix::SquareMatrix3f(_R_to_earth.T() * rot_cov_body * _R_to_earth).diag();
}

/**
 * @brief 获取本地导航坐标系（NED frame）下的姿态旋转误差方差（对角元素）
 *
 * 该函数直接从状态协方差矩阵中提取四元数（quat_nominal）对应的3x3协方差子块，
 * 并返回其对角元素，即roll/pitch tilt误差和yaw误差在NED坐标系下的方差。
 *
 * @return Vector3f NED坐标系下三个旋转轴的误差方差（rad²）
 *         - x: roll轴方差（或对应tilt误差分量）
 *         - y: pitch轴方差（或对应tilt误差分量）
 *         - z: yaw轴方差
 *
 * @note
 *       - 姿态误差协方差在EKF中天然定义在NED/Earth坐标系中。
 *       - 仅返回对角元素，不包含交叉协方差项。
 *       - 常用于整体姿态不确定性评估、yaw对齐检查或estimator_status输出。
 *
 * @see getRotVarBody() 获取旋转到机体坐标系后的姿态方差
 */
Vector3f Ekf::getRotVarNed() const
{
	const matrix::SquareMatrix3f rot_cov_ned = getStateCovariance<State::quat_nominal>();
	return rot_cov_ned.diag();
}

/**
 * @brief 计算航向角的方差
 */
float Ekf::getYawVar() const
{
	return getRotVarNed()(2);
}

/**
 * @brief 计算倾斜角的方差
 */
float Ekf::getTiltVariance() const
{
	const Vector3f rot_var_ned = getRotVarNed();
	return rot_var_ned(0) + rot_var_ned(1);
}

/**
 * @brief 更新地面效应
 */
#if defined(CONFIG_EKF2_BAROMETER)
void Ekf::updateGroundEffect()
{
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
#if defined(CONFIG_EKF2_TERRAIN)

		if (isHeightAboveGroundEstimateValid()) {
			// automatically set ground effect if HAGL is valid
			float height = getHagl();
			_control_status.flags.gnd_effect = (height < _params.ekf2_gnd_max_hgt);

		} else
#endif // CONFIG_EKF2_TERRAIN
			if (_control_status.flags.gnd_effect) {
				// Turn off ground effect compensation if it times out
				// 翻译注释：如果地面效应超时，则关闭地面效应补偿
				if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {
					_control_status.flags.gnd_effect = false;
				}
			}

	} else {
		_control_status.flags.gnd_effect = false;
	}
}
#endif // CONFIG_EKF2_BAROMETER


/**
 * @brief 更新IMU偏置抑制
 * @param imu_delayed 延迟的IMU数据
 */
void Ekf::updateIMUBiasInhibit(const imuSample &imu_delayed)
{
	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	{
		const Vector3f gyro_corrected = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - _state.gyro_bias;

		const float alpha = math::constrain((imu_delayed.delta_ang_dt / _params.ekf2_abl_tau), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_ang_rate_magnitude_filt = fmaxf(gyro_corrected.norm(), beta * _ang_rate_magnitude_filt);
	}

	{
		const Vector3f accel_corrected = imu_delayed.delta_vel / imu_delayed.delta_vel_dt - _state.accel_bias;

		const float alpha = math::constrain((imu_delayed.delta_vel_dt / _params.ekf2_abl_tau), 0.f, 1.f);
		const float beta = 1.f - alpha;

		_accel_magnitude_filt = fmaxf(accel_corrected.norm(), beta * _accel_magnitude_filt);
	}


	const bool is_manoeuvre_level_high = (_ang_rate_magnitude_filt > _params.ekf2_abl_gyrlim)
					     || (_accel_magnitude_filt > _params.ekf2_abl_acclim);


	// gyro bias inhibit
	// 翻译：gyro 偏置抑制
	const bool do_inhibit_all_gyro_axes = !(_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		bool is_bias_observable = true; // TODO: gyro bias conditions
		_gyro_bias_inhibit[index] = do_inhibit_all_gyro_axes || !is_bias_observable;
	}

	// accel bias inhibit
	// 翻译：accel 偏置抑制
	const bool do_inhibit_all_accel_axes = !(_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
					       || is_manoeuvre_level_high
					       || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		bool is_bias_observable = true;

		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos || _control_status.flags.gravity_vector) {
			// only consider an accel bias observable if aligned with the gravity vector
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f); // cos 15 degrees ~= 0.966
		}

		_accel_bias_inhibit[index] = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;
	}
}

void Ekf::fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index,
				     bool constrain_variances)
{
	VectorState K;  // Kalman gain vector for any single observation - sequential fusion is used.

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// This is extremely expensive to compute. Use for debugging purposes only.
	auto A = matrix::eye<float, State::size>();
	VectorState H;
	H(state_index) = 1.f;
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
	VectorState PH = P.row(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetric if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	// P (or "P_temp") is not symmetric so we must take the column
	PH = P.col(state_index);

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	if (constrain_variances) {
		constrainStateVariances();
	}

	// apply the state corrections
	// 翻译：应用状态修正
	fuse(K, innov);
}

/**
 * @brief 执行标量测量的卡尔曼滤波更新（约瑟夫形式）
 *
 * 该函数针对单个标量测量值执行卡尔曼滤波的测量更新步骤，
 * 使用数值稳定的约瑟夫形式（Joseph stabilized form）更新协方差矩阵。
 *
 * @param[out] k   计算得到的卡尔曼增益（标量，状态维度中对应维度的增益）
 * @param[in]  H   测量矩阵的对应行向量（状态向量，1×n），将状态映射到该标量测量
 * @param[in]  R   测量噪声方差（标量）
 * @param[in]  innovation  创新（残差），即 measurement - H * x_pred
 *
 * @return bool 更新是否成功
 *         - true:  更新成功，状态和协方差已更新
 *         - false: 更新失败（如创新方差为非正、数值异常等）
 *
 * @note 该函数会就地修改成员变量：
 *       - 状态估计: _state += k * innovation
 *       - 协方差:    P = (I - k*H^T) * P * (I - k*H^T)^T + k² * R   （约瑟夫形式）
 *
 * @see 向量测量版本的 measurementUpdate()
 */
bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
	// 清除抑制状态下的卡尔曼增益
	clearInhibitedStateKalmanGains(K);

#if false
	// Matrix implementation of the Joseph stabilized covariance update
	// 翻译：约瑟夫稳定协方差更新的矩阵实现
	// This is extremely expensive to compute. Use for debugging purposes only.
	// 翻译：这计算起来非常昂贵。 仅用于调试目的。
	auto A = matrix::eye<float, State::size>();
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// Efficient implementation of the Joseph stabilized covariance update
	// Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
	// P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
	//   =      P_temp     * (I - H.T * K.T) + K * R * K.T
	//   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

	// Step 1: conventional update
	// Compute P_temp and store it in P to avoid allocating more memory
	// P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major

	// 翻译：约瑟夫稳定协方差更新的高效实现
	// 基于“G. J. Bierman。离散序列估计的因式分解方法。学术出版社，多佛出版社，纽约，1977 年，2006 年”
	// P = (I - K * H) * P * (I - K * H).T K * R * K.T
	// = P_temp * (I - H.T * K.T) K * R * K.T
	// = P_temp - P_temp * H.T * K.T K * R * K.T

	// 步骤1：常规更新
	// 计算 P_temp 并将其存储在 P 中以避免分配更多内存
	// P 是对称的，因此 PH == H.T * P.T == H.T * P。由于矩阵是行优先的，所以取行速度更快
	VectorState PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
		}
	}

	// Step 2: stabilized update
	// 翻译：Step 2: 稳定化更新
	PH = P * H; // H is stored as a column vector. H is in fact H.T

	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}

#endif

	constrainStateVariances();

	// apply the state corrections
	// 翻译：应用状态修正
	fuse(K, innovation);
	return true;
}

/**
 * @brief 重置辅助源状态为零创新
 * @param status 状态结构体
 */
void Ekf::resetAidSourceStatusZeroInnovation(estimator_aid_source1d_s &status) const
{
	status.time_last_fuse = _time_delayed_us;

	status.innovation = 0.f;
	status.innovation_filtered = 0.f;
	status.innovation_variance = status.observation_variance;

	status.test_ratio = 0.f;
	status.test_ratio_filtered = 0.f;

	status.innovation_rejected = false;
	status.fused = true;
}

/**
 * @brief 更新辅助源状态
 * @param status 状态结构体
 * @param timestamp_sample 时间戳
 * @param observation 观测值
 * @param observation_variance 观测方差
 * @param innovation 创新值
 * @param innovation_variance 创新方差
 * @param innovation_gate 创新门限
 */
void Ekf::updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				const float &observation, const float &observation_variance,
				const float &innovation, const float &innovation_variance,
				float innovation_gate) const
{
	bool innovation_rejected = false;

	const float test_ratio = sq(innovation) / (sq(innovation_gate) * innovation_variance);

	if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f;
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		// test_ratio_filtered
		if (PX4_ISFINITE(status.test_ratio_filtered)) {
			status.test_ratio_filtered += alpha * (matrix::sign(innovation) * test_ratio - status.test_ratio_filtered);

		} else {
			// otherwise, init the filtered test ratio
			// 翻译：否则，初始化过滤测试比率
			status.test_ratio_filtered = test_ratio;
		}

		// innovation_filtered
		// 翻译：创新_过滤
		if (PX4_ISFINITE(status.innovation_filtered)) {
			status.innovation_filtered += alpha * (innovation - status.innovation_filtered);

		} else {
			// otherwise, init the filtered innovation
			// 翻译：否则，初始化过滤的创新
			status.innovation_filtered = innovation;
		}


		// limit extremes in filtered values
		// 翻译：限制滤波值中的极值
		static constexpr float kNormalizedInnovationLimit = 2.f;
		static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit);

		if (test_ratio > kTestRatioLimit) {

			status.test_ratio_filtered = math::constrain(status.test_ratio_filtered, -kTestRatioLimit, kTestRatioLimit);

			const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance);
			status.innovation_filtered = math::constrain(status.innovation_filtered, -innov_limit, innov_limit);
		}

	} else {
		// invalid timestamp_sample, reset
		status.test_ratio_filtered = test_ratio;
		status.innovation_filtered = innovation;
	}

	status.test_ratio = test_ratio;

	status.observation = observation;
	status.observation_variance = observation_variance;

	status.innovation = innovation;
	status.innovation_variance = innovation_variance;

	if ((test_ratio > 1.f)
	    || !PX4_ISFINITE(test_ratio)
	    || !PX4_ISFINITE(status.innovation)
	    || !PX4_ISFINITE(status.innovation_variance)
	   ) {
		innovation_rejected = true;
	}

	status.timestamp_sample = timestamp_sample;

	// if any of the innovations are rejected, then the overall innovation is rejected
	status.innovation_rejected = innovation_rejected;

	// reset
	status.fused = false;
}

/**
 * @brief 清除抑制状态的卡尔曼增益
 * @param K 卡尔曼增益向量
 */
void Ekf::clearInhibitedStateKalmanGains(VectorState &K) const
{
	if (!_control_status.flags.heading_observable) {
		K(State::quat_nominal.idx + 2) = 0.f;
	}

	for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
		if (_gyro_bias_inhibit[i]) {
			K(State::gyro_bias.idx + i) = 0.f;
		}
	}

	for (unsigned i = 0; i < State::accel_bias.dof; i++) {
		if (_accel_bias_inhibit[i]) {
			K(State::accel_bias.idx + i) = 0.f;
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f;
		}
	}

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f;
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER
}

/**
 * @brief 获取航向创新值
 */
float Ekf::getHeadingInnov() const
{
	float innov = 0.f;

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		innov = Vector3f(_aid_src_mag.innovation).max();

	} else {
		innov = _mag_heading_innov_lpf.getState();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		innov = _aid_src_gnss_yaw.innovation;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		innov = _aid_src_ev_yaw.innovation;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return innov;
}

/**
 * @brief 获取航向创新方差
 */
float Ekf::getHeadingInnovVar() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation_variance).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation_variance;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}

/**
 * @brief 获取航向创新比率
 */
float Ekf::getHeadingInnovRatio() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.test_ratio).max();
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.test_ratio;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f;
}
