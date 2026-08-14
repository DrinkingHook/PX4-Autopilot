/****************************************************************************
 *
 *   Copyright (c) 2018-2026 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAuto.hpp
 *
 * Map from global triplet to local quadruple.
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/takeoff_status.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/sticks/Sticks.hpp>
#include <lib/stick_yaw/StickYaw.hpp>
#include <lib/weather_vane/WeatherVane.hpp>
#include "StickAccelerationXY.hpp"

/**
 * This enum has to agree with position_setpoint_s type definition
 * The only reason for not using the struct position_setpoint is because
 * of the size
 */
enum class WaypointType : int {
	position = position_setpoint_s::SETPOINT_TYPE_POSITION,
	velocity = position_setpoint_s::SETPOINT_TYPE_VELOCITY,
	loiter = position_setpoint_s::SETPOINT_TYPE_LOITER,
	takeoff = position_setpoint_s::SETPOINT_TYPE_TAKEOFF,
	land = position_setpoint_s::SETPOINT_TYPE_LAND,
	idle = position_setpoint_s::SETPOINT_TYPE_IDLE
};

enum class yaw_mode : int32_t {
	towards_waypoint = 0,
	towards_home = 1,
	away_from_home = 2,
	along_trajectory = 3,
	towards_waypoint_yaw_first = 4,
	yaw_fixed = 5,
};

class FlightTaskAuto : public FlightTask
{
public:
	FlightTaskAuto() = default;
	virtual ~FlightTaskAuto() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

	void overrideCruiseSpeed(const float cruise_speed_m_s) override;

protected:
	bool _compute_heading_from_2D_vector(float &heading, matrix::Vector2f v); /**< Computes and sets heading a 2D vector */

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy) override;
	void _ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy) override;
	void _ekfResetHandlerPositionZ(float delta_z) override;
	void _ekfResetHandlerVelocityZ(float delta_vz) override;
	void _ekfResetHandlerHeading(float delta_psi) override;

	void _checkEmergencyBraking();
	bool _generateHeadingAlongTraj(); /**< Generates heading along trajectory. */
	bool isTargetModified() const;
	void _updateTrajConstraints();

	void rcHelpModifyYaw(float &yaw_sp);

	/** determines when to trigger a takeoff (ignored in flight) */
	bool _checkTakeoff() override { return _want_takeoff; };

	void extracted(Vector2f &sticks_xy, Vector2f &sticks_ne,
		       float &max_speed);
	void _prepareLandSetpoints();
	bool _highEnoughForLandingGear(); /**< Checks if gears can be lowered. */

	void updateParams() override; /**< See ModuleParam class */

	bool _prev_was_valid{false};
	bool _next_was_valid{false};
	float _mc_cruise_speed{NAN}; /**< Requested cruise speed. If not valid, default cruise speed is used. */
	WaypointType _type{WaypointType::idle}; /**< Type of current target triplet. */

	uORB::SubscriptionData<position_setpoint_triplet_s> _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::SubscriptionData<home_position_s> _sub_home_position{ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_status_s> _sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<takeoff_status_s> _takeoff_status_sub{ORB_ID(takeoff_status)};

	float _target_acceptance_radius{0.0f}; /**< Acceptances radius of the target */

	float _yaw_setpoint_previous{NAN}; /**< Used because _yaw_setpoint is overwritten in multiple places */
	float _triplet_yaw{NAN}; /**< Last yaw from position_setpoint_triplet, to detect navigator changes */
	bool _manual_yaw_active{false};
	uint8_t _nav_state_prev{0};
	HeadingSmoothing _heading_smoothing;
	bool _yaw_sp_aligned{false};

	PositionSmoothing _position_smoothing;
	Vector3f _unsmoothed_velocity_setpoint;
	Sticks _sticks{this};
	StickAccelerationXY _stick_acceleration_xy{this};
	StickYaw _stick_yaw{this};
	matrix::Vector3f _land_position;
	WaypointType _type_previous{WaypointType::idle}; /**< Previous type of current target triplet. */
	bool _is_emergency_braking_active{false};
	bool _want_takeoff{false};

	/**
	 * @param _param_mpc_xy_cruise      自动模式（Mission/Offboard等）下的默认水平巡航速度（通常用于航点间飞行时的目标速度）
	 * @param _param_nav_mc_alt_rad     垂直方向航点接受半径，到达该高度范围内视为到达航点并切换下一个航点
	 * @param _param_mpc_yaw_mode       定义自动模式下航向（yaw）的控制方式（0=朝向航点、1=沿轨迹、2=固定航向、3=朝向下一个航点先转 yaw 等）
	 * @param _param_mpc_yawrauto_acc   自动模式下 yaw 角加速度（决定 yaw 转动快慢的加速度）
	 * @param _param_mpc_yawrauto_max   自动模式下最大 yaw 角速度（单位：°/s 或 rad/s，根据版本）
	 * @param _param_mis_yaw_err        任务模式下航向误差阈值，超过此值可能触发某些行为或判定未对准
	 * @param _param_mpc_acc_hor        水平方向（xy）的最大加速度（用于自动模式和部分手动位置控制）
	 * @param _param_mpc_acc_up_max     垂直向上最大加速度（爬升时限制）
	 * @param _param_mpc_acc_down_max   垂直向下最大加速度（下降时限制，通常比向上小以保护安全）
	 * @param _param_mpc_jerk_auto      自动模式下 jerk（加加速度）限制，用于平滑轨迹规划（jerk-limited trajectory）
	 * @param _param_mpc_xy_traj_p      水平轨迹位置控制 P 增益（用于轨迹跟踪的修正力度）
	 * @param _param_mpc_xy_err_max     水平位置误差最大允许值，超过可能触发保护或切换行为
	 * @param _param_mpc_land_speed     自动降落时的下降速度（最后阶段的目标下降速率，通常 0.6~1.0 m/s 左右）
	 * @param _param_mpc_land_crwl      降落爬行（crawl）阶段的下降速度（接近地面时极低速阶段，配合 MPC_LAND_ALTx 使用）
	 * @param _param_mpc_auto_nudging   在自主模式下启用摇杆轻推(bit0 - 偏航轻推 bit1 - 降落轻推)
	 * @param _param_mpc_land_radius    降落接受半径（水平方向，认为进入该圆内开始执行降落逻辑）
	 * @param _param_mpc_land_alt1      降落第一阶段切换高度（通常较高，进入慢速下降）
	 * @param _param_mpc_land_alt2      降落第二阶段切换高度（更低，进入更慢下降或爬行阶段）
	 * @param _param_mpc_land_alt3      降落第三阶段切换高度（非常接近地面，常配合激光雷达使用）
	 * @param _param_mpc_z_v_auto_up    自动模式下垂直向上最大/目标速度（爬升速度）
	 * @param _param_mpc_z_v_auto_dn    自动模式下垂直向下最大/目标速度（非降落时的下降速度）
	 * @param _param_mpc_tko_speed      起飞时的爬升速度（垂直向上速度，从起飞到安全高度）
	 * @param _param_mpc_tko_ramp_t     起飞油门斜坡时间（throttle ramp time），控制起飞时油门缓慢增加的时间，避免突然冲击
	 */
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
					_param_nav_mc_alt_rad, //vertical acceptance radius at which waypoints are updated
					(ParamInt<px4::params::MPC_YAW_MODE>) _param_mpc_yaw_mode, // defines how heading is executed,
					(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc,
					(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
					(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err, // yaw-error threshold
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, // acceleration in flight
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
					(ParamFloat<px4::params::MPC_Z_ERR_MAX>) _param_mpc_z_err_max,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
					(ParamFloat<px4::params::MPC_LAND_CRWL>) _param_mpc_land_crwl,
					(ParamInt<px4::params::MPC_AUTO_NUDGING>) _param_mpc_auto_nudging,
					(ParamFloat<px4::params::MPC_LAND_RADIUS>) _param_mpc_land_radius,
					(ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1,
					(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2,
					(ParamFloat<px4::params::MPC_LAND_ALT3>) _param_mpc_land_alt3,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
					(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t
				       );

private:
	matrix::Vector2f _lock_position_xy; /**< if no valid triplet is received, lock positition to current position */
	matrix::Vector3f _takeoff_liftoff_position; /**< tracks the position state during the takeoff ramp and is frozen at FLIGHT */
	bool _yaw_lock{false}; /**< if within acceptance radius, lock yaw to current yaw */

	matrix::Vector3f _triplet_previous; ///< previous waypoint in triplet from navigator
	matrix::Vector3f _triplet_current; ///< current waypoint in triplet from navigator
	matrix::Vector3f _triplet_next; ///< next waypoint in triplet from navigator

	hrt_abstime _time_last_cruise_speed_override{0}; ///< timestamp the cruise speed was last time overridden using DO_CHANGE_SPEED

	MapProjection _reference_position{}; /**< Class used to project lat/lon setpoint into local frame. */
	float _reference_altitude{NAN}; /**< Altitude relative to ground. */
	hrt_abstime _time_stamp_reference{0}; /**< time stamp when last reference update occured. */

	WeatherVane _weathervane{this}; /**< weathervane library, used to implement a yaw control law that turns the vehicle nose into the wind */

	matrix::Vector3f _initial_land_position;

	void _smoothYaw(); /**< Smoothen the yaw setpoint. */
	bool _evaluatePositionSetpointTriplet();
	bool _isFinite(const position_setpoint_s &sp); /**< Checks if all waypoint triplets are finite. */
	bool _evaluateGlobalReference(); /**< Check is global reference is available. */
	void _set_heading_from_mode(); /**< @see  MPC_YAW_MODE */
};
