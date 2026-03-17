/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.hpp
 *
 * Flight task for manual position controlled mode.
 *
 */

#pragma once

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include <lib/weather_vane/WeatherVane.hpp>
#include "FlightTaskManualAltitude.hpp"

class FlightTaskManualPosition : public FlightTaskManualAltitude
{
public:
	FlightTaskManualPosition() = default;
	virtual ~FlightTaskManualPosition() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;

protected:
	void _updateXYlock(); /**< applies position lock based on stick and velocity */
	void _updateSetpoints() override;
	void _scaleSticks() override;

	/**
	 * @param _param_mpc_vel_manual     手动模式下水平方向（前后+侧向）最大速度（摇杆满偏时的目标速度，单位：m/s，通常 5~10 m/s）
	 * @param _param_mpc_vel_man_side   手动模式下纯侧向（左右）最大速度（如果与 MPC_VEL_MANUAL 不同，可单独限制侧飞速度）
	 * @param _param_mpc_vel_man_back   手动模式下向后最大速度（通常比向前小，以限制后退速度，单位：m/s）
	 * @param _param_mpc_acc_hor_max    手动模式下水平方向最大加速度（m/s²，限制摇杆响应时的加速快慢，通常 3~5 m/s²）
	 * @param _param_mpc_hold_max_xy    位置保持（松开摇杆时）允许的最大水平漂移/误差（m，超过此值可能重新激活定位控制）
	 */
	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAltitude,
					(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
					(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,
					(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,
					(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy
				       )
private:
	uint8_t _reset_counter{0}; /**< counter for estimator resets in xy-direction */

	WeatherVane _weathervane{this}; /**< weathervane library, used to implement a yaw control law that turns the vehicle nose into the wind */
	CollisionPrevention _collision_prevention{this}; /**< collision prevention setpoint amendment */
};
