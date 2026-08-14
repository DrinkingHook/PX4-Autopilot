/****************************************************************************
 *
 *   Copyright (c) 2013-2024 PX4 Development Team. All rights reserved.
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
 * @file rtl_direct.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include <float.h>

#include "rtl_direct.h"
#include "mission_item_utils.h"
#include "navigator.h"
#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
#include "rtl_geofence_avoidance_helper.h"
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
#include <px4_platform_common/events.h>

#include <lib/geo/geo.h>

using namespace math;

RtlDirect::RtlDirect(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL),
	ModuleParams(navigator)
{
	_land_approach.lat = static_cast<double>(NAN);
	_land_approach.lon = static_cast<double>(NAN);
	_land_approach.height_m = NAN;
}

void RtlDirect::on_inactivation()
{
	if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}

	_rtl_state = RTLState::IDLE;
}

void RtlDirect::on_activation()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	_rtl_state = getActivationState();

	// save the setpoint the previous mode left before resetting the triplet, so the climb can
	// continue an already-established loiter (used in set_rtl_item(), CLIMBING state)
	// 翻译：在重置三元组之前，保存上一模式留下的设定点，以便爬升可以延续已建立的悬停状态(在set_rtl_item()函数中，CLIMBING状态下使用)
	_setpoint_on_activation = _navigator->get_position_setpoint_triplet()->current;
	_navigator->reset_triplets();

	// reset cruising speed and throttle to default for RTL
	// 翻译：重置巡航速度和默认的RTL
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	set_rtl_item();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: start return at %d m (%d m above destination)\t",
			 (int)ceilf(_rtl_alt), (int)ceilf(_rtl_alt - _destination.alt));
	events::send<int32_t, int32_t>(events::ID("vrtl_return_at"), events::Log::Info,
				       "RTL: start return at {1m_v} ({2m_v} above destination)",
				       (int32_t)ceilf(_rtl_alt), (int32_t)ceilf(_rtl_alt - _destination.alt));

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	if (geofenceAvoidanceActive()) {
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: avoiding geofence\t");
		events::send(events::ID("rtl_avoiding_geofence"), events::Log::Info, "RTL: avoiding geofence");

	} else if (_navigator->get_geofence_avoidance_planner().needsStraightLineFallback()) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: no geofence avoidance path; flying directly\t");
		events::send(events::ID("rtl_avoidance_runtime_fallback"), {events::Log::Critical, events::LogInternal::Error},
			     "RTL: no geofence avoidance path; flying directly");
	}

#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
}

void RtlDirect::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	if (_rtl_state != RTLState::IDLE && is_mission_item_reached_or_completed()) {
		_updateRtlState();
		set_rtl_item();
	}

	if (_rtl_state != RTLState::IDLE && _rtl_state != RTLState::LAND) {
		//check for terrain collision and update altitude if needed
		// note: it may trigger multiple times during a RTL, as every time the altitude set is reset
		// 翻译：检查地形碰撞并更新高度，如果需要的话。
		//      note: 在RTL过程中，每次高度设置重置时，可能会触发多次。
		updateAltToAvoidTerrainCollisionAndRepublishTriplet(_mission_item);
	}

	if (_rtl_state == RTLState::LAND && _mission_item.land_precision > 0) {
		// Need to update the position and type on the current setpoint triplet.
		// 翻译：需要更新当前设置点三元组的位置和类型。
		_navigator->get_precland()->on_active();

	} else if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}

	if (_rtl_state == RTLState::IDLE) {
		_navigator->mode_completed(getNavigatorStateId());
	}
}

void RtlDirect::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
}

void RtlDirect::setRtlPosition(const PositionYawSetpoint &rtl_position, const loiter_point_s &loiter_pos)
{
	parameters_update();

	// Only allow to set a new approach if the mode is not activated yet.
	// 翻译：如果模式未激活，则允许设置新的方法。
	if (!isActive()) {
		_destination = rtl_position;
		_force_heading = false;

		// 判断数据有效性，做保底
		_land_approach = sanitizeLandApproach(loiter_pos);

		const float dist_to_destination{get_distance_to_next_waypoint(_land_approach.lat, _land_approach.lon, _destination.lat, _destination.lon)};

		if (dist_to_destination > _navigator->get_acceptance_radius()) {
			_force_heading = true;
		}
	}
}

bool RtlDirect::geofenceAvoidanceActive() const
{
#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
	return _navigator->get_geofence_avoidance_planner().hasMore();
#else
	return false;
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
}

void RtlDirect::_updateRtlState()
{
	// RTL_LAND_DELAY > 0 -> wait seconds, < 0 wait indefinitely
	const bool wait_at_rtl_descend_alt = fabsf(_param_rtl_land_delay.get()) > FLT_EPSILON;
	const bool is_multicopter = (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

	RTLState new_state{RTLState::IDLE};

	switch (_rtl_state) {
	case RTLState::CLIMBING:
	case RTLState::AVOID_GEOFENCE:
		new_state = geofenceAvoidanceActive() ? RTLState::AVOID_GEOFENCE : RTLState::MOVE_TO_LOITER;
		break;

	case RTLState::MOVE_TO_LOITER:
		if (!is_multicopter || wait_at_rtl_descend_alt) {
			new_state = RTLState::LOITER_DOWN;

		} else {
			new_state = RTLState::LAND;
		}

		break;

	case RTLState::LOITER_DOWN:
		new_state = RTLState::LOITER_HOLD;
		break;

	case RTLState::LOITER_HOLD:
		if (_vehicle_status_sub.get().is_vtol
		    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			new_state = RTLState::MOVE_TO_LAND;

		} else {
			new_state = RTLState::MOVE_TO_LAND_HOVER;
		}

		break;

	case RTLState::MOVE_TO_LAND:
		new_state = RTLState::TRANSITION_TO_MC;
		break;

	case RTLState::TRANSITION_TO_MC:
		new_state = RTLState::MOVE_TO_LAND_HOVER;
		break;

	case RTLState::MOVE_TO_LAND_HOVER:
		new_state = RTLState::LAND;
		break;

	case RTLState::LAND:
		new_state = RTLState::IDLE;
		break;

	case RTLState::IDLE: // Fallthrough
	default:
		new_state = RTLState::IDLE;
		break;
	}

	_rtl_state = new_state;
}

void RtlDirect::set_rtl_item()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
				       _global_pos_sub.get().lat, _global_pos_sub.get().lon);
	const float loiter_altitude = math::min(_land_approach.height_m, _rtl_alt);

	const bool is_close_to_destination = destination_dist < _param_rtl_min_dist.get();

	float altitude_acceptance_radius = static_cast<float>(NAN);

	switch (_rtl_state) {
	case RTLState::CLIMBING: {
			// By default climb on a loiter centered at the current position.
			// 翻译：默认情况下，爬升时使用以当前位置为中心的盘旋
			double loiter_center_lat = _global_pos_sub.get().lat;
			double loiter_center_lon = _global_pos_sub.get().lon;
			float loiter_radius = _navigator->get_default_loiter_rad();

			// If the vehicle was already established on a loiter when RTL was engaged (e.g. from Hold),
			// keep that loiter's center and radius while climbing instead of re-centering the circle on
			// the current position.
			// 翻译：如果在触发 RTL(返航)时飞行器已处于悬停(Loiter)状态(例如从“保持”模式进入)，则在爬升过程中应保持该悬停模式的中心点和半径，而不是将圆心重新设定为当前位置
			if (_navigator->is_established_on_loiter(_setpoint_on_activation)) {
				loiter_center_lat = _setpoint_on_activation.lat;
				loiter_center_lon = _setpoint_on_activation.lon;
				// loiter_radius sign encodes direction (negative == counter-clockwise).
				// 翻译: loiter_radius 符号表示方向(负值表示逆时针方向)
				loiter_radius = _setpoint_on_activation.loiter_direction_counter_clockwise ?
						-_setpoint_on_activation.loiter_radius : _setpoint_on_activation.loiter_radius;
			}

			PositionYawSetpoint pos_yaw_sp {
				.lat = loiter_center_lat,
				.lon = loiter_center_lon,
				.alt = _rtl_alt,
				.yaw = _param_wv_en.get() ? NAN : _navigator->get_local_position()->heading,
			};
			setLoiterToAltMissionItem(_mission_item, pos_yaw_sp, loiter_radius);

			break;
		}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	case RTLState::AVOID_GEOFENCE: {

			GeofenceAvoidancePlanner &planner = _navigator->get_geofence_avoidance_planner();
			matrix::Vector2d point = planner.getCurrentWaypoint();
			const matrix::Vector2d next_point = planner.getNextWaypoint();
			const bool is_first_waypoint = 0 == planner.getPathCursor();
			planner.advanceWaypoint();

			if (!point.isAllFinite()) {
				// Should never happen -- AVOID_GEOFENCE is only entered while hasMore() is true.
				// Fall back to RTLing in a straight line
				// 翻译：这种情况不应该发生——只有当 hasMore() 为真时才会进入 AVOID_GEOFENCE 状态。否则将回退到直线返航
				point(0) = _land_approach.lat;
				point(1) = _land_approach.lon;
			}

			float yaw = NAN;

			// 当VTOL在多旋翼模式下时，侧风容易把机翼“掀”起来导致翻机，所以需要开启风向标模式，让其机头保持朝向迎风面
			if (!_param_wv_en.get() && point.isAllFinite()) {
				yaw = get_bearing_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
								   point(0), point(1));
			}

			PositionYawSetpoint pos_yaw_sp {
				.lat = point(0),
				.lon = point(1),
				.alt = _rtl_alt,
				.yaw = yaw,
			};

			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);

			// Line following only between points on the path, not when flying to the first point
			// 翻译：仅在路径上的点之间进行沿线飞行，而不是在飞向第一个点时进行沿线飞行
			if (is_first_waypoint) {
				_navigator->reset_position_setpoint(pos_sp_triplet->previous);

			} else {
				pos_sp_triplet->previous = pos_sp_triplet->current;
			}

			pos_sp_triplet->next.valid = true;
			pos_sp_triplet->next.alt = _rtl_alt;
			pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

			if (next_point.isAllFinite()) {
				pos_sp_triplet->next.lat = next_point(0);
				pos_sp_triplet->next.lon = next_point(1);

			} else {
				// Last avoidance waypoint -- land approach is next.
				pos_sp_triplet->next.lat = _land_approach.lat;
				pos_sp_triplet->next.lon = _land_approach.lon;
			}

			break;
		}

#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	case RTLState::MOVE_TO_LOITER: {

			// Shift waypoint to enable line following with respect to last waypoint
			// 翻译: 移动航点以启用相对于上一个航点的直线跟踪
			pos_sp_triplet->previous = pos_sp_triplet->current;

			PositionYawSetpoint pos_yaw_sp {
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = _rtl_alt,
			};

			// For FW flight:set to LOITER_TIME (with 0s loiter time), such that the loiter (orbit) status
			// can be displayed on groundstation and the WP is accepted once within loiter radius
			// 翻译：对于FW飞行：设置为LOITER_TIME(滞空时间为0秒)，以便在地面站显示滞空(轨道)状态，并且一旦进入滞空半径，WP就会被接受
			if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				pos_yaw_sp.yaw = NAN;
				setLoiterHoldMissionItem(_mission_item, pos_yaw_sp, 0.f, _land_approach.loiter_radius_m);

			} else {
				// already set final yaw if close to destination and weather vane is disabled
				// 翻译：如果接近目的地且风向标已禁用，则已设置最终偏航角。
				pos_yaw_sp.yaw = (is_close_to_destination && !_param_wv_en.get()) ? _destination.yaw : NAN;
				setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);
			}

			break;
		}

	case RTLState::LOITER_DOWN: {
			PositionYawSetpoint pos_yaw_sp{
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = loiter_altitude,
				.yaw = !_param_wv_en.get() ? _destination.yaw : NAN, // set final yaw if weather vane is disabled
			};

			setLoiterToAltMissionItem(_mission_item, pos_yaw_sp, _land_approach.loiter_radius_m);

			pos_sp_triplet->next.valid = true;
			pos_sp_triplet->next.lat = _destination.lat;
			pos_sp_triplet->next.lon = _destination.lon;
			pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_LAND;

			if (_force_heading) {
				_mission_item.force_heading = true;
			}

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			break;
		}

	case RTLState::LOITER_HOLD: {
			PositionYawSetpoint pos_yaw_sp {
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = loiter_altitude,
				.yaw = !_param_wv_en.get() ? _destination.yaw : NAN, // set final yaw if weather vane is disabled
			};

			setLoiterHoldMissionItem(_mission_item, pos_yaw_sp, _param_rtl_land_delay.get(), _land_approach.loiter_radius_m);

			if (_param_rtl_land_delay.get() < -FLT_EPSILON) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering\t");
				events::send(events::ID("rtl_completed_loiter"), events::Log::Info, "RTL: completed, loitering");

			} else {
				/* Set the altitude tracking to best effort but not strictly enforce it */
				altitude_acceptance_radius = FLT_MAX;

				if (_force_heading) {
					_mission_item.force_heading = true;
				}
			}

			break;
		}

	case RTLState::MOVE_TO_LAND: {

			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.alt = loiter_altitude;
			pos_yaw_sp.yaw = NAN;

			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);

			// Prepare for transition
			// 翻译：准备过渡
			_mission_item.vtol_back_transition = true;
			_mission_item.force_heading = false;

			// set previous item location to loiter location such that vehicle tracks line between loiter
			// location and land location after exiting the loiter circle
			// 翻译：将先前物品位置设置为盘旋位置，以便载具在离开盘旋圈后沿盘旋位置和着陆位置之间的连线行驶
			pos_sp_triplet->previous.lat = _land_approach.lat;
			pos_sp_triplet->previous.lon = _land_approach.lon;
			pos_sp_triplet->previous.alt = get_absolute_altitude_for_item(_mission_item);
			pos_sp_triplet->previous.valid = true;

			break;
		}

	case RTLState::TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

			break;
		}

	case RTLState::MOVE_TO_LAND_HOVER: {
			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.alt = loiter_altitude;
			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled

			altitude_acceptance_radius = FLT_MAX;
			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);
			_navigator->reset_position_setpoint(pos_sp_triplet->previous);

			break;
		}

	case RTLState::LAND: {
			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled
			setLandMissionItem(_mission_item, pos_yaw_sp);

			_mission_item.land_precision = _param_rtl_pld_md.get();

			if (_mission_item.land_precision > 0) {
				startPrecLand(_mission_item.land_precision);
			}

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination\t");
			events::send(events::ID("rtl_land_at_destination"), events::Log::Info, "RTL: land at destination");
			break;
		}

	case RTLState::IDLE: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!mission_item_contains_position(_mission_item)) {
		issue_command(_mission_item);

	} else {
		// Convert mission item to current position setpoint and make it valid.
		// 翻译：将任务项转换为当前位置设定点并使其有效
		if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
			pos_sp_triplet->current.alt_acceptance_radius = altitude_acceptance_radius;
			_navigator->set_position_setpoint_triplet_updated();
		}
	}

	publish_rtl_direct_navigator_mission_item(); // for logging
}

RtlDirect::RTLState RtlDirect::getActivationState()
{
	_land_detected_sub.update();

	RTLState activation_state;

	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		// Skip to LAND state if we are a rover
		activation_state = RTLState::LAND;

	} else if (_land_detected_sub.get().landed) {
		// For safety reasons don't go into RTL if landed.
		activation_state = RTLState::IDLE;

	} else if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {
		activation_state = RTLState::CLIMBING;

	} else if (geofenceAvoidanceActive()) {
		activation_state = RTLState::AVOID_GEOFENCE;

	} else {
		activation_state = RTLState::MOVE_TO_LOITER;
	}

	return activation_state;
}

/**
 * @brief 计算RTL时间估计
 */
rtl_time_estimate_s RtlDirect::calc_rtl_time_estimate()
{
	_global_pos_sub.update();
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	RTLState start_state_for_estimate;

	if (isActive()) {
		start_state_for_estimate = _rtl_state;

	} else {
		start_state_for_estimate = getActivationState();
	}

	// Calculate RTL time estimate only when there is a valid destination
	// TODO: Also check if vehicle position is valid
	// 翻译：计算RTL时间估计时，只有当目的地有效时才进行计算
	//      TODO：当车辆位置有效时，也进行检查
	if (PX4_ISFINITE(_destination.lat) && PX4_ISFINITE(_destination.lon) && PX4_ISFINITE(_destination.alt)) {

		loiter_point_s land_approach = sanitizeLandApproach(_land_approach);

		const float loiter_altitude = min(land_approach.height_m, _rtl_alt);

		// If geofence avoidance, this is set to the last waypoint.
		// Otherwise, we move directly from vehicle position to loiter.
		// 翻译：如果启用了地理围栏避开，则此值设置为上一个航点
		// 	否则，我们将直接从车辆位置转移到徘徊位置
		matrix::Vector2d pos_before_loiter{_global_pos_sub.get().lat, _global_pos_sub.get().lon};

		// Sum up time estimate for various segments of the landing procedure
		// 翻译：汇总着陆过程各阶段的预计用时
		switch (start_state_for_estimate) {
		case RTLState::CLIMBING: {
				// Climb segment is only relevant if the drone is below return altitude
				// 翻译：爬升段仅在无人机低于返回高度时才相关
				if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {
					_rtl_time_estimator.addVertDistance(_rtl_alt - _global_pos_sub.get().alt);
				}
			}


		// FALLTHROUGH
		case RTLState::AVOID_GEOFENCE: {
#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
				pos_before_loiter = add_geofence_avoidance_path_distance(
							    _rtl_time_estimator,
							    _navigator->get_geofence_avoidance_planner(),
							    matrix::Vector2d(_global_pos_sub.get().lat, _global_pos_sub.get().lon)
						    );
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
			}

		// FALLTHROUGH
		case RTLState::MOVE_TO_LOITER: {
				matrix::Vector2f direction{};
				get_vector_to_next_waypoint(pos_before_loiter(0), pos_before_loiter(1),
							    land_approach.lat, land_approach.lon, &direction(0), &direction(1));

				float move_to_land_dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, land_approach.lat, land_approach.lon)};

				if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					move_to_land_dist = max(0.f, move_to_land_dist - land_approach.loiter_radius_m);
				}

				_rtl_time_estimator.addDistance(move_to_land_dist, direction, 0.f);
			}

		// FALLTHROUGH
		case RTLState::LOITER_DOWN: {
				// when descending, the target altitude is stored in the current mission item
				// 翻译：当下降时，目标高度存储在当前航点中
				float initial_altitude = 0.f;

				if (start_state_for_estimate == RTLState::LOITER_DOWN) {
					// Take current vehicle altitude as the starting point for calculation
					// 翻译：以当前飞行器高度作为计算的起始点
					initial_altitude = _global_pos_sub.get().alt;  // TODO: Check if this is in the right frame

				} else {
					// Take the return altitude as the starting point for the calculation
					// 翻译：以返回高度作为计算的起始点。
					initial_altitude = _rtl_alt; // CLIMB and RETURN
				}

				_rtl_time_estimator.addVertDistance(loiter_altitude - initial_altitude);
			}

		// FALLTHROUGH
		case RTLState::LOITER_HOLD:
			// Add land delay (the short pause for deploying landing gear)
			// 翻译：添加降落延迟(降落前的短暂暂停)
			_rtl_time_estimator.addWait(_param_rtl_land_delay.get());

			if (_param_rtl_land_delay.get() < -FLT_EPSILON) { // Set to loiter infinitely and not land. Stop calculation here
				break;
			}


		// FALLTHROUGH
		case RTLState::MOVE_TO_LAND:
		case RTLState::TRANSITION_TO_MC:
		case RTLState::MOVE_TO_LAND_HOVER: {
				// Add cruise segment to destination
				// 翻译：将邮轮航段添加到目的地
				float move_to_land_dist{0.f};
				matrix::Vector2f direction{};

				if (start_state_for_estimate >= RTLState::MOVE_TO_LAND) {
					move_to_land_dist = get_distance_to_next_waypoint(
								    _global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat, _destination.lon);
					get_vector_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, _destination.lat, _destination.lon,
								    &direction(0), &direction(1));

				} else {
					move_to_land_dist = get_distance_to_next_waypoint(
								    land_approach.lat, land_approach.lon, _destination.lat, _destination.lon);
					get_vector_to_next_waypoint(land_approach.lat, land_approach.lon, _destination.lat, _destination.lon, &direction(0),
								    &direction(1));
				}

				_rtl_time_estimator.addDistance(move_to_land_dist, direction, 0.f);
			}

		// FALLTHROUGH
		case RTLState::LAND: {
				float initial_altitude;

				// Add land segment (second landing phase) which comes after LOITER
				// 翻译：添加着陆段(第二个着陆阶段)，在LOITER之后
				if (start_state_for_estimate == RTLState::LAND) {
					// If we are in this phase, use the current vehicle altitude  instead
					// of the altitude paramteter to get a continous time estimate
					// 翻译：如果我们在这个阶段，使用当前的车辆高度，而不是高度参数来获得连续的时间估计
					initial_altitude = _global_pos_sub.get().alt;


				} else {
					// If this phase is not active yet, simply use the loiter altitude,
					// which is where the LAND phase will start
					// 翻译：如果这个阶段还没有激活，简单地使用着陆高度，这是着陆阶段将开始的地方
					initial_altitude = loiter_altitude;
				}

				if (_vehicle_status_sub.get().is_vtol) {
					_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				}

				_rtl_time_estimator.addVertDistance(_destination.alt - initial_altitude);
			}

			break;

		case RTLState::IDLE:
			// Remaining time is 0
			break;
		}
	}

	return _rtl_time_estimator.getEstimate();
}

void RtlDirect::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		// 翻译：如果任何参数更新，调用updateParams()检查是否需要更新此类属性(并执行更新)
		updateParams();
	}
}

loiter_point_s RtlDirect::sanitizeLandApproach(const loiter_point_s &land_approach) const
{
	loiter_point_s sanitized_land_approach{land_approach};

	if (!PX4_ISFINITE(land_approach.lat) || !PX4_ISFINITE(land_approach.lon)) {
		sanitized_land_approach.lat = _destination.lat;
		sanitized_land_approach.lon = _destination.lon;
	}

	if (!PX4_ISFINITE(land_approach.height_m)) {
		sanitized_land_approach.height_m = _destination.alt + _param_rtl_descend_alt.get();
	}

	if (!PX4_ISFINITE(land_approach.loiter_radius_m) || fabsf(land_approach.loiter_radius_m) <= FLT_EPSILON) {
		sanitized_land_approach.loiter_radius_m = _param_rtl_loiter_rad.get();
	}

	return sanitized_land_approach;
}

void RtlDirect::publish_rtl_direct_navigator_mission_item()
{
	navigator_mission_item_s navigator_mission_item{};

	navigator_mission_item.sequence_current = static_cast<uint16_t>(_rtl_state);
	navigator_mission_item.nav_cmd = _mission_item.nav_cmd;
	navigator_mission_item.latitude = _mission_item.lat;
	navigator_mission_item.longitude = _mission_item.lon;
	navigator_mission_item.altitude = _mission_item.altitude;

	navigator_mission_item.time_inside = get_time_inside(_mission_item);
	navigator_mission_item.acceptance_radius = _mission_item.acceptance_radius;
	navigator_mission_item.loiter_radius = _mission_item.loiter_radius;
	navigator_mission_item.yaw = _mission_item.yaw;

	navigator_mission_item.frame = _mission_item.frame;
	navigator_mission_item.origin = _mission_item.origin;

	navigator_mission_item.loiter_exit_xtrack = _mission_item.loiter_exit_xtrack;
	navigator_mission_item.force_heading = _mission_item.force_heading;
	navigator_mission_item.altitude_is_relative = _mission_item.altitude_is_relative;
	navigator_mission_item.autocontinue = _mission_item.autocontinue;
	navigator_mission_item.vtol_back_transition = _mission_item.vtol_back_transition;

	navigator_mission_item.timestamp = hrt_absolute_time();

	_navigator_mission_item_pub.publish(navigator_mission_item);
}
