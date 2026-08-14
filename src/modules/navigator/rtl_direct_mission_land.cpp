/***************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file rtl_direct_mission_land.cpp
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl_direct_mission_land.h"
#include "mission_item_utils.h"
#include "navigator.h"
#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
#include "rtl_geofence_avoidance_helper.h"
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

#include <drivers/drv_hrt.h>

static constexpr int32_t DEFAULT_DIRECT_MISSION_LAND_CACHE_SIZE = 5;

RtlDirectMissionLand::RtlDirectMissionLand(Navigator *navigator, const mission_s &mission) :
	RtlBase(navigator, DEFAULT_DIRECT_MISSION_LAND_CACHE_SIZE)
{
	_mission = mission;
}

void
RtlDirectMissionLand::updateDatamanCache()
{
	int32_t start_index;

	start_index = math::min(_mission.land_start_index, static_cast<int32_t>(_mission.count));

	if ((start_index >= 0) && (_mission.count > 0) && hasMissionLandStart() && (start_index != _load_mission_index)) {

		int32_t end_index = static_cast<int32_t>(_mission.count);

		// Check that we load all data into the cache
		// 翻译：检查我们是否将所有数据加载到缓存中
		if (end_index - start_index > _dataman_cache_size_signed) {
			_dataman_cache.invalidate();
			_dataman_cache_size_signed = end_index - start_index;
			_dataman_cache.resize(_dataman_cache_size_signed);
		}

		for (int32_t index = start_index; index != end_index; index += math::signNoZero(_dataman_cache_size_signed)) {

			_dataman_cache.load(static_cast<dm_item_t>(_mission.mission_dataman_id), index);
		}

		_load_mission_index = start_index;
	}

	_dataman_cache.update();
}

void RtlDirectMissionLand::on_inactive()
{
	MissionBase::on_inactive();

	updateDatamanCache();
}

void RtlDirectMissionLand::on_activation()
{
	_land_detected_sub.update();
	_global_pos_sub.update();

	_needs_climbing = false;

	if (hasMissionLandStart()) {
		_is_current_planned_mission_item_valid = (goToItem(_mission.land_start_index, MissionTraversalType::IgnoreDoJump) == PX4_OK);

		_needs_climbing = checkNeedsToClimb();

	} else {
		_is_current_planned_mission_item_valid = false;
	}


	if (_land_detected_sub.get().landed) {
		// already landed, no need to do anything, invalidad the position mission item.
		// 翻译：已经着陆，无需执行任何操作，无效化位置任务项目。
		_is_current_planned_mission_item_valid = false;
	}

	// Snapshot the setpoint the previous mode left before MissionBase::on_activation() resets the
	// triplet, so the climb can continue an already-established loiter (used in setActiveMissionItems()).
	_setpoint_on_activation = _navigator->get_position_setpoint_triplet()->current;

	MissionBase::on_activation();
}

bool RtlDirectMissionLand::setNextMissionItem()
{
#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	// Stay on the same mission item until the geofence-avoidance path is fully consumed.
	if (_navigator->get_geofence_avoidance_planner().hasMore()) {
		return true;
	}

#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
	return (goToNextPositionItem() == PX4_OK);
}

/**
 * @brief 设置当前任务项目
 */
void RtlDirectMissionLand::setActiveMissionItems()
{
	WorkItemType new_work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	const position_setpoint_s current_setpoint_copy = pos_sp_triplet->current;

	// Climb to altitude
	// 翻译：爬升到高度
	if (_needs_climbing && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// TODO: check if we also should use NAV_CMD_LOITER_TO_ALT for rotary wing
		// 翻译：检查是否也应该对旋翼机使用 NAV_CMD_LOITER_TO_ALT
		if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

		} else {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
		}

		// By default climb centered on the current position with the default loiter radius.
		// 翻译：默认情况下，爬升以当前位置为中心，并具有默认的盘旋半径
		_mission_item.lat = _global_pos_sub.get().lat;
		_mission_item.lon = _global_pos_sub.get().lon;
		_mission_item.loiter_radius = _navigator->get_default_loiter_rad();

		// If the vehicle was already established on a loiter when RTL was engaged (e.g. from Hold),
		// keep that loiter's center and radius while climbing instead of re-centering the circle on
		// the current position. The setpoint was snapshotted on activation before the triplet reset.
		// 翻译：如果在启用 RTL 功能时(例如从保持模式切换回来)，飞行器已处于悬停状态，则爬升过程中保持该悬停状态的中心和半径不变，
		// 	而不是将圆心重新定位到当前位置。设定点在三元组重置之前，于激活时被快照
		if (_setpoint_on_activation.valid
		    && _setpoint_on_activation.type == position_setpoint_s::SETPOINT_TYPE_LOITER
		    && _setpoint_on_activation.loiter_pattern == position_setpoint_s::LOITER_TYPE_ORBIT) {
			const float dist_to_center = get_distance_to_next_waypoint(
							     _setpoint_on_activation.lat, _setpoint_on_activation.lon,
							     _global_pos_sub.get().lat, _global_pos_sub.get().lon);

			if (dist_to_center <= (_navigator->get_acceptance_radius() + fabsf(_setpoint_on_activation.loiter_radius))) {
				_mission_item.lat = _setpoint_on_activation.lat;
				_mission_item.lon = _setpoint_on_activation.lon;
				// loiter_radius sign encodes direction (negative == counter-clockwise).
				// 翻译：loiter_radius 符号表示方向(负号表示逆时针方向)
				_mission_item.loiter_radius = _setpoint_on_activation.loiter_direction_counter_clockwise ?
							      -_setpoint_on_activation.loiter_radius : _setpoint_on_activation.loiter_radius;
			}
		}

		_mission_item.altitude = _rtl_alt;
		_mission_item.altitude_is_relative = false;

		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL Mission land: climb to %d m\t",
				 (int)ceilf(_rtl_alt));
		events::send<int32_t>(events::ID("rtl_mission_land_climb"), events::Log::Info,
				      "RTL Mission Land: climb to {1m_v}",
				      (int32_t)ceilf(_rtl_alt));

		_needs_climbing = false;
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_CLIMB;

	} else if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
		   _vehicle_status_sub.get().is_vtol &&
		   !_land_detected_sub.get().landed && _work_item_type == WorkItemType::WORK_ITEM_TYPE_DEFAULT) {
		// 该分支只在 VTOL 处于旋转翼状态（尚未进入固定翼）且已离地、RTL 刚被激活时进入：主动触发向固定翼过渡，避免 VTOL 以悬停姿态直接执行 RTL 的地形回航/下降逻辑。
		// Transition to fixed wing if necessary.
		// 翻译：如果必要，转换到固定翼
		set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
		_mission_item.yaw = _navigator->get_local_position()->heading;

		// keep current setpoints (FW position controller generates wp to track during transition)
		// 翻译：保持当前设定点(FW位置控制器在转换期间生成wp来跟踪)
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		new_work_item_type = WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF;

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	// 然后路径优先跟着地理围栏绕飞点走（队列取完为止）
	} else if (_navigator->get_geofence_avoidance_planner().hasMore()) {

		GeofenceAvoidancePlanner &planner = _navigator->get_geofence_avoidance_planner();
		matrix::Vector2d point = planner.getCurrentWaypoint();
		const matrix::Vector2d next_point = planner.getNextWaypoint();
		const bool is_first_waypoint = 0 == planner.getPathCursor();
		planner.advanceWaypoint();

		if (!point.isAllFinite()) {
			// Should never happen -- the geofence branch is only entered while hasMore() is true.
			// Fall back to flying straight to the destination, as rtl_direct does.
			// 翻译：这种情况不应该发生——只有当 hasMore() 为真时才会进入地理围栏分支，回退到像 rtl_direct 那样直接飞往目的地
			const matrix::Vector2d destination = getRtlPlannerDestination();

			if (destination.isAllFinite()) {
				point = destination;

			} else {
				point(0) = _global_pos_sub.get().lat;
				point(1) = _global_pos_sub.get().lon;
			}
		}

		// Line following only between points on the path, not when flying to the first point
		// 翻译：仅在路径上的点之间进行沿线飞行，而不是在飞往第一个点时进行沿线飞行
		if (is_first_waypoint) {
			_navigator->reset_position_setpoint(pos_sp_triplet->previous);

		} else {
			pos_sp_triplet->previous = current_setpoint_copy;
		}

		_mission_item.lat = point(0);
		_mission_item.lon = point(1);
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.altitude = _rtl_alt;
		_mission_item.altitude_is_relative = false;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;
		_mission_item.loiter_radius = _navigator->get_default_loiter_rad();

		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

		// If next point does not exist, we have NaN and inalid next setpoint
		// 翻译：如果下一个点不存在，则返回 NaN 值，并且下一个设定点无效
		pos_sp_triplet->next.valid = next_point.isAllFinite();
		pos_sp_triplet->next.alt = _rtl_alt;
		pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->next.lat = next_point(0);
		pos_sp_triplet->next.lon = next_point(1);

#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

	// 最后落到普通回航航点
	} else if (mission_item_contains_position(_mission_item)) {

		static constexpr size_t max_num_next_items{1u};
		int32_t next_mission_items_index[max_num_next_items];
		size_t num_found_items = 0;
		getNextPositionItems(_mission.current_seq + 1, next_mission_items_index, num_found_items, max_num_next_items);

		mission_item_s next_mission_items[max_num_next_items];
		const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_mission.mission_dataman_id);

		for (size_t i = 0U; i < num_found_items; i++) {
			mission_item_s next_mission_item;
			bool success = _dataman_cache.loadWait(mission_dataman_id, next_mission_items_index[i],
							       reinterpret_cast<uint8_t *>(&next_mission_item), sizeof(next_mission_item), MAX_DATAMAN_LOAD_WAIT);

			if (success) {
				next_mission_items[i] = next_mission_item;

			} else {
				num_found_items = i;
				break;
			}
		}

		if (_mission_item.nav_cmd == NAV_CMD_LAND ||
		    _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
			handleLanding(new_work_item_type, next_mission_items, num_found_items);

		} else {
			// convert mission item to a simple waypoint, keep loiter to alt
			// 翻译：将任务项转换为简单的航点，保持高度保持
			if (_mission_item.nav_cmd != NAV_CMD_LOITER_TO_ALT) {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			}

			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0.0f;
		}

		if (num_found_items > 0) {
			mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->next);
		}

		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

		// Only set the previous position item if the current one really changed
		// 翻译：只有当当前位置点真的改变时才设置前一个位置点
		if ((_work_item_type != WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND) &&
		    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
			pos_sp_triplet->previous = current_setpoint_copy;
		}

		// prevent lateral guidance from loitering at a waypoint as part of a mission landing if the altitude
		// is not achieved.
		// 翻译：防止在任务着陆时在航点上进行横向引导，如果高度未达到
		const bool fw_on_mission_landing = _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
						   && isLanding() &&
						   _mission_item.nav_cmd == NAV_CMD_WAYPOINT;
		const bool mc_landing_after_transition = _vehicle_status_sub.get().vehicle_type ==
				vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && _vehicle_status_sub.get().is_vtol &&
				new_work_item_type == WorkItemType::WORK_ITEM_TYPE_MOVE_TO_LAND;

		if (fw_on_mission_landing || mc_landing_after_transition) {
			pos_sp_triplet->current.alt_acceptance_radius = FLT_MAX;
		}
	}

	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	reset_mission_item_reached();

	if (_mission_type == MissionType::MISSION_TYPE_MISSION) {
		set_mission_result();
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

rtl_time_estimate_s RtlDirectMissionLand::calc_rtl_time_estimate()
{
	_rtl_time_estimator.update();
	_rtl_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_rtl_time_estimator.reset();

	if (_mission.count > 0 && hasMissionLandStart()) {
		int32_t start_item_index{-1};
		bool is_in_climbing_submode{false};

		if (isActive()) {
			start_item_index = math::max(_mission.current_seq, _mission.land_start_index);
			is_in_climbing_submode = _work_item_type == WorkItemType::WORK_ITEM_TYPE_CLIMB;

		} else {
			start_item_index = _mission.land_start_index;
			is_in_climbing_submode = checkNeedsToClimb();
		}


#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
		matrix::Vector2d hor_position_at_calculation_point = add_geofence_avoidance_path_distance(
					_rtl_time_estimator,
					_navigator->get_geofence_avoidance_planner(),
					matrix::Vector2d(_global_pos_sub.get().lat, _global_pos_sub.get().lon)
				);
#else
		matrix::Vector2d hor_position_at_calculation_point {_global_pos_sub.get().lat, _global_pos_sub.get().lon};
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

		if (start_item_index >= 0 && start_item_index < static_cast<int32_t>(_mission.count)) {
			float altitude_at_calculation_point;

			if (is_in_climbing_submode) {
				if (_enforce_rtl_alt) {
					_rtl_time_estimator.addVertDistance(_rtl_alt - _global_pos_sub.get().alt);
					altitude_at_calculation_point = _rtl_alt;

				} else {
					if (_global_pos_sub.get().alt < _rtl_alt) {
						_rtl_time_estimator.addVertDistance(_rtl_alt - _global_pos_sub.get().alt);
					}

					altitude_at_calculation_point = math::max(_rtl_alt, _global_pos_sub.get().alt);
				}

			} else {
				altitude_at_calculation_point = _global_pos_sub.get().alt;
			}

			while (start_item_index < _mission.count && start_item_index >= 0) {
				int32_t next_mission_item_index;
				size_t num_found_items{0U};
				getNextPositionItems(start_item_index, &next_mission_item_index, num_found_items, 1U);

				if (num_found_items > 0U) {
					mission_item_s next_position_mission_item;
					const dm_item_t dataman_id = static_cast<dm_item_t>(_mission.mission_dataman_id);
					bool success = _dataman_cache.loadWait(dataman_id, next_mission_item_index,
									       reinterpret_cast<uint8_t *>(&next_position_mission_item), sizeof(next_position_mission_item), MAX_DATAMAN_LOAD_WAIT);

					if (!success) {
						// Could not load the mission item, mark time estimate as invalid.
						// 翻译：无法加载航点，将时间估计标记为无效
						_rtl_time_estimator.reset();
						break;
					}

					switch (next_position_mission_item.nav_cmd) {
					case NAV_CMD_LOITER_UNLIMITED: {
							_rtl_time_estimator.reset();
							break;
						}

					case NAV_CMD_LOITER_TIME_LIMIT: {
							// Go to loiter
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									 hor_position_at_calculation_point(1), next_position_mission_item.lat,
									 next_position_mission_item.lon);

							if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
								hor_dist = math::max(0.f, hor_dist - next_position_mission_item.loiter_radius);
							}

							_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

							// add time
							_rtl_time_estimator.addWait(next_position_mission_item.time_inside);
							break;
						}

					case NAV_CMD_LOITER_TO_ALT: {
							// Go to point horizontally
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									 hor_position_at_calculation_point(1), next_position_mission_item.lat,
									 next_position_mission_item.lon);

							if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
								hor_dist = math::max(0.f, hor_dist - next_position_mission_item.loiter_radius);
							}

							_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

							// Add the vertical loiter
							_rtl_time_estimator.addVertDistance(get_absolute_altitude_for_item(next_position_mission_item) -
											    altitude_at_calculation_point);

							break;
						}

					case NAV_CMD_LAND: // Fallthrough
					case NAV_CMD_VTOL_LAND: {

							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							const float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									       hor_position_at_calculation_point(1), next_position_mission_item.lat, next_position_mission_item.lon);

							// For fixed wing, add diagonal line
							// 翻译：对于固定翼，添加对角线
							if ((_vehicle_status_sub.get().vehicle_type != vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
							    && (!_vehicle_status_sub.get().is_vtol)) {

								_rtl_time_estimator.addDistance(hor_dist, direction,
												get_absolute_altitude_for_item(next_position_mission_item) - altitude_at_calculation_point);

							} else {
								// For VTOL, Rotary, go there horizontally first, then land
								// 翻译：对于垂直起降飞行器和旋翼机，先水平飞行，然后再降落
								_rtl_time_estimator.addDistance(hor_dist, direction, 0.f);

								if (_vehicle_status_sub.get().is_vtol) {
									_rtl_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
								}

								_rtl_time_estimator.addVertDistance(get_absolute_altitude_for_item(next_position_mission_item) -
												    altitude_at_calculation_point);
							}

							break;
						}

					default: {
							// Default assume can go to the location directly
							// 翻译：默认假设可以直接到达该位置
							matrix::Vector2f direction{};
							get_vector_to_next_waypoint(hor_position_at_calculation_point(0), hor_position_at_calculation_point(1),
										    next_position_mission_item.lat, next_position_mission_item.lon, &direction(0), &direction(1));

							const float hor_dist = get_distance_to_next_waypoint(hor_position_at_calculation_point(0),
									       hor_position_at_calculation_point(1), next_position_mission_item.lat, next_position_mission_item.lon);

							_rtl_time_estimator.addDistance(hor_dist, direction,
											get_absolute_altitude_for_item(next_position_mission_item) - altitude_at_calculation_point);
							break;
						}
					}

					start_item_index = next_mission_item_index + 1;
					hor_position_at_calculation_point(0) = next_position_mission_item.lat;
					hor_position_at_calculation_point(1) = next_position_mission_item.lon;
					altitude_at_calculation_point = get_absolute_altitude_for_item(next_position_mission_item);


				} else {
					start_item_index = -1;
				}
			}
		}
	}

	return _rtl_time_estimator.getEstimate();
}

/**
 * @brief 检查是否需要爬升到指定的航点
 */
bool RtlDirectMissionLand::checkNeedsToClimb()
{
	bool needs_climbing{false};

	if ((_global_pos_sub.get().alt < _rtl_alt) || _enforce_rtl_alt) {

		// If lower than return altitude, climb up first.
		// If enforce_rtl_alt is true then forcing altitude change even if above.
 		// 翻译：如果低于返回高度，则先爬升
		// 	如果强制返回高度，则强制高度变化，即使在上面
		needs_climbing = true;

	}

	return needs_climbing;
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
matrix::Vector2d RtlDirectMissionLand::getRtlPlannerDestination()
{
	matrix::Vector2d position((double)NAN, (double)NAN);

	if (!hasMissionLandStart()) {
		return position;
	}

	int32_t next_mission_item_index{-1};
	size_t num_found_items{0U};
	getNextPositionItems(_mission.land_start_index + 1, &next_mission_item_index, num_found_items, 1U);

	if (num_found_items == 0U) {
		return position;
	}

	mission_item_s next_position_mission_item;
	const dm_item_t mission_dataman_id = static_cast<dm_item_t>(_mission.mission_dataman_id);
	const bool success = _dataman_cache.loadWait(mission_dataman_id, next_mission_item_index,
			     reinterpret_cast<uint8_t *>(&next_position_mission_item),
			     sizeof(next_position_mission_item), MAX_DATAMAN_LOAD_WAIT);

	if (!success) {
		return position;
	}

	position(0) = next_position_mission_item.lat;
	position(1) = next_position_mission_item.lon;

	return position;
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
