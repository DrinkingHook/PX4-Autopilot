/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "framework.h"
#define DEFINE_GET_PX4_CUSTOM_MODE
#include "../px4_custom_mode.h"

#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

using failsafe_action_t = events::px4::enums::failsafe_action_t;
using failsafe_cause_t = events::px4::enums::failsafe_cause_t;

using namespace time_literals;

FailsafeBase::FailsafeBase(ModuleParams *parent) : ModuleParams(parent)
{
	_current_start_delay = _param_com_fail_act_t.get() * 1_s;
}

uint8_t FailsafeBase::update(const hrt_abstime &time_us, const State &state, bool user_intended_mode_updated,
			     bool rc_sticks_takeover_request,
			     const failsafe_flags_s &status_flags)
{
	if (_last_update == 0) {
		_last_update = time_us;
	}

	if ((_last_armed && !state.armed) || (!_last_armed && state.armed)) { // Disarming or Arming
		removeActions(ClearCondition::OnDisarm);
		removeActions(ClearCondition::OnModeChangeOrDisarm);
		_user_takeover_active = false;
	}

	// 当上次的用户期望模式和现在的期望模式不同或 user_intended_mode_updated 本身就为True时user_intended_mode_updated为True
	user_intended_mode_updated |= _last_user_intended_mode != state.user_intended_mode;

	// 切换用户期望模式或者用户正在接管则移除action
	if (user_intended_mode_updated || _user_takeover_active) {
		removeActions(ClearCondition::OnModeChangeOrDisarm);
	}

	/**
	 * @brief
	 * @param _defer_failsafes 延迟标志位
	 * @param _failsafe_defer_started 延迟开始时间,如不为0则表示已经开始
	 * @param _defer_timeout 延迟时间
	 */
	// 判断故障保护延迟时间是否已经结束
	if (_defer_failsafes && _failsafe_defer_started != 0 && _defer_timeout > 0
	    && time_us > _failsafe_defer_started + _defer_timeout) {
		_defer_failsafes = false;
	}

	if (_failsafe_defer_started == 0) {
		updateDelay(time_us - _last_update);
	}

	// 检查状态和模式 这个是通过现在状态和故障标志添加故障保护action
	checkStateAndMode(time_us, state, status_flags);
	removeNonActivatedActions();
	clearDelayIfNeeded(state, status_flags);

	// 根据状态选择合适的action以应对故障
	SelectedActionState action_state{};
	getSelectedAction(state, status_flags, user_intended_mode_updated, rc_sticks_takeover_request, action_state);

	/**
	 * @brief 更新开始延迟的故障保护action
	 * @param action_state.delayed_action 此参数的值在 getSelectedAction()函数中赋值,用于存储需要延迟操作的action
	 */

	updateStartDelay(time_us - _last_update, action_state.delayed_action != Action::None);
	// 更新故障保护延迟状态
	updateFailsafeDeferState(time_us, action_state.failsafe_deferred);

	// Notify about escalation, or about any new subsumed condition as an informational warning
	// 翻译：通知升级情况，或将任何新出现的被包含条件作为提示性警告进行通报
	if (action_state.action > _selected_action) {
		notifyUser(state.user_intended_mode, action_state.action, action_state.delayed_action, action_state.cause);

	} else if (_pending_notification_cause != Cause::Count) {
		notifyUser(state.user_intended_mode, Action::Warn, Action::None, _pending_notification_cause);
	}

	_pending_notification_cause = Cause::Count;

	// 修改用户意图模式
	_last_user_intended_mode = modifyUserIntendedMode(_selected_action, action_state.action,
				   action_state.updated_user_intended_mode);
	_user_takeover_active = action_state.user_takeover;
	// 最终得到的故障保护action,Commander()函数调用selectedAction()以实现具体功能
	_selected_action = action_state.action;
	_last_update = time_us;
	_last_status_flags = status_flags;
	_last_armed = state.armed;
	return _last_user_intended_mode;
}

void FailsafeBase::updateFailsafeDeferState(const hrt_abstime &time_us, bool defer)
{
	if (defer) {
		if (_failsafe_defer_started == 0) {
			_failsafe_defer_started = time_us;
		}

	} else {
		_failsafe_defer_started = 0;
	}
}

void FailsafeBase::updateStartDelay(const hrt_abstime &dt, bool delay_active)
{
	// Ensure that even with a toggling state the delayed action is executed at some point.
	// This is done by increasing the delay slower than reducing it.
	// 翻译：确保即使在状态切换的情况下，延迟的操作最终也能执行。
	// 	这是通过增加延迟的速度比减少延迟的速度慢来实现的。
	if (delay_active) {
		if (dt < _current_start_delay) {
			_current_start_delay -= dt;

		} else {
			_current_start_delay = 0;
		}

	} else {
		_current_start_delay += dt / 4;
		hrt_abstime configured_delay = _param_com_fail_act_t.get() * 1_s;

		if (_current_start_delay > configured_delay) {
			_current_start_delay = configured_delay;
		}
	}
}

void FailsafeBase::updateParams()
{
	ModuleParams::updateParams();
	_current_start_delay = _param_com_fail_act_t.get() * 1_s;
}

void FailsafeBase::updateDelay(const hrt_abstime &elapsed_us)
{
	if (_current_delay < elapsed_us) {
		_current_delay = 0;

	} else {
		_current_delay -= elapsed_us;
	}
}

void FailsafeBase::removeActions(ClearCondition condition)
{
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && !cur_action.state_failure && cur_action.clear_condition == condition) {
			PX4_DEBUG("Caller %i: clear condition triggered, removing", cur_action.id);
			cur_action.setInvalid();
		}
	}
}

/**
 * @brief 通知用户 如果最新一次的故障action比上一次的更加严重便会执行
 *
 * @param user_intended_mode 用户期望的模式
 * @param action 故障保护动作
 * @param delayed_action 延迟的故障保护动作
 * @param cause 故障保护原因
 */
void FailsafeBase::notifyUser(uint8_t user_intended_mode, Action action, Action delayed_action, Cause cause)
{
	if (_on_notify_user_cb) {
		_on_notify_user_cb(_on_notify_user_arg);
	}

	int delay_s = (_current_delay + 500_ms) / 1_s;
	PX4_DEBUG("User notification: failsafe triggered (action=%i, delayed_action=%i, cause=%i, delay=%is)", (int)action,
		  (int)delayed_action, (int)cause, delay_s);

#ifdef EMSCRIPTEN_BUILD
	(void)_mavlink_log_pub;
#else

	px4_custom_mode custom_mode = get_px4_custom_mode(user_intended_mode);
	uint32_t mavlink_mode = custom_mode.data;

	static_assert((int)failsafe_cause_t::_max + 1 == (int)Cause::Count, "Enum needs to be extended");
	static_assert((int)failsafe_action_t::_max + 1 == (int)Action::Count, "Enum needs to be extended");
	failsafe_cause_t failsafe_cause = (failsafe_cause_t)cause;

	if (action == Action::Hold && delayed_action != Action::None) {
		failsafe_action_t failsafe_action = (failsafe_action_t)delayed_action;

		if (cause == Cause::Generic) {
			/* EVENT
			* @type append_health_and_arming_messages
			*/
			events::send<uint32_t, events::px4::enums::failsafe_action_t, uint16_t>(
				events::ID("commander_failsafe_enter_generic_hold"),
			{events::Log::Critical, events::LogInternal::Warning},
			"Failsafe activated: switching to {2} in {3} seconds", mavlink_mode, failsafe_action,
			(uint16_t) delay_s);

		} else {
			/* EVENT
			*/
			events::send<uint32_t, events::px4::enums::failsafe_action_t, uint16_t, events::px4::enums::failsafe_cause_t>(
				events::ID("commander_failsafe_enter_hold"),
			{events::Log::Critical, events::LogInternal::Warning},
			"{4}: switching to {2} in {3} seconds", mavlink_mode, failsafe_action,
			(uint16_t) delay_s, failsafe_cause);
		}

		mavlink_log_critical(&_mavlink_log_pub, "Failsafe activated: entering Hold for %i seconds\t", delay_s);

	} else { // no delay
		failsafe_action_t failsafe_action = (failsafe_action_t)action;

		if (cause == Cause::Generic) {
			if (action == Action::Warn) {
				/* EVENT
				* @description No action is triggered.
				* @type append_health_and_arming_messages
				*/
				events::send<uint32_t>(
					events::ID("commander_failsafe_enter_generic_warn"),
				{events::Log::Warning, events::LogInternal::Warning},
				"Failsafe warning:", mavlink_mode);

			} else if (action == Action::Descend || action == Action::FallbackAltCtrl || action == Action::FallbackStab) {
				/* EVENT
				* @description Failsafe actions that disengage the autopilot (remove position control)
				* @type append_health_and_arming_messages
				*/
				events::send<uint32_t, events::px4::enums::failsafe_action_t>(
					events::ID("commander_failsafe_enter_autopilot_disengaged"),
				{events::Log::Critical, events::LogInternal::Warning},
				"Failsafe activated: Autopilot disengaged, switching to {2}", mavlink_mode, failsafe_action);

			} else {
				/* EVENT
				* @type append_health_and_arming_messages
				*/
				events::send<uint32_t, events::px4::enums::failsafe_action_t>(
					events::ID("commander_failsafe_enter_generic"),
				{events::Log::Critical, events::LogInternal::Warning},
				"Failsafe activated: switching to {2}", mavlink_mode, failsafe_action);
			}

		} else {
			if (action == Action::Warn) {
				if (cause == Cause::BatteryLow) {
					events::send(events::ID("commander_failsafe_enter_low_bat"),
					{events::Log::Warning, events::LogInternal::Info},
					"Low battery level, return advised");

				} else if (cause == Cause::BatteryCritical) {
					events::send(events::ID("commander_failsafe_enter_crit_bat_warn"),
					{events::Log::Critical, events::LogInternal::Info},
					"Critical battery level, land now");

				} else if (cause == Cause::BatteryEmergency) {
					events::send(events::ID("commander_failsafe_enter_crit_low_bat_warn"), {events::Log::Emergency, events::LogInternal::Info},
						     "Emergency battery level, land immediately");

				} else if (cause == Cause::RemainingFlightTimeLow) {
					events::send(events::ID("commander_failsafe_enter_low_flight_time_warn"),
					{events::Log::Warning, events::LogInternal::Info},
					"Low remaining flight time, return advised");

				} else {
					/* EVENT
					* @description No action is triggered.
					*/
					events::send<uint32_t, events::px4::enums::failsafe_cause_t>(
						events::ID("commander_failsafe_enter_warn"),
					{events::Log::Warning, events::LogInternal::Warning},
					"Failsafe warning: {2}", mavlink_mode, failsafe_cause);

				}

			} else { // action != Warn
				/* EVENT
				*/
				events::send<uint32_t, events::px4::enums::failsafe_action_t, events::px4::enums::failsafe_cause_t>(
					events::ID("commander_failsafe_enter"),
				{events::Log::Critical, events::LogInternal::Warning},
				"{3}: switching to {2}", mavlink_mode, failsafe_action, failsafe_cause);
			}
		}

		if (action != Action::Warn) {
			mavlink_log_critical(&_mavlink_log_pub, "Failsafe activated\t");
		}
	}

#endif /* EMSCRIPTEN_BUILD */
}

/**
 * @brief Checks and manages failsafe actions based on current and previous state failures.
 *	  翻译：基于当前和先前状态失败来检查和管理故障安全动作。
 * This function evaluates whether a failsafe action should be added, updated, or removed from the
 * `_actions` array based on the current fault state (`cur_state_failure`) and the previous fault state
 * (`last_state_failure`). It handles fault conditions such as invalid sensor data or communication loss,
 * and applies the specified action (e.g., Warn, RTL, Land, Hold) from the provided options.
 *
 * @param caller_id 调用者的唯一标识，用于区分不同的故障保护触发源（如 GPS 失效、电池低电量、四轴降落伞触发等）。
 * @param last_state_failure 上一次循环中是否检测到故障状态（true 表示之前有故障）。
 * @param cur_state_failure 当前是否检测到故障状态（true 表示当前有故障）。
 * @param options 故障保护选项，包含动作（Action，如 Warn、RTL、Land、Hold）和清除条件（ClearCondition），通常由 Failsafe::fromQuadchuteActParam 等函数生成。
 *
 * @return bool，直接返回 cur_state_failure，表示当前是否处于故障状态。
 *
 * @note This function interacts with uORB topics such as `position_setpoint_triplet_s` and `trajectory_setpoint_s`
 *       to apply failsafe actions like RTL or Land. It uses `PX4_ISFINITE` or `isAllFinite` to validate data.
 * @warning Duplicate caller IDs may trigger a bug warning (`PX4_ERR`) if detected.
 * @see Failsafe::fromQuadchuteActParam
 * @see ActionOptions
 */
bool FailsafeBase::checkFailsafe(int caller_id, bool last_state_failure, bool cur_state_failure,
				 const ActionOptions &options)
{
	// 如果当前有故障
	if (cur_state_failure) {
		// Invalid state: find or add action
		// 翻译：无效状态：查找或添加操作
		int free_idx = -1;
		int found_idx = -1;

		for (int i = 0; i < max_num_actions; ++i) {
			// // 查找空闲的action槽
			// if (!_actions[i].valid()) {
			// 	free_idx = i;

			// } else if (_actions[i].id == caller_id) {
			// 	found_idx = i;
			// 	// 只有这个else if中break出去
			// 	break;
			// }

			// 寻找相匹配的 action
			if (_actions[i].id == caller_id) {
				found_idx = i;
				break;
				// 寻找空闲的 action

			} else if (!_actions[i].valid()) {
				free_idx = i;
			}
		}

		// 找到已有的 action
		if (found_idx != -1) {
			if (_actions[found_idx].activated && !_duplicate_reported_once) {
				PX4_ERR("BUG: duplicate check for caller_id %i", caller_id);
				_duplicate_reported_once = true;
			}

			_actions[found_idx].state_failure = true;
			_actions[found_idx].activated = true;
			_actions[found_idx].action = options.action; // Allow action to be updated, but keep the rest

			if (!last_state_failure) {
				PX4_DEBUG("Caller %i: state changed to failed, action already active", caller_id);
			}

		} else {
			// 没有找到现成的action，且无空闲的action位置.
			if (free_idx == -1) {
				PX4_ERR("No free failsafe action idx");

				// replace based on action severity
				// 翻译：根据操作严重性进行替换
				for (int i = 0; i < max_num_actions; ++i) {
					if (options.action > _actions[i].action) {
						free_idx = i;
					}
				}
			}

			// 找到空闲位置
			if (free_idx != -1) {
				_actions[free_idx] = options;
				_actions[free_idx].id = caller_id;
				_actions[free_idx].state_failure = true;
				_actions[free_idx].activated = true;

				if (options.allow_user_takeover == UserTakeoverAllowed::Auto) {
					if (_param_com_fail_act_t.get() > 0.1f) {
						if (options.action != Action::Warn && _current_delay == 0) {
							_current_delay = _current_start_delay;
						}

						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::Always;

					} else {
						_actions[free_idx].allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;
					}
				}

				if (options.action != Action::None) { // If not disabled
					_pending_notification_cause = options.cause;
				}

				if (options.action >= Action::Hold) { // If not a Fallback
					_user_takeover_active = false; // Clear takeover
				}

				PX4_DEBUG("Caller %i: state changed to failed, adding action", caller_id);
			}
		}

	} else if (last_state_failure && !cur_state_failure) {
		// Invalid -> valid transition: remove action
		bool found = false;

		for (int i = 0; i < max_num_actions; ++i) {
			if (_actions[i].id == caller_id) {
				if (found) {
					PX4_ERR("Dup action with ID %i", caller_id);
				}

				removeAction(_actions[i]);
				found = true;
			}
		}

		// It's ok if we did not find the action, it might already have been removed due to not being activated
	}

	return cur_state_failure;
}

/**
 * @brief
 *
 * @param action
 */
void FailsafeBase::removeAction(ActionOptions &action) const
{
	// If failsafes are being deferred and the action can be deferred, remove it immediately independent of the
	// clear_condition to avoid triggering a failsafe after deferring is disabled.
	// 翻译：如果故障保护被推迟并且可以推迟操作，则立即将其移除，而不考虑 clear_condition，以避免在禁用推迟后触发故障保护。
	const bool remove_while_deferring = _defer_failsafes && action.can_be_deferred;

	if (action.clear_condition == ClearCondition::WhenConditionClears || remove_while_deferring) {
		// Remove action
		PX4_DEBUG("Caller %i: state changed to valid, removing action", action.id);
		action.setInvalid();

	} else {
		if (action.state_failure) {
			PX4_DEBUG("Caller %i: state changed to valid, keeping action", action.id);
		}

		// Keep action, just flag the state
		action.state_failure = false;
	}
}

/**
 * @brief 删除无效的不活跃的action
 *
 */
void FailsafeBase::removeNonActivatedActions()
{
	// A non-activated action means the check was not called during the last update:
	// treat the state as valid and remove the action depending on the clear_condition
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid() && !cur_action.activated) {
			if (_actions[action_idx].state_failure) {
				PX4_DEBUG("Caller %i: action not activated", cur_action.id);
			}

			removeAction(cur_action);
		}

		cur_action.activated = false;
	}
}


/**
 * @brief 根据当前状态和标志选择合适的故障安全动作。
 *
 * 在函数 CHECK_FAILSAFE() 中根据配置和故障类型注册了action，此函数就是根据所有的故障选择最终需要执行的action
 * 该函数用于计算并返回选定的故障安全动作状态，包括动作类型、原因、用户意图模式更新等。
 * 它考虑了多种因素，如用户接管、延迟故障安全、模式可用性等，以确保系统安全。
 * 逻辑流程：
 * 1. 处理终止情况：如果用户意图为终止或已选择终止，直接返回终止动作。
 * 2. 如果未武装，返回无动作。
 * 3. 遍历所有动作，选择最严重的动作，并确定用户接管允许级别和是否可延迟。
 * 4. 如果允许延迟故障安全，则可能延迟动作。
 * 5. 处理延迟Hold：如果有延迟且条件满足，将动作设置为Hold。
 * 6. 处理用户接管：如果允许，用户可通过模式切换或RC摇杆接管，动作降为警告。
 * 7. 检查模式回退：确保选定动作对应的模式可运行，否则回退到更安全模式。
 * 8. UX优化：避免在已处于某些模式（如着陆、RTL）时重复进入类似动作。
 *
 * @param state 当前车辆状态，包括用户意图模式等。
 * @param status_flags 故障安全标志，用于检查模式可用性。
 * @param user_intended_mode_updated 用户意图模式是否已更新。
 * @param rc_sticks_takeover_request RC摇杆接管请求。
 * @param[out] returned_state 返回的选定动作状态，包括动作、原因、延迟动作等。
 */
void FailsafeBase::getSelectedAction(const State &state, const failsafe_flags_s &status_flags,
				     bool user_intended_mode_updated,
				     bool rc_sticks_takeover_request,
				     SelectedActionState &returned_state) const
{
	// 保底设置避免故障
	returned_state.updated_user_intended_mode = state.user_intended_mode;
	// 故障原因
	returned_state.cause = Cause::Generic;

	// 处理终止情况：如果用户意图为终止或已选择终止，直接返回终止动作
	if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_TERMINATION
	    || _selected_action == Action::Terminate) { // Terminate never clears
		returned_state.action = Action::Terminate;
		return;
	}

	// 如果车辆未武装，则返回无动作
	if (!state.armed) {
		returned_state.action = Action::None;
		return;
	}

	returned_state.action = Action::None;
	/**
	 * @param selected_action 用于存储 _actions[] 的值
	 * 这里不是简单的赋值而是类似c语言的指针用法，表示引用，修改 selected_action 等同于修改 returned_state.action
	 * c写法为：Action *selected_action = &returned_state->action; // 取地址
	 *         *selected_action = Disarm; // 解引用修改
	 */
	Action &selected_action = returned_state.action;
	UserTakeoverAllowed allow_user_takeover = UserTakeoverAllowed::Always;
	bool allow_failsafe_to_be_deferred{true};

	// Select the worst action based on the current active actions
	// 翻译：根据当前的活动动作选择最坏的动作
	for (int action_idx = 0; action_idx < max_num_actions; ++action_idx) {
		const ActionOptions &cur_action = _actions[action_idx];

		if (cur_action.valid()) {
			if (cur_action.action == Action::None) {
				continue;
			}

			if (cur_action.allow_user_takeover > allow_user_takeover) {
				// Use the most restrictive setting among all active actions
				allow_user_takeover = cur_action.allow_user_takeover;
			}

			if (cur_action.action > selected_action) {
				selected_action = cur_action.action;
				returned_state.cause = cur_action.cause;
			}

			if (!cur_action.can_be_deferred) {
				allow_failsafe_to_be_deferred = false;
			}
		}
	}

	// 推迟failsafe保护
	if (_defer_failsafes && allow_failsafe_to_be_deferred && selected_action != Action::None) {
		returned_state.failsafe_deferred = selected_action > Action::Warn;
		returned_state.action = Action::None;
		return;
	}

	// Check if we should enter delayed Hold
	// 翻译：检查是否应进入延迟保持状态。
	// e.g. 当rc连接断开，不应立即触发保护，而是若超过一段时间还没有恢复则执行保护.
	const bool action_can_be_delayed = selected_action != Action::None &&
					   selected_action != Action::Warn &&
					   selected_action != Action::Disarm &&
					   selected_action != Action::Terminate &&
					   selected_action != Action::Hold;

	if (_current_delay > 0 && !_user_takeover_active && allow_user_takeover <= UserTakeoverAllowed::AlwaysModeSwitchOnly
	    && action_can_be_delayed) {
		// 保存目标动作
		returned_state.delayed_action = selected_action;
		// 延迟保护,所以将 selected_action 替换为Hold
		selected_action = Action::Hold;
		allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;
	}

	// User takeover interrupting a failsafe is triggered by a change of the user-intended mode
	// (only if a failsafe action is already active otherwise there can be immediate takeover when entering a failsafe) or by stick movement
	// 翻译：用户接管中断安全机制的触发条件是用户预期模式的改变
	//	(仅当安全机制操作已激活时才会触发，否则进入安全机制时可能会立即发生接管)或摇杆移动
	bool want_user_takeover_mode_switch = user_intended_mode_updated && (_selected_action > Action::Warn);
	bool want_user_takeover = want_user_takeover_mode_switch || rc_sticks_takeover_request;
	bool takeover_allowed =
		(allow_user_takeover == UserTakeoverAllowed::Always && (_user_takeover_active || want_user_takeover))
		|| (allow_user_takeover == UserTakeoverAllowed::AlwaysModeSwitchOnly && (_user_takeover_active
				|| want_user_takeover_mode_switch));

	// action 允许用户接管
	if (actionAllowsUserTakeover(selected_action) && takeover_allowed) {
		if (!_user_takeover_active && rc_sticks_takeover_request) {
			// TODO: if the user intended mode is a stick-controlled mode, switch back to that instead
			// 翻译：TODO：如果用户意图更新模式为摇杆控制模式，则切换回该模式。
			returned_state.updated_user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		}

		selected_action = Action::Warn;
		returned_state.user_takeover = true;
		returned_state.delayed_action = Action::None;

		if (!_user_takeover_active) {
			PX4_DEBUG("Activating user takeover");
#ifndef EMSCRIPTEN_BUILD
			events::send(events::ID("failsafe_rc_override"), events::Log::Info, "Pilot took over using sticks");
#endif // EMSCRIPTEN_BUILD
		}

		// We must check for mode fallback again here
		// 翻译：我们必须在此处再次检查模式后备
		Action mode_fallback = checkModeFallback(status_flags, modeFromAction(selected_action,
				       returned_state.updated_user_intended_mode));

		if (mode_fallback > selected_action) {
			selected_action = mode_fallback;
		}
	}

	// Check if the selected action is possible, and fall back if needed
	// 翻译：检查选定的操作是否可能，并在需要时退回
	switch (selected_action) {

	case Action::FallbackPosCtrl:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_POSCTL)) {
			selected_action = Action::FallbackPosCtrl;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::FallbackAltCtrl:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_ALTCTL)) {
			selected_action = Action::FallbackAltCtrl;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::FallbackStab:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_STAB)) {
			selected_action = Action::FallbackStab;
			break;
		} // else: fall through here as well. If stabilized isn't available, we most certainly end up in Terminate

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Hold:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
			selected_action = Action::Hold;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::RTL:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)) {
			selected_action = Action::RTL;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Land:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Land;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Descend:
		if (modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_DESCEND)
		    && _param_com_pos_fs_act.get() != (int32_t)PositionFailsafeAction::Terminate) {
			selected_action = Action::Descend;
			break;
		}

		returned_state.cause = Cause::Generic;

	// fallthrough
	case Action::Terminate:
		selected_action = Action::Terminate;
		break;

	case Action::Disarm:
		selected_action = Action::Disarm;
		break;

	case Action::None:
	case Action::Warn:
	case Action::Count:
		break;
	}

	// 下方的作用是,如果在用户期望的模式为特殊模式如(LAND,RTL等)，但是由于安全故障导致action为其他任务,而导致的不安全行为，提前预防
	// UX improvement (this is optional for safety): change failsafe to a warning in certain situations.
	// If already landing, do not go into RTL

	// 翻译：用户体验改进（出于安全考虑，此项为可选）：在某些情况下将故障保护更改为警告。
	// 	如果已在着陆，则不要进入返航状态
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		if ((selected_action == Action::RTL || returned_state.delayed_action == Action::RTL)
		    && modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}

	// If already in RTL, do not go into RTL again (would cause a Hold delay first, then re-start RTL)
	// 翻译：如果已处于 RTL 模式，请勿再次进入 RTL 模式（这将导致先进入锁定延迟，然后再重新启动 RTL 模式）
	// 这样做是因为触发RTL后通常伴随着先爬升到指定高度，若已经触发RTL再次因故障保护触发RTL会导致不必要的爬升出现
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		if ((selected_action == Action::RTL || returned_state.delayed_action == Action::RTL)
		    && modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}

	// If already precision landing, do not go into RTL or Land
	if (returned_state.updated_user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND) {
		if ((selected_action == Action::RTL || selected_action == Action::Land ||
		     returned_state.delayed_action == Action::RTL || returned_state.delayed_action == Action::Land)
		    && modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND)) {
			selected_action = Action::Warn;
			returned_state.delayed_action = Action::None;
		}
	}
}

bool FailsafeBase::actionAllowsUserTakeover(Action action) const
{
	// Stick-controlled modes do not need user takeover
	return action == Action::Hold || action == Action::RTL || action == Action::Land || action == Action::Descend;
}

/**
 * @brief 如果需要,删除延迟
 * @param state
 * @param status_flags
 */
void FailsafeBase::clearDelayIfNeeded(const State &state,
				      const failsafe_flags_s &status_flags)
{
	// Clear delay if one of the following is true:
	// 翻译：如果下列之一成立，则清除延迟：
	// - Already in a failsafe
	// - Hold not available
	// - Takeover is active (due to a mode switch during the delay)
	if (_selected_action > Action::Hold || !modeCanRun(status_flags, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
	    || _user_takeover_active) {
		if (_current_delay > 0) {
			PX4_DEBUG("Clearing delay, Hold not available, already in failsafe or taken over");
		}

		_current_delay = 0;
	}
}

uint8_t FailsafeBase::modeFromAction(const Action &action, uint8_t user_intended_mode)
{
	switch (action) {

	case Action::FallbackPosCtrl: return vehicle_status_s::NAVIGATION_STATE_POSCTL;

	case Action::FallbackAltCtrl: return vehicle_status_s::NAVIGATION_STATE_ALTCTL;

	case Action::FallbackStab: return vehicle_status_s::NAVIGATION_STATE_STAB;

	case Action::Hold: return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	case Action::RTL: return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

	case Action::Land: return vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

	case Action::Descend: return vehicle_status_s::NAVIGATION_STATE_DESCEND;

	case Action::Terminate:
	case Action::Disarm:
	case Action::None:
	case Action::Warn:
	case Action::Count:
		break;
	}

	return user_intended_mode;
}

bool FailsafeBase::modeCanRun(const failsafe_flags_s &status_flags, uint8_t mode)
{
	uint32_t mode_mask = 1u << mode;
	// mode_req_wind_and_flight_time_compliance: does not need to be handled here (these are separate failsafe triggers)
	// mode_req_manual_control: is handled separately
	// 翻译：mode_req_wind_and_flight_time_compliance：不需要在此处处理（这些是单独的故障安全触发器）
	//	MODE_REQ_MANUAL_CONTROL：单独处理
	return
		(!status_flags.angular_velocity_invalid || ((status_flags.mode_req_angular_velocity & mode_mask) == 0)) &&
		(!status_flags.attitude_invalid || ((status_flags.mode_req_attitude & mode_mask) == 0)) &&
		(!status_flags.local_position_invalid || ((status_flags.mode_req_local_position & mode_mask) == 0)) &&
		(!status_flags.local_position_invalid_relaxed || ((status_flags.mode_req_local_position_relaxed & mode_mask) == 0)) &&
		(!status_flags.global_position_invalid || ((status_flags.mode_req_global_position & mode_mask) == 0)) &&
		(!status_flags.global_position_invalid_relaxed || ((status_flags.mode_req_global_position_relaxed & mode_mask) == 0)) &&
		(!status_flags.local_altitude_invalid || ((status_flags.mode_req_local_alt & mode_mask) == 0)) &&
		(!status_flags.auto_mission_missing || ((status_flags.mode_req_mission & mode_mask) == 0)) &&
		(!status_flags.offboard_control_signal_lost || ((status_flags.mode_req_offboard_signal & mode_mask) == 0)) &&
		(!status_flags.home_position_invalid || ((status_flags.mode_req_home_position & mode_mask) == 0)) &&
		((status_flags.mode_req_other & mode_mask) == 0);
}

// 延迟所有可以延迟的故障保护机制，避免在紧急情况下触发故障保护
bool FailsafeBase::deferFailsafes(bool enabled, int timeout_s)
{
	if (enabled && _selected_action > Action::Warn) {
		return false;
	}

	if (!enabled && _defer_failsafes && _failsafe_defer_started == 0) {
		_current_delay = 0;
	}

	if (timeout_s == 0) {
		_defer_timeout = DEFAULT_DEFER_TIMEOUT;

	} else if (timeout_s < 0) {
		_defer_timeout = 0;

	} else {
		_defer_timeout = timeout_s * 1_s;
	}

	_defer_failsafes = enabled;
	return true;
}
