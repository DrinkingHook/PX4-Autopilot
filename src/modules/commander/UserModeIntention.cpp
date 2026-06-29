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


#include "UserModeIntention.hpp"

UserModeIntention::UserModeIntention(const vehicle_status_s &vehicle_status,
				     const HealthAndArmingChecks &health_and_arming_checks, ModeChangeHandler *handler)
	: _vehicle_status(vehicle_status), _health_and_arming_checks(health_and_arming_checks),
	  _handler(handler)
{
}

/**
 * @brief 更改模式
 * @param user_intended_nav_state 用户所需的导航模式
 * @param source 任务请求来源 user or ModeExecutor
 * @param allow_fallback
 * @param force
 * @return true
 * @return false
 */
bool UserModeIntention::change(uint8_t user_intended_nav_state, ModeChangeSource source, bool allow_fallback,
			       bool force)
{
	if (_handler) {
		// If a replacement mode is selected, select the internal one instead. The replacement will be selected after.
		// 翻译：如果选择了替换模式，请选择内部模式。之后将选择替换模式。

		// 如果存在_handler，它会检查请求的模式 （user_intended_nav_state） 是否具有预定义的替换模式。
		// 例如，由于系统约束或配置，某些模式可能会映射到内部等效模式。
		// 如果定义了替换，此步骤会修改user_intended_nav_state，确保系统在继续之前使用适当的内部模式。
		user_intended_nav_state = _handler->getReplacedModeIfAny(user_intended_nav_state);
	}

	// Always allow mode change while disarmed
	// 翻译：始终允许在撤防时更改模式
	bool always_allow = force || !isArmed();
	bool allow_change = true;

	// 若是非强制(force为false)且已武装
	if (!always_allow) {
		// 是否可以切换到指定的导航模式
		allow_change = _health_and_arming_checks.canRun(user_intended_nav_state);

		// 如果无法切换( allow_change 为false)，且 allow_fallback 为true，则判断是否可以回退切换模式为NAVIGATION_STATE_ALTCTL(定高模式)
		// Check fallback
		// 翻译：处理回调任务
		if (!allow_change && allow_fallback) {
			if (user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
				allow_change = _health_and_arming_checks.canRun(vehicle_status_s::NAVIGATION_STATE_ALTCTL);
				// We still use the original user intended mode. The failsafe state machine will then set the
				// fallback and once can_run becomes true, the actual user intended mode will be selected.
				// 翻译：我们仍然使用用户最初期望的模式。故障保护状态机随后会设置备用方案，一旦 can_run 变为 true，就会选择用户实际期望的模式。
			}
		}
	}

	// never allow to change out of termination state
	// 翻译：车辆为终止状态下的话绝不允许改变
	allow_change &= _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_TERMINATION;

	if (allow_change) {
		_had_mode_change = true;
		// 存储用户的期望模式
		_user_intented_nav_state = user_intended_nav_state;

		// Special case termination state: even though this mode prevents arming,
		// still don't switch out of it after disarm and thus store it in _nav_state_after_disarming.
		// 翻译：特殊终止状态：即使此模式会阻止布防，但在撤防后仍然不会切换出此模式，因此将其存储在 _nav_state_after_disarming 中。
		if ((!_health_and_arming_checks.modePreventsArming(user_intended_nav_state)
		     && !isTakeOffIntended(user_intended_nav_state))
		    || user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
			_nav_state_after_disarming = user_intended_nav_state;
		}

		if (_handler) {
			_handler->onUserIntendedNavStateChange(source, user_intended_nav_state);
		}
	}

	return allow_change;
}

void UserModeIntention::onDisarm()
{
	if (_handler) {
		_user_intented_nav_state = _handler->onDisarm(_nav_state_after_disarming);

	} else {
		_user_intented_nav_state = _nav_state_after_disarming;
	}
}
