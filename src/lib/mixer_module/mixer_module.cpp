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

#include "mixer_module.hpp"

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;


/**
 * FunctionProvider提供了两个构造函数：
 *
 *  FunctionProvider(OutputFunction min_func_, OutputFunction max_func_, Constructor constructor_)
 *      : min_func(min_func_), max_func(max_func_), constructor(constructor_) {}
 *
 *  - 接受三个参数：`min_func_`（最小功能标识）、`max_func_`（最大功能标识）、`constructor_`（构造函数指针）。
 *  - 使用**初始化列表**（`: min_func(min_func_)` 等）将参数值赋值给成员变量。
 *  - 这个构造函数适用于定义一个功能范围（从 `min_func_` 到 `max_func_`）以及对应的构造函数。
 *
 *  FunctionProvider(OutputFunction func, Constructor constructor_)
 *      : min_func(func), max_func(func), constructor(constructor_) {}
 *
 *  - 接受两个参数：`func`（单一的功能标识）和 `constructor_`（构造函数指针）。
 *  - 将 `min_func` 和 `max_func` 都设置为同一个 `func`，表示这个功能没有范围（最小值和最大值相同）。
 *  - 同样使用初始化列表赋值。
 *
 */
struct FunctionProvider {
	using Constructor = FunctionProviderBase * (*)(const FunctionProviderBase::Context &context);
	constexpr FunctionProvider(OutputFunction min_func_, OutputFunction max_func_, Constructor constructor_)
		: min_func(min_func_), max_func(max_func_), constructor(constructor_) {}
	constexpr FunctionProvider(OutputFunction func, Constructor constructor_)
		: min_func(func), max_func(func), constructor(constructor_) {}

	OutputFunction min_func;
	OutputFunction max_func;
	Constructor constructor;
};

static constexpr FunctionProvider all_function_providers[] = {
	// Providers higher up take precedence for subscription callback in case there are multiple
	{OutputFunction::Constant_Min, &FunctionConstantMin::allocate},
	{OutputFunction::Constant_Max, &FunctionConstantMax::allocate},
	{OutputFunction::Motor1, OutputFunction::MotorMax, &FunctionMotors::allocate},
	{OutputFunction::Servo1, OutputFunction::ServoMax, &FunctionServos::allocate},
	{OutputFunction::Peripheral_via_Actuator_Set1, OutputFunction::Peripheral_via_Actuator_Set6, &FunctionActuatorSet::allocate},
	{OutputFunction::Landing_Gear, &FunctionLandingGear::allocate},
	{OutputFunction::Landing_Gear_Wheel, &FunctionLandingGearWheel::allocate},
	{OutputFunction::Parachute, &FunctionParachute::allocate},
	{OutputFunction::Gripper, &FunctionGripper::allocate},
	{OutputFunction::RC_Roll, OutputFunction::RC_AUXMax, &FunctionManualRC::allocate},
	{OutputFunction::Gimbal_Roll, OutputFunction::Gimbal_Yaw, &FunctionGimbal::allocate},
	{OutputFunction::IC_Engine_Ignition, OutputFunction::IC_Engine_Starter, &FunctionICEControl::allocate},
};

MixingOutput::MixingOutput(const char *param_prefix, uint8_t max_num_outputs, OutputModuleInterface &interface,
			   SchedulingPolicy scheduling_policy, bool support_esc_calibration, bool ramp_up, const uint8_t instance_start) :
	ModuleParams(&interface),
	_output_ramp_up(ramp_up),
	_scheduling_policy(scheduling_policy),
	_support_esc_calibration(support_esc_calibration),
	_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
	_interface(interface),
	_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency")),
	_param_prefix(param_prefix)
{
	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.termination = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	initParamHandles(instance_start);

	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_failsafe_value[i] = UINT16_MAX;
	}

	updateParams();

	_outputs_pub.advertise();
}

MixingOutput::~MixingOutput()
{
	perf_free(_control_latency_perf);
	px4_sem_destroy(&_lock);

	cleanupFunctions();

	_outputs_pub.unadvertise();
}

void MixingOutput::initParamHandles(const uint8_t instance_start)
{
	char param_name[17];

	for (unsigned i = 0; i < _max_num_outputs; ++i) {
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FUNC", i + instance_start);
		_param_handles[i].function = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "DIS", i + instance_start);
		_param_handles[i].disarmed = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MIN", i + instance_start);
		_param_handles[i].min = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "CENT", i + instance_start);
		_param_handles[i].center = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "MAX", i + instance_start);
		_param_handles[i].max = param_find(param_name);
		snprintf(param_name, sizeof(param_name), "%s_%s%d", _param_prefix, "FAIL", i + instance_start);
		_param_handles[i].failsafe = param_find(param_name);
	}

	snprintf(param_name, sizeof(param_name), "%s_%s", _param_prefix, "REV");
	_param_handle_rev_range = param_find(param_name);
}

void MixingOutput::printStatus() const
{
	PX4_INFO("Param prefix: %s", _param_prefix);
	perf_print_counter(_control_latency_perf);

	if (_wq_switched) {
		PX4_INFO("Switched to rate_ctrl work queue");
	}

	PX4_INFO_RAW("Channel Configuration:\n");

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		PX4_INFO_RAW("Channel %2d: func: %3d, value: %.2f, failsafe: %.2f, disarmed: %d, min: %d, max: %d, center: %d\n",
			     i, (int)_function_assignment[i], (double)_current_output_value[i], (double)actualFailsafeValue(i),
			     _disarmed_value[i], _min_value[i], _max_value[i], _center_value[i]);
	}
}

void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	bool function_changed = false;

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int32_t val;

		if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
			if (val != (int32_t)_function_assignment[i]) {
				function_changed = true;
			}

			// we set _function_assignment[i] later to ensure _functions[i] is updated at the same time
			// 翻译:我们稍后会设置 _function_assignment[i]，以确保 _functions[i] 同时更新。
		}

		if (_param_handles[i].disarmed != PARAM_INVALID && param_get(_param_handles[i].disarmed, &val) == 0) {
			_disarmed_value[i] = val;
		}

		if (_param_handles[i].min != PARAM_INVALID && param_get(_param_handles[i].min, &val) == 0) {
			_min_value[i] = val;
		}

		if (_param_handles[i].center != PARAM_INVALID && param_get(_param_handles[i].center, &val) == 0) {
			_center_value[i] = val;
		}

		if (_param_handles[i].max != PARAM_INVALID && param_get(_param_handles[i].max, &val) == 0) {
			_max_value[i] = val;
		}

		if (_min_value[i] > _max_value[i]) {
			uint16_t tmp = _min_value[i];
			_min_value[i] = _max_value[i];
			_max_value[i] = tmp;
		}


		if (_param_handles[i].failsafe != PARAM_INVALID && param_get(_param_handles[i].failsafe, &val) == 0) {
			_failsafe_value[i] = val;
		}
	}

	_reverse_output_mask = 0;
	int32_t rev_range_param;

	if (_param_handle_rev_range != PARAM_INVALID && param_get(_param_handle_rev_range, &rev_range_param) == 0) {
		_reverse_output_mask = rev_range_param;
	}

	if (function_changed) {
		_need_function_update = true;
	}
}

void MixingOutput::cleanupFunctions()
{
	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
		_subscription_callback = nullptr;
	}

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		delete _function_allocated[i];
		_function_allocated[i] = nullptr;
		_functions[i] = nullptr;
	}
}

/**
 * @brief 更新订阅
 * @param allow_wq_switch 是否允许工作队列切换。
 */
bool MixingOutput::updateSubscriptions(bool allow_wq_switch)
{
	if (!_need_function_update || _armed.armed) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	// 翻译：必须被锁定才可能改变工作场
	lock();

	_has_backup_schedule = false;

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		// 翻译：首先清理所有东西
		unregister();
		_interface.ScheduleClear();

		bool switch_requested = false;

		// potentially switch work queue if we run motor outputs
		// 翻译：如果我们运行电机输出那么可能切换工作队列
		for (unsigned i = 0; i < _max_num_outputs; i++) {
			// read function directly from param, as _function_assignment[i] is updated later
			// 翻译：直接从参数中读取功能，由于 _function_assignment [i]将稍后更新
			int32_t function;

			// _param_handles 等参数 在 initParamHandles 中初始化
			if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &function) == 0) {
				if (function >= (int32_t)OutputFunction::Motor1 && function <= (int32_t)OutputFunction::MotorMax) {
					switch_requested = true;
				}
			}
		}

		if (allow_wq_switch && !_wq_switched && switch_requested) {
			if (_interface.ChangeWorkQueue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				// 翻译：让新的WQ处理订阅更新
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}
	}

	// Now update the functions
	// 翻译：现在更新功能
	PX4_DEBUG("updating functions");

	cleanupFunctions();

	const FunctionProviderBase::Context context{_interface, _param_thr_mdl_fac.reference()};
	// 记录已分配的 provider 在 all_function_providers 数组中的索引（即 p 值），数组元素按 next_provider 顺序填充（未填充的元素为 0）
	int provider_indexes[MAX_ACTUATORS] {};
	// 已经分配的 provider 数量（也是 _function_allocated 下一个可用槽的索引）
	int next_provider = 0;
	int subscription_callback_provider_index = INT_MAX;
	bool all_disabled = true;

	for (int i = 0; i < _max_num_outputs; ++i) {
		int32_t val;

		// function 等参数 在 initParamHandles 中初始化
		if (_param_handles[i].function != PARAM_INVALID && param_get(_param_handles[i].function, &val) == 0) {
			_function_assignment[i] = (OutputFunction)val;

		} else {
			_function_assignment[i] = OutputFunction::Disabled;
		}

		for (int p = 0; p < (int)(sizeof(all_function_providers) / sizeof(all_function_providers[0])); ++p) {
			if (_function_assignment[i] >= all_function_providers[p].min_func &&
			    _function_assignment[i] <= all_function_providers[p].max_func) {
				all_disabled = false;
				int found_index = -1;

				/**
				 * p=0=min_func; p=1=max_func; p=2=Motor1~MotorMax ; p=3=Servo1~ServoMax
				 * 假设我的配置为Motor1, Motor2, Servo1
				 * 当第一次运行时 i = 0,p = 2 时会创建 Motor1 的 provider 实例，并记录 provider_indexes[0] = 2
				 * 当第二次运行时 i = 1,p = 2 时会复用
				 * 当第三次运行时 i = 2,p = 3 时会创建 Servo1 的 provider 实例，并记录 provider_indexes[1] = 3
				 */
				for (int existing = 0; existing < next_provider; ++existing) {
					if (provider_indexes[existing] == p) {
						found_index = existing;
						break;
					}
				}

				// 如果找到则复用已分配的 provider 实例，若没有找到则创建一个新的 provider 实例
				if (found_index >= 0) {
					_functions[i] = _function_allocated[found_index];

				} else {
					// 新建实例
					_function_allocated[next_provider] = all_function_providers[p].constructor(context);

					// 判断是否构建成功，若成功则分配给当前通道，并记录 provider 索引
					if (_function_allocated[next_provider]) {
						_functions[i] = _function_allocated[next_provider];
						provider_indexes[next_provider++] = p;

						// lowest provider takes precedence for scheduling
						// 翻译：最低提供者优先考虑调度
						if (p < subscription_callback_provider_index && _functions[i]->subscriptionCallback()) {
							subscription_callback_provider_index = p;
							_subscription_callback = _functions[i]->subscriptionCallback();
						}

					} else {
						PX4_ERR("function alloc failed");
					}
				}

				break;
			}
		}
	}

	hrt_abstime fixed_rate_scheduling_interval = 4_ms; // schedule at 250Hz

	if (_max_topic_update_interval_us > fixed_rate_scheduling_interval) {
		fixed_rate_scheduling_interval = _max_topic_update_interval_us;
	}

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		if (_subscription_callback) {
			if (_subscription_callback->registerCallback()) {
				PX4_DEBUG("Scheduling via callback");
				_has_backup_schedule = true;
				_interface.ScheduleDelayed(50_ms);

			} else {
				PX4_ERR("registerCallback failed, scheduling at fixed rate");
				_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			}

		} else if (all_disabled) {
			_interface.ScheduleOnInterval(_lowrate_schedule_interval);
			PX4_DEBUG("Scheduling at low rate");

		} else {
			_interface.ScheduleOnInterval(fixed_rate_scheduling_interval);
			PX4_DEBUG("Scheduling at fixed rate");
		}
	}

	setMaxTopicUpdateRate(_max_topic_update_interval_us);
	_need_function_update = false;

	_actuator_test.reset();

	unlock();

	_interface.mixerChanged();

	return true;
}

void MixingOutput::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	if (_subscription_callback) {
		_subscription_callback->set_interval_us(_max_topic_update_interval_us);
	}
}

void MixingOutput::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].min = PARAM_INVALID;
		_min_value[i] = value;
	}
}

void MixingOutput::setAllCenterValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].center = PARAM_INVALID;
		_center_value[i] = value;
	}
}

void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].max = PARAM_INVALID;
		_max_value[i] = value;
	}
}

void MixingOutput::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].failsafe = PARAM_INVALID;
		_failsafe_value[i] = value;
	}
}

void MixingOutput::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_param_handles[i].disarmed = PARAM_INVALID;
		_disarmed_value[i] = value;
	}
}

void MixingOutput::unregister()
{
	if (_subscription_callback) {
		_subscription_callback->unregisterCallback();
	}
}

bool MixingOutput::update()
{
	// check arming state
	// 翻译：检查武装状态
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		// 翻译：更新武装状态并且检查我们是不是没有被锁定
		//      我们还需要为电调校准启用油门
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;
	}

	// only used for sitl with lockstep
	// 翻译：仅用于锁定的SITL
	bool has_updates = _subscription_callback && _subscription_callback->updated();

	// update topics
	for (int i = 0; i < MAX_ACTUATORS && _function_allocated[i]; ++i) {
		_function_allocated[i]->update();
	}

	if (_has_backup_schedule) {
		_interface.ScheduleDelayed(50_ms);
	}

	// check for actuator test
	_actuator_test.update(_max_num_outputs, _param_thr_mdl_fac.get());

	// get output values
	float outputs[MAX_ACTUATORS];
	bool all_disabled = true;
	const uint32_t reversible_mask_prev = _reversible_mask;
	_reversible_mask = 0;

	for (int i = 0; i < _max_num_outputs; ++i) {
		if (_functions[i]) {
			all_disabled = false;

			if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
				outputs[i] = _functions[i]->value(_function_assignment[i]);

			} else {
				outputs[i] = NAN;
			}

			_reversible_mask |= (uint32_t)_functions[i]->reversible(_function_assignment[i]) << i;

		} else {
			outputs[i] = NAN;
		}
	}

	if (_reversible_mask != reversible_mask_prev) {
		_interface.reversibleMaskChanged(_reversible_mask);
	}

	// Send output if any function mapped or one last disabling sample
	// 翻译：如果有任何函数被映射或最后一个禁用样本，则发送输出
	if (!all_disabled || !_was_all_disabled) {
		if (!_armed.armed && !_armed.kill) {
			_actuator_test.overrideValues(outputs, _max_num_outputs);
		}

		// 输出限制-也是实际的输出
		limitAndUpdateOutputs(outputs, has_updates);
	}

	_was_all_disabled = all_disabled;

	return true;
}

void
MixingOutput::limitAndUpdateOutputs(float outputs[MAX_ACTUATORS], bool has_updates)
{
	if (_armed.lockdown || _armed.kill) {
		// overwrite outputs in case of lockdown with disarmed values
		// 翻译：在锁定的情况下使用解除武装的值覆盖输出
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

	} else if (_armed.termination) {
		// Overwrite outputs with _failsafe_value when terminated
		// 翻译：终止时用_failSafe_value覆盖输出
		for (size_t i = 0; i < _max_num_outputs; i++) {
			_current_output_value[i] = actualFailsafeValue(i);
		}

	} else {
		// the output limit call takes care of out of band errors, NaN and constrains
		// 翻译：输出限制调用处理带外错误、NaN 和约束
		output_limit_calc(_throttle_armed || _actuator_test.inTestMode(), _max_num_outputs, outputs);
	}

	// We must calibrate the PWM and Oneshot ESCs to a consistent range of 1000-2000us (gets mapped to 125-250us for Oneshot)
	// Doing so makes calibrations consistent among different configurations and hence PWM minimum and maximum have a consistent effect
	// hence the defaults for these parameters also make most setups work out of the box
	// 翻译：我们必须将PWM和Oneshot电调校准到1000-2000微秒的统一范围（Oneshot对应125-250微秒）。
	//      这样做可以确保不同配置下的校准结果一致，从而使PWM的最小值和最大值产生一致的效果。
	//      因此，这些参数的默认值也能保证大多数设置开箱即用。
	if (_armed.in_esc_calibration_mode) {
		static constexpr float PWM_CALIBRATION_LOW = 1000.f;
		static constexpr float PWM_CALIBRATION_HIGH = 2000.f;

		for (int i = 0; i < _max_num_outputs; i++) {
			if (fabsf(_current_output_value[i] - (float)_min_value[i]) < 0.5f) {
				_current_output_value[i] = PWM_CALIBRATION_LOW;
			}

			if (fabsf(_current_output_value[i] - (float)_max_value[i]) < 0.5f) {
				_current_output_value[i] = PWM_CALIBRATION_HIGH;
			}
		}
	}

	/* now return the outputs to the driver */
	// 翻译：现在将输出返回驱动程序
	if (_interface.updateOutputs(_current_output_value, _max_num_outputs, has_updates)) {
		actuator_outputs_s actuator_outputs{};
		// 发布执行器输出
		setAndPublishActuatorOutputs(_max_num_outputs, actuator_outputs);

		// 性能记录
		updateLatencyPerfCounter(actuator_outputs);
	}
}

float MixingOutput::output_limit_calc_single(int i, float value) const
{
	// check for invalid / disabled channels
	if (!PX4_ISFINITE(value)) {
		return _disarmed_value[i];
	}

	if (_reverse_output_mask & (1 << i)) {
		value = -1.f * value;
	}

	float output = _disarmed_value[i];

	if (((_function_assignment[i] >= OutputFunction::Servo1
	      && _function_assignment[i] <= OutputFunction::ServoMax) || _function_assignment[i] == OutputFunction::Landing_Gear_Wheel
	     || (_function_assignment[i] >= OutputFunction::Gimbal_Roll
		 && _function_assignment[i] <= OutputFunction::Gimbal_Yaw))
	    && _param_handles[i].center != PARAM_INVALID
	    && _center_value[i] >= 800
	    && _center_value[i] <= 2200) {
		output = math::interpolateNXY(value, {-1.f, 0.f, 1.f}, {(float)_min_value[i], (float)_center_value[i], (float)_max_value[i]});
	}

	// Everything except servos, or if center is not set
	// 翻译：除舵机外，或未设置中心位置时，所有电机均适用。
	else {
		output = math::interpolate(value, -1.f, 1.f, static_cast<float>(_min_value[i]), static_cast<float>(_max_value[i]));
	}

	return output;
}

void
MixingOutput::output_limit_calc(const bool armed, const int num_channels, const float output[MAX_ACTUATORS])
{
	// time to slowly ramp up the ESCs
	// 翻译：是时候逐步增加 ESC 了。
	static constexpr hrt_abstime RAMP_TIME_US = 500_ms;

	/* first evaluate state changes */
	// 翻译：首先评估状态变化。
	switch (_output_state) {
	case OutputLimitState::OFF:
		if (armed) {
			if (_output_ramp_up) {
				_output_state = OutputLimitState::RAMP;

			} else {
				_output_state = OutputLimitState::ON;
			}

			// reset arming time, used for ramp timing
			// 翻译：重置启动时间，用于斜坡计时。
			_output_time_armed = hrt_absolute_time();
		}

		break;

	// 爬升阶段 当超过设定的时间后切换输出状态为ON
	case OutputLimitState::RAMP:
		if (!armed) {
			_output_state = OutputLimitState::OFF;

		} else if (hrt_elapsed_time(&_output_time_armed) >= RAMP_TIME_US) {
			_output_state = OutputLimitState::ON;
		}

		break;

	case OutputLimitState::ON:
		if (!armed) {
			_output_state = OutputLimitState::OFF;
		}

		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 */

	/*
	 * 翻译：如果系统已预先准备就绪，则限位状态会暂时开启，因为某些输出有效，而无效输出已被设置为 NaN。
	 *      但这不会存储在状态机中，因为油门通道需要在正常的准备时间经过斜坡过程。
	 */
	auto local_limit_state = _output_state;

	if (isPrearmed()) {
		local_limit_state = OutputLimitState::ON;
	}

	// then set _current_output_value based on state
	switch (local_limit_state) {
	case OutputLimitState::OFF:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		break;

	case OutputLimitState::RAMP: {
			hrt_abstime diff = hrt_elapsed_time(&_output_time_armed);
			float progress = static_cast<float>(diff) / RAMP_TIME_US;

			if (progress > 1.f) {
				progress = 1.f;
			}

			for (int i = 0; i < num_channels; i++) {
				// Ramp from disarmed value to currently desired output that would apply without ramp
				float desired_output = output_limit_calc_single(i, output[i]);
				_current_output_value[i] = _disarmed_value[i] + progress * (desired_output - _disarmed_value[i]);
			}
		}
		break;

	case OutputLimitState::ON:
		for (int i = 0; i < num_channels; i++) {
			_current_output_value[i] = output_limit_calc_single(i, output[i]);
		}

		break;
	}
}

/**
 * @brief 设置并发布执行器输出
 * @param num_outputs 执行器输出的数量
 * @param actuator_outputs 执行器输出结构体
 */
void
MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);
}

/**
 * @brief 更新控制延迟性能计数器
 * @param actuator_outputs 执行器输出结构体
 */
void
MixingOutput::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	// Just check the first function. It means we only get the latency if motors are assigned first, which is the default
	// 翻译：只需检查第一个函数即可。这意味着只有在电机被首先分配的情况下（即默认情况），我们才能获得延迟。
	if (_function_allocated[0]) {
		hrt_abstime timestamp_sample;

		if (_function_allocated[0]->getLatestSampleTimestamp(timestamp_sample)) {
			perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
		}
	}
}

float MixingOutput::actualFailsafeValue(int index) const
{
	float value = 0;

	if (_failsafe_value[index] == UINT16_MAX) { // if set to default, use the one provided by the function
		float default_failsafe = NAN;

		if (_functions[index]) {
			default_failsafe = _functions[index]->defaultFailsafeValue(_function_assignment[index]);
		}

		value = output_limit_calc_single(index, default_failsafe);

	} else {
		value = static_cast<float>(_failsafe_value[index]);
	}

	return value;
}
