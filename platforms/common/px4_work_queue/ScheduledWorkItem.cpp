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

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

namespace px4
{

ScheduledWorkItem::ScheduledWorkItem(const char *name, const wq_config_t &config) : WorkItem(name, config) {}

ScheduledWorkItem::~ScheduledWorkItem()
{
	if (_call.arg != nullptr) {
		ScheduleClear();
	}
}

/**
 * @brief 调度跳板函数（trampoline）
 * 「对象指针传递桥」，解决 C++ 成员函数无法直接作为 C 回调的问题
 * 用于工作队列或高精度定时器回调，将 void* 参数转换为类指针，
 * 并调用非静态的 ScheduleNow() 方法。
 * 注意：此函数本身不执行实际任务，仅用于类型转换和方法转发。
 *
 * @param arg this 指针（ScheduledWorkItem 对象）
 */
void ScheduledWorkItem::schedule_trampoline(void *arg)
{
	ScheduledWorkItem *dev = static_cast<ScheduledWorkItem *>(arg);
	dev->ScheduleNow();
}

/**
 * @brief 延迟指定时间后执行一次任务（一次性定时器）
 *
 * 使用高精度定时器（hrt_call）在 delay_us 微秒后调用 schedule_trampoline，
 * 从而触发本对象的 Run() 执行一次。
 *
 * @param delay_us 延迟时间，单位：微秒（us）。必须 > 0
 * @note 若已存在未触发的延迟任务，会先取消再重新注册（防重复）
 */
void ScheduledWorkItem::ScheduleDelayed(uint32_t delay_us)
{
	hrt_call_after(&_call, delay_us, (hrt_callout)&ScheduledWorkItem::schedule_trampoline, this);
}

/**
 * @brief 按固定周期重复执行任务（周期性定时器）
 *
 * 首次在 delay_us 微秒后执行，之后每 interval_us 微秒重复执行一次。
 * 实际执行入口仍是 schedule_trampoline → ScheduleNow() → Run()。
 *
 * @param interval_us  执行周期，单位：微秒（us）。必须 > 0
 * @param delay_us     首次执行前的延迟时间，单位：微秒（us）。0 表示立即开始第一个周期
 * @note 若已存在周期性任务，会先取消再重新注册
 */
void ScheduledWorkItem::ScheduleOnInterval(uint32_t interval_us, uint32_t delay_us)
{
	hrt_call_every(&_call, delay_us, interval_us, (hrt_callout)&ScheduledWorkItem::schedule_trampoline, this);
}

/**
 * @brief 在指定的绝对时间点执行一次任务
 *
 * 用于需要严格与系统绝对时间对齐的场景（比如对齐 GPS 时间、传感器采样相位等）。
 *
 * @param time_us  期望触发时的绝对时间，单位：微秒（自开机起，hrt_absolute_time）
 * @note 如果指定时间已过期，会立即执行一次（行为等同 ScheduleNow()）
 * @note 若已存在未触发的定时任务，会被覆盖
 */
void ScheduledWorkItem::ScheduleAt(hrt_abstime time_us)
{
	hrt_call_at(&_call, time_us, (hrt_callout)&ScheduledWorkItem::schedule_trampoline, this);
}

/**
 * @brief 取消当前已注册的任何定时任务（延迟、周期、绝对时间）
 *
 * 安全取消，防止回调被意外触发。对象析构前必须调用一次。
 */
void ScheduledWorkItem::ScheduleClear()
{
	// first clear any scheduled hrt call, then remove the item from the runnable queue
	hrt_cancel(&_call);
	WorkItem::ScheduleClear();
}

void ScheduledWorkItem::print_run_status()
{
	if (_call.period > 0) {
		PX4_INFO_RAW("%-29s %8.1f Hz %12.0f us (%" PRId64 " us)\n", _item_name, (double)average_rate(),
			     (double)average_interval(), _call.period);

	} else {
		WorkItem::print_run_status();
	}
}

} // namespace px4
