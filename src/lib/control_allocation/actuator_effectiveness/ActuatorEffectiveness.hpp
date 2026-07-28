/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectiveness.hpp
 *
 * Interface for Actuator Effectiveness
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>
#include <uORB/topics/control_allocator_status.h>

enum class AllocationMethod {
	NONE = -1,
	// 过饱和时直接裁剪，不做额外补偿
	PSEUDO_INVERSE = 0,
	// 过饱和时优先保证滚转/俯仰 → 偏航 → 油门，逐步投影回可行域
	SEQUENTIAL_DESATURATION = 1,
	AUTO = 2,
};

enum class ActuatorType {
	MOTORS = 0,
	SERVOS,

	COUNT
};

/**
 * @brief
 * @param NO_EXTERNAL_UPDATE 没有外部更新. 默认状态.表示在最近的控制周期中,没有任何需要重新计算执行器有效性的外部因素发生.
 * @param CONFIGURATION_UPDATE 配置更新. 配置参数发生变化.例如,用户通过地面站修改了像推力曲线,混控矩阵的增益,或者更改了飞行器类型等参数.
 * @param MOTOR_ACTIVATION_UPDATE 电机激活更新 电机状态发生变化(电机故障,关闭冗余电机)
 */
enum class EffectivenessUpdateReason {
	NO_EXTERNAL_UPDATE = 0,
	CONFIGURATION_UPDATE = 1, ///< config changes (parameter)
	MOTOR_ACTIVATION_UPDATE = 2, ///< motor failure detected or certain redundant motors are switched off to save energy
};

class ActuatorEffectiveness
{
public:
	ActuatorEffectiveness() = default;
	virtual ~ActuatorEffectiveness() = default;

	static constexpr int NUM_ACTUATORS = 16;
	static constexpr int NUM_AXES = 6;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	static constexpr int MAX_NUM_MATRICES = 2;

	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
	using ActuatorVector = matrix::Vector<float, NUM_ACTUATORS>;
	using ActuatorBitmask = uint32_t;

	static_assert(NUM_ACTUATORS <= 8 * sizeof(ActuatorBitmask),
		      "NUM_ACTUATORS exceeds the number of bits available in the mask type.");

	enum class FlightPhase {
		HOVER_FLIGHT = 0,
		FORWARD_FLIGHT = 1,
		TRANSITION_HF_TO_FF = 2,
		TRANSITION_FF_TO_HF = 3
	};

	struct Configuration {
		/**
		 * Add an actuator to the selected matrix, returning the index, or -1 on error
		 */
		int addActuator(ActuatorType type, const matrix::Vector3f &torque, const matrix::Vector3f &thrust);

		/**
		 * Call this after manually adding N actuators to the selected matrix
		 */
		void actuatorsAdded(ActuatorType type, int count);

		int totalNumActuators() const;

		/// Configured effectiveness matrix. Actuators are expected to be filled in order, motors first, then servos
		// 翻译：配置有效性矩阵。执行器应按顺序填充，首先是电机，然后是伺服系统
		EffectivenessMatrix effectiveness_matrices[MAX_NUM_MATRICES];

		int num_actuators_matrix[MAX_NUM_MATRICES]; ///< current amount, and next actuator index to fill in to effectiveness_matrices
		ActuatorVector trim[MAX_NUM_MATRICES];

		ActuatorVector linearization_point[MAX_NUM_MATRICES];

		int selected_matrix;

		uint8_t matrix_selection_indexes[NUM_ACTUATORS * MAX_NUM_MATRICES];

		int num_actuators[(int)ActuatorType::COUNT];
	};

	/**
	 * Set the current flight phase
	 *
	 * @param Flight phase
	 */
	virtual void setFlightPhase(const FlightPhase &flight_phase)
	{
		_flight_phase = flight_phase;
	}

	/**
	 * Get the number of effectiveness matrices. Must be <= MAX_NUM_MATRICES.
	 * This is expected to stay constant.
	 */
	virtual int numMatrices() const { return 1; }

	/**
	 * Get the desired allocation method(s) for each matrix, if configured as AUTO
	 * 翻译：如果配置为自动，则为每个矩阵获取所需的分配方法
	 */
	virtual void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			allocation_method_out[i] = AllocationMethod::PSEUDO_INVERSE;
		}
	}

	/**
	 * Query if the roll, pitch and yaw columns of the mixing matrix should be normalized
	 * 翻译：查询混合矩阵的滚动、俯仰和偏航列是否应标准化
	 */
	virtual void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			normalize[i] = false;
		}
	}

	/**
	 * Get the control effectiveness matrix if updated
	 *
	 * @return true if updated and matrix is set
	 */
	virtual bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) { return false;}

	/**
	 * Get the current flight phase
	 *
	 * @return Flight phase
	 */
	const FlightPhase &getFlightPhase() const
	{
		return _flight_phase;
	}

	/**
	 * Display name
	 */
	virtual const char *name() const = 0;

	/**
	 * Callback from the control allocation, allowing to manipulate the setpoint.
	 * Used to allocate auxiliary controls to actuators (e.g. flaps and spoilers).
	 *
	 * @param actuator_sp input & output setpoint
	 */
	virtual void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) {}

	/**
	 * Callback from the control allocation, allowing to manipulate the setpoint.
	 * This can be used to e.g. add non-linear or external terms.
	 * It is called after the matrix multiplication and before final clipping.
	 * @param actuator_sp input & output setpoint
	 */
	virtual void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
				    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) {}

	/**
	 * Get a bitmask of motors to be stopped
	 */
	ActuatorBitmask getStoppedMotors() const;

	/**
	 * Fill in the unallocated torque and thrust, customized by effectiveness type.
	 * Can be implemented for every type separately. If not implemented then the effectivenes matrix is used instead.
	 */
	/**
	 * 填写未分配的扭矩和推力，按功效类型定制。
	 * 可以针对每种类型单独实施。 如果未实现，则使用效果矩阵代替。
	 */
	virtual void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) {}

	/**
	 * Override the collective tilt setpoint that would normally come from
	 * tiltrotor_extra_controls. Base implementation is a no-op.
	 *
	 * @param do_override When true, use @p collective_tilt instead of the uORB value.
	 * @param collective_tilt Normalised setpoint in [0, 1]. 0: vertical, 1: horizontal.
	 */
	virtual void overrideCollectiveTilt(bool /*do_override*/, float /*collective_tilt*/) {}

	/**
	 * Record which components of the thrust setpoint are NaN, so that motors in that direction are stopped.
	 *
	 * @param thrust_sp The thrust setpoint as received by the allocator (instance zero - multicopter only)
	 */
	void stopMotorsBasedOnThrustSetpoint(const matrix::Vector3f &thrust_sp)
	{
		_longitudinal_motors_stopped_by_thrust = !PX4_ISFINITE(thrust_sp(0));
		_lateral_motors_stopped_by_thrust = !PX4_ISFINITE(thrust_sp(1));
		_vertical_motors_stopped_by_thrust = !PX4_ISFINITE(thrust_sp(2));
	}

protected:

	struct MotorDirectionBitmasks {
		ActuatorBitmask longitudinal{};
		ActuatorBitmask lateral{};
		ActuatorBitmask vertical{};
	} _motor_direction_bitmasks;

	FlightPhase _flight_phase{FlightPhase::HOVER_FLIGHT};
	ActuatorBitmask _stopped_motors_mask_due_to_flight_phase{};

	bool _longitudinal_motors_stopped_by_thrust{false};
	bool _vertical_motors_stopped_by_thrust{false};
	bool _lateral_motors_stopped_by_thrust{false};

};
