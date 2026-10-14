#include "imu_down_sampler/imu_down_sampler.hpp"

#include <lib/mathlib/mathlib.h>

ImuDownSampler::ImuDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
	reset();
}

// integrate imu samples until target dt reached
// assumes that dt of the gyroscope is close to the dt of the accelerometer
// returns true if target dt is reached
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	_delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu_sample_new.delta_ang_dt;

	// accumulate time deltas
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.delta_vel_clipping[0] |= imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] |= imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] |= imu_sample_new.delta_vel_clipping[2];

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	// 翻译：使用四元数累积增量角度数据
	// 	该四元数表示从累积周期开始到结束的旋转
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	_accumulated_samples++;


	// required number of samples accumulated and the total time is at least half of the target
	//  OR total time already exceeds the target
	// 翻译：已累积所需的样本数量且总时长至少达到目标时长的一半，或者总时长已超过目标时长
	if ((_accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s)
	    || (_imu_down_sampled.delta_ang_dt > _target_dt_s)) {

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);
		return true;
	}

	return false;
}

void ImuDownSampler::reset()
{
	_imu_down_sampled = {};
	_delta_angle_accumulated.setIdentity();
	_accumulated_samples = 0;

	// target dt in seconds safely constrained
	// 翻译：已进行安全约束的目标时间间隔（dt，单位：秒）
	// _target_dt_us的值由 EKF2_PREDICT_US 参数通过ImuDownSampler的构造函数传入
	float target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1e-6f;

	// 预测的需要的样本数
	_required_samples = math::max((int)roundf(target_dt_s / _delta_ang_dt_avg), 1);

	// 预测的总目标时间
	_target_dt_s = _required_samples * _delta_ang_dt_avg;

	// minimum delta angle dt (in addition to number of samples)
	_min_dt_s = math::max(_delta_ang_dt_avg * (_required_samples - 1.f), _delta_ang_dt_avg * 0.5f);
}
