/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file airspeed.cpp
 * Airspeed estimation
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

 /**
  * @param CAS  | Calibrated Airspeed             | 校准空速             | 空速管测出来的原始动压，经过仪表误差校准（Instrument Error Correction）后的值 | 空速管直接输出   |
  * @param TAS  | True Airspeed                   | 真空速 / 真速        | 飞机相对于空气的真实速度（不受气压、温度、密度影响）     | 飞控真正用于导航、风速估计、失速保护的那个速度 |
  * @param IAS  | Indicated Airspeed              | 指示空速             | 仪表上直接读出来的（还没校准仪表误差）                   | 基本不用         |
  * @param EAS  | Equivalent Airspeed             | 等效空速             | 考虑空气可压缩性后的中间值（>0.7Ma 才重要）              | 高亚音速才关心   |
  * @param GS   | Ground Speed                    | 地速                 | 相对于地面的速度 = TAS ± 风                              | GPS 给出         |
  */
#include "airspeed.h"

#include <px4_platform_common/defines.h>
#include <lib/atmosphere/atmosphere.h>

using atmosphere::getDensityFromPressureAndTemp;
using atmosphere::kAirDensitySeaLevelStandardAtmos;

float calc_IAS_corrected(enum AIRSPEED_COMPENSATION_MODEL pmodel, enum AIRSPEED_SENSOR_MODEL smodel,
			 float tube_len, float tube_dia_mm, float differential_pressure, float pressure_ambient, float temperature_celsius)
{
	if (!PX4_ISFINITE(temperature_celsius)) {
		temperature_celsius = 15.f; // ICAO Standard Atmosphere 15 degrees Celsius
	}

	// air density in kg/m3
	// 翻译：空气密度（kg/m³）
	const float rho_air = getDensityFromPressureAndTemp(pressure_ambient, temperature_celsius);

	const float dp = fabsf(differential_pressure);
	float dp_tot = dp;

	float dv = 0.0f;

	switch (smodel) {

	case AIRSPEED_SENSOR_MODEL_MEMBRANE: {
			// do nothing
		}
		break;

	case AIRSPEED_SENSOR_MODEL_SDP3X: {
			// assumes a metal pitot tube with round tip as here: https://drotek.com/shop/2986-large_default/sdp3x-airspeed-sensor-kit-sdp31.jpg
			// and tubing as provided by px4/drotek (1.5 mm diameter)
			// The tube_len represents the length of the tubes connecting the pitot to the sensor.
			// 假设使用带圆形尖端的金属皮托管，如下图所示：https://drotek.com/shop/2986-large_default/sdp3x-airspeed-sensor-kit-sdp31.jpg
			// 以及 px4/drotek 提供的管路（直径 1.5 毫米）
			// tube_len 表示连接皮托管和传感器的管路长度。
			switch (pmodel) {
			case AIRSPEED_COMPENSATION_MODEL_PITOT:
			case AIRSPEED_COMPENSATION_MODEL_NO_PITOT: {
					const float dp_corr = dp * 96600.0f / pressure_ambient;
					// flow through sensor
					float flow_SDP33 = (300.805f - 300.878f / (0.00344205f * powf(dp_corr, 0.68698f) + 1.0f)) * 1.29f / rho_air;

					// for too small readings the compensation might result in a negative flow which causes numerical issues
					// 翻译：如果流量太小，补偿可能会导致负流量，从而引起数值问题
					if (flow_SDP33 < 0.0f) {
						flow_SDP33 = 0.0f;
					}

					float dp_pitot = 0.0f;

					switch (pmodel) {
					case AIRSPEED_COMPENSATION_MODEL_PITOT:
						dp_pitot = (0.0032f * flow_SDP33 * flow_SDP33 + 0.0123f * flow_SDP33 + 1.0f) * 1.29f / rho_air;
						break;

					default:
						// do nothing
						break;
					}

					// pressure drop through tube
					// 翻译：通过管路的压力损失
					const float dp_tube = (flow_SDP33 * 0.674f) / 450.0f * tube_len * rho_air / 1.29f;

					// speed at pitot-tube tip due to flow through sensor
					// 翻译：通过传感器的流量
					dv = 0.125f * flow_SDP33;

					// sum of all pressure drops
					// 翻译：所有压力损失的总和
					dp_tot = dp_corr + dp_tube + dp_pitot;
				}
				break;

			case AIRSPEED_COMPENSATION_TUBE_PRESSURE_LOSS: {
					// Pressure loss compensation as defined in https://goo.gl/UHV1Vv.
					// tube_dia_mm: Diameter in mm of the pitot and tubes, must have the same diameter.
					// tube_len: Length of the tubes connecting the pitot to the sensor and the static + dynamic port length of the pitot.
					// 翻译注释：管路压力损失补偿，根据https://goo.gl/UHV1Vv定义。
					// 翻译：tube_dia_mm：皮托管和管子的直径（单位：毫米），必须相同。
					// 翻译： tube_len：连接皮托管和传感器的管子长度，以及皮托管的静压口和动压口总长度。

					// check if the tube diameter and dp is nonzero to avoid division by 0
					// 翻译：检查管子直径和dp是否为零，以避免除以零。
					if ((tube_dia_mm > 0.0f) && (dp > 0.0f)) {
						const float d_tubePow4 = powf(tube_dia_mm * 1e-3f, 4);
						const float denominator = M_PI_F * d_tubePow4 * rho_air * dp;

						// avoid division by 0
						// 翻译：避免除以零。
						float eps = 0.0f;

						if (fabsf(denominator) > 1e-32f) {
							const float viscosity = (18.205f + 0.0484f * (temperature_celsius - 20.0f)) * 1e-6f;

							// 4.79 * 1e-7 -> mass flow through sensor
							// 翻译：4.79 * 1e-7 -> 传感器通过的流量
							// 59.5 -> dp sensor constant where linear and quadratic contribution to dp vs flow is equal
							// 翻译：59.5 -> dp 传感器常数，其中线性和二次方对 dp 与流量关系的贡献相等。
							eps = -64.0f * tube_len * viscosity * 4.79f * 1e-7f * (sqrtf(1.0f + 8.0f * dp / 59.3319f) - 1.0f) / denominator;
						}

						// range check on eps
						// 翻译：eps 范围检查
						if (fabsf(eps) >= 1.0f) {
							eps = 0.0f;
						}

						// pressure correction
						// 翻译：压力校正
						dp_tot = dp / (1.0f + eps);
					}
				}
				break;

			default: {
					// do nothing
				}
				break;
			}

		}
		break;

	default: {
			// do nothing
		}
		break;
	}

	// computed airspeed without correction for inflow-speed at tip of pitot-tube
	// 翻译：计算空速，未对皮托管尖端的流入速度进行修正。
	const float airspeed_uncorrected = sqrtf(2.0f * dp_tot / kAirDensitySeaLevelStandardAtmos);

	// corrected airspeed
	// 翻译：修正后的空速
	const float airspeed_corrected = airspeed_uncorrected + dv;

	// return result with correct sign
	// 翻译：返回结果，正确符号
	return (differential_pressure > 0.0f) ? airspeed_corrected : -airspeed_corrected;
}

/**
 * @brief 计算指示空速（IAS）。
 * @param differential_pressure 差压
 * @return IAS（米/秒）
 */
float calc_IAS(float differential_pressure)
{
	if (differential_pressure > 0.0f) {
		return sqrtf((2.0f * differential_pressure) / kAirDensitySeaLevelStandardAtmos);

	} else {
		return -sqrtf((2.0f * fabsf(differential_pressure)) / kAirDensitySeaLevelStandardAtmos);
	}

}

/**
 * @brief 根据校准空速 (CAS) 计算真实空速 (TAS)。
 * @param speed_calibrated 当前校准空速
 * @param pressure_ambient 管道/飞机侧面的环境压力
 * @param temperature_celsius 气温（摄氏度）
 * @return TAS（米/秒）
 */
float calc_TAS_from_CAS(float speed_calibrated, float pressure_ambient, float temperature_celsius)
{
	if (!PX4_ISFINITE(temperature_celsius)) {
		temperature_celsius = 15.f; // ICAO Standard Atmosphere 15 degrees Celsius
	}

	return speed_calibrated * sqrtf(kAirDensitySeaLevelStandardAtmos / getDensityFromPressureAndTemp(pressure_ambient,
					temperature_celsius));
}

/**
 * @brief 根据指示空速 (IAS) 计算校准空速 (CAS)。
 * @param speed_indicated 当前指示空速
 * @param scale 从 IAS 到 CAS 的比例尺（考虑仪表和皮托管位置误差）
 * @return CAS 值（单位：米/秒）
 */
float calc_CAS_from_IAS(float speed_indicated, float scale)
{
	return speed_indicated * scale;
}

/**
* @brief 直接计算真实空速 (TAS)。
* 	此处假设没有仪表或皮托管位置误差 (IAS = CAS)。
* 	注意：由于风的影响，实际空速并非地速。
* @param total_pressure 皮托管/普朗特管内的压力
* @param static_pressure 管壁/飞机侧面的压力
* @param temperature_celsius 气温（摄氏度）
* @return 实际空速（米/秒）
*/
float calc_TAS(float total_pressure, float static_pressure, float temperature_celsius)
{
	float density = getDensityFromPressureAndTemp(static_pressure, temperature_celsius);

	if (density < 0.0001f || !PX4_ISFINITE(density)) {
		density = kAirDensitySeaLevelStandardAtmos;
	}

	float pressure_difference = total_pressure - static_pressure;

	if (pressure_difference > 0) {
		return sqrtf((2.0f * (pressure_difference)) / density);

	} else {
		return -sqrtf((2.0f * fabsf(pressure_difference)) / density);
	}
}

/**
* @brief 根据真实空速和空气密度计算校准后的空速
* @param speed_true 真实空速 [m/s]
* @param air_density 空气密度 [kg/m3]
* @return 校准后的空速 [m/s]
*/
float calc_calibrated_from_true_airspeed(float speed_true, float air_density)
{
	return speed_true * sqrtf(air_density / kAirDensitySeaLevelStandardAtmos);
}

/**
 * @brief 根据校准空速和空气密度计算真实空速
 * @param speed_calibrated 校准空速 [m/s]
 * @param air_density 空气密度 [kg/m3]
 * @return 真实空速 [m/s]
 */
float calc_true_from_calibrated_airspeed(float speed_calibrated, float air_density)
{
	return speed_calibrated * sqrtf(kAirDensitySeaLevelStandardAtmos / air_density);
}
