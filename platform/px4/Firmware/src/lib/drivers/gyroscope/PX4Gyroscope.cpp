/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

using namespace time_literals;
using matrix::Vector3f;

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation) :
	_device_id{device_id},
	_rotation{rotation}
{
	// advertise immediately to keep instance numbering in sync
	_sensor_pub.advertise();

	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
}

PX4Gyroscope::~PX4Gyroscope()
{
	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
}

void PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Gyroscope::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;
	}
}

void PX4Gyroscope::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	sensor_gyro_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;

	_rfly_ctrl_sub.copy(&rflydata);
	if (int(rflydata.modes - 13) == 0)
	{
		if (int(rflydata.flags - 1) == 0)
		{
			report.x = rflydata.controls[0];
			report.y = rflydata.controls[1];
			report.z = rflydata.controls[2];
		}

		if (int(rflydata.flags - 2) == 0)
		{
			if(int(rflydata.controls[0]) == 1){
				report.x = x * _scale + rflydata.controls[3];
			}else{
				report.x = x * _scale;
			}
			if(int(rflydata.controls[1]) == 1){
				report.y = y * _scale + rflydata.controls[4];
			}else{
				report.y = y * _scale;
			}
			if(int(rflydata.controls[2]) == 1){
				report.z = z * _scale + rflydata.controls[5];
			}else{
				report.z = z * _scale;
			}
		}

		if (int(rflydata.flags - 3) == 0)
		{
			// rflydata.controls[3] = mean
			// rflydata.controls[4] = std
			if(int(rflydata.controls[0]) == 1){
				report.x = x * _scale + generate_wgn(rflydata.controls[3],rflydata.controls[4]);
			}else{
				report.x = x * _scale;
			}
			if(int(rflydata.controls[1]) == 1){
				report.y = y * _scale + generate_wgn(rflydata.controls[3],rflydata.controls[4]);
			}else{
				report.y = y * _scale;
			}
			if(int(rflydata.controls[2]) == 1){
				report.z = z * _scale + generate_wgn(rflydata.controls[3],rflydata.controls[4]);
			}else{
				report.z = z * _scale;
			}
		}

		if (int(rflydata.flags - 4) == 0)
		{
			report.x = x * _scale * rflydata.controls[0];
			report.y = y * _scale * rflydata.controls[1];
			report.z = z * _scale * rflydata.controls[2];
		}
	}
	else
	{
		report.x = x * _scale;
		report.y = y * _scale;
		report.z = z * _scale;
	}

	report.samples = 1;
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

void PX4Gyroscope::updateFIFO(sensor_gyro_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	const uint8_t N = sample.samples;

	for (int n = 0; n < N; n++) {
		rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	}

	sample.device_id = _device_id;
	sample.scale = _scale;
	sample.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(sample);


	// trapezoidal integration (equally spaced, scaled by dt later)
	const Vector3f integral{
		(0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)),
		(0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)),
		(0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)),
	};

	_last_sample[0] = sample.x[N - 1];
	_last_sample[1] = sample.y[N - 1];
	_last_sample[2] = sample.z[N - 1];


	const float scale = _scale / N;

	sensor_gyro_s report;

	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;



	_rfly_ctrl_sub.copy(&rflydata);
	if (int(rflydata.modes - 13) == 0)
	{
		if (int(rflydata.flags - 1) == 0)
		{
			if (int(rflydata.flags - 1) == 0)
			{
				if(int(rflydata.controls[0]) == 1){
					report.x = rflydata.controls[3];
				}else{
					report.x = integral(0) * scale;
				}
				if(int(rflydata.controls[1]) == 1){
					report.y = rflydata.controls[4];
				}else{
					report.y = integral(1) * scale;
				}
				if(int(rflydata.controls[2]) == 1){
					report.z = rflydata.controls[5];
				}else{
					report.z = integral(2) * scale;
				}
			}
		}

		if (int(rflydata.flags - 2) == 0)
		{
			report.x = integral(0) * scale + rflydata.controls[0];
			report.y = integral(1) * scale + rflydata.controls[1];
			report.z = integral(2) * scale + rflydata.controls[2];
		}

		if (int(rflydata.flags - 3) == 0)
		{
			// rflydata.controls[3] = mean
			// rflydata.controls[4] = std
			if(int(rflydata.controls[0]) == 1){
				report.x = integral(0) * scale + generate_wgn(rflydata.controls[3], rflydata.controls[4]);
			}else{
				report.x = integral(0) * scale;
			}
			if(int(rflydata.controls[1]) == 1){
				report.y = integral(1) * scale + generate_wgn(rflydata.controls[3], rflydata.controls[4]);
			}else{
				report.y = integral(1) * scale;
			}
			if(int(rflydata.controls[2]) == 1){
				report.z = integral(2) * scale + generate_wgn(rflydata.controls[3], rflydata.controls[4]);
			}else{
				report.z = integral(2) * scale;
			}
		}

		if (int(rflydata.flags - 4) == 0)
		{
			report.x = integral(0) * scale * rflydata.controls[0];
			report.y = integral(1) * scale * rflydata.controls[1];
			report.z = integral(2) * scale * rflydata.controls[2];
		}
	}
	else
	{
		report.x = integral(0) * scale;
		report.y = integral(1) * scale;
		report.z = integral(2) * scale;
	}

	report.samples = N;
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

float PX4Gyroscope::generate_wgn(float mean, float stddev)
{
	// generate white Gaussian noise sample

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	float Y = mean + stddev * X; // 线性变换

	phase = !phase;
	return Y;
}
