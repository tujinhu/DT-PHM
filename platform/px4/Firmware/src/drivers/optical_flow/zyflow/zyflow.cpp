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

/**
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 *
 * Driver for the ZYFlow-3901U optical flow sensor
 */

#include <termios.h>
#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <conversion/rotation.h>
#include <drivers/device/device.h>
#include <drivers/device/Device.hpp>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include "zyflow_parser.h"

class Zyflow : public px4::ScheduledWorkItem
{
public:
	Zyflow(const char *port);
	virtual ~Zyflow();

	virtual int init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

private:
	char                     _port[20];
	Rotation            	 _rotation;
	int                      _cycle_interval;
	int                      _fd;
	char                     _linebuf[5];
	unsigned                 _linebuf_index;
	ZYFLOW_PARSE_STATE    _parse_state;

	hrt_abstime              _last_read;
	hrt_abstime              _prev_frame_time;

	optical_flow_s           _report;
	orb_advert_t             _optical_flow_pub;
	uORB::PublicationMulti<distance_sensor_s>	_distance_sensor_topic{ORB_ID(distance_sensor)};

	PX4Rangefinder	_px4_rangefinder;
	perf_counter_t           _sample_perf;
	perf_counter_t           _comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*/
	void                start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void                stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void                Run() override;
	int             collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int zyflow_main(int argc, char *argv[]);

Zyflow::Zyflow(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_rotation(Rotation(0)),
	_cycle_interval(2000),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(ZYFLOW_PARSE_STATE0_UNSYNC),
	_last_read(0),
	_prev_frame_time(0),
	_report(),
	_optical_flow_pub(nullptr),
	_px4_rangefinder(0, distance_sensor_s::ROTATION_DOWNWARD_FACING),
	_sample_perf(perf_alloc(PC_ELAPSED, "Zyflow_read")),
	_comms_errors(perf_alloc(PC_COUNT, "Zyflow_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L1X);

	_px4_rangefinder.set_max_distance(4.0);
	_px4_rangefinder.set_min_distance(0.0);
	_px4_rangefinder.set_fov(math::radians(25.f));
}

Zyflow::~Zyflow()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
Zyflow::init()
{
	int ret = PX4_OK;

	do { /* create a scope to handle exit conditions using break */

		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		/* Baudrate 19200, 8 bits, no parity, 1 stop bit */
		unsigned speed = B19200;

		struct termios uart_config;

		int termios_state;

		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = PX4_ERROR;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;         /* 8-bit characters */
		uart_config.c_cflag &= ~PARENB;     /* no parity bit */
		uart_config.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
		uart_config.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

		/* setup for non-canonical mode */
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		/* fetch bytes as they become available */
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: flow fd");
			ret = PX4_ERROR;
			break;
		}

		/* get yaw rotation from sensor frame to body frame */
		param_t rot = param_find("SENS_FLOW_ROT");

		if (rot != PARAM_INVALID) {
			int32_t val = 0;
			param_get(rot, &val);

			_rotation = Rotation(val);
		}

		/* Initialise report structure */
		/* No gyro on this board */
		_report.gyro_x_rate_integral = NAN;
		_report.gyro_y_rate_integral = NAN;
		_report.gyro_z_rate_integral = NAN;

		/* Conservative specs according to datasheet */
		_report.max_flow_rate = 5.0f;           // Datasheet: 7.4 rad/s
		_report.min_ground_distance = 0.1f;     // Datasheet: 80mm
		_report.max_ground_distance = 30.0f;    // Datasheet: infinity

		/* Integrated flow is sent at 66fps */
		_report.frame_count_since_last_readout = 1;
		_report.integration_timespan = 10526;	// microseconds

		/* Get a publish handle on the optical flow topic */
		_optical_flow_pub = orb_advertise(ORB_ID(optical_flow), &_report);

		if (_optical_flow_pub == nullptr) {
			PX4_ERR("Failed to create optical_flow object");
			ret = PX4_ERROR;
			break;
		}

	} while (0);

	/* Close the fd */
	::close(_fd);
	_fd = -1;

	/* Start measurement */
	start();

	return ret;
}

int
Zyflow::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is the packet size */
	char readbuf[10];
	const unsigned readlen = 9;

	int ret = 0;

	/* Check the number of bytes available in the buffer*/
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		return -EAGAIN;
	}

	bool valid = false;

	do {
		/* Read from UART buffer) */
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			/* only throw an error if we time out */
			if (read_elapsed > (_cycle_interval * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		/* Parse each byte of read buffer */
		for (int i = 0; i < ret; i++) {
			valid |= zyflow_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &_report);
		}

		/* Publish most recent valid measurement */
		if (valid) {

			_report.timestamp = hrt_absolute_time();
			//uint32_t dt = _report.timestamp-_prev_frame_time;
			//_prev_frame_time  = _report.timestamp;
			//_report.timestamp = _report.timestamp;
			//_report.integration_timespan = dt;
			/* Rotate measurements from sensor frame to body frame */
			float zeroval = 0.0f;
			rotate_3f(_rotation, _report.pixel_flow_x_integral, _report.pixel_flow_y_integral, zeroval);

			orb_publish(ORB_ID(optical_flow), _optical_flow_pub, &_report);
			/* publish to the distance_sensor topic as well */
			int8_t signal_quality = -1;
			_px4_rangefinder.update(_report.timestamp, _report.ground_distance_m, signal_quality);

		}

		/* Bytes left to parse */
		bytes_available -= ret;

	} while (bytes_available > 0);

	/* no valid measurement after parsing all available bytes, or partial packet parsed */
	if (!valid || _parse_state != ZYFLOW_PARSE_STATE11_FOOTER) {
		return -EAGAIN;
	}

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
Zyflow::start()
{
	ScheduleNow();
}

void
Zyflow::stop()
{
	ScheduleClear();
}

void
Zyflow::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);
	}

	if (collect() == -EAGAIN) {
		/* Reschedule earlier to grab the missing bits, time to transmit 9 bytes @ 19200 bps */
		ScheduleDelayed(520 * 9);
		return;
	}

	ScheduleDelayed(_cycle_interval);
}

void
Zyflow::print_info()
{
	PX4_INFO("Using port '%s'", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/**
 * Local functions in support of the shell command.
 */
namespace zyflow
{

Zyflow   *g_dev;

int start(const char *port);
int stop();
int info();
void usage();

/**
 * Start the driver.
 */
int
start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return 1;
	}

	/* create the driver */
	g_dev = new Zyflow(port);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	return 0;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return 1;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	g_dev->print_info();

	return 0;
}

/**
 * Print a little info on how to use the driver.
 */
void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the ZYFlow-3901U optical flow sensor.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFLOW_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/pmw3901.html#ZY-ZYflow-3901u

### Examples

Attempt to start driver on a specified serial device.
$ ZYflow start -d /dev/ttyS1
Stop driver
$ ZYflow stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("zyflow", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("optical_flow");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");
}

} // namespace

int
zyflow_main(int argc, char *argv[])
{
    int ch;
    const char *device_path = "";
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'd':
            device_path = myoptarg;
            break;

        default:
            PX4_WARN("Unknown option!");
            return -1;
        }
    }

    if (myoptind >= argc) {
        goto out_error;
    }

    /*
     * Start/load the driver.
     */
    if (!strcmp(argv[myoptind], "start")) {
        if (strcmp(device_path, "") != 0) {
            return zyflow::start(device_path);

        } else {
            PX4_WARN("Please specify device path!");
            zyflow::usage();
            return -1;
        }
    }

    /*
     * Stop the driver
     */
    if (!strcmp(argv[myoptind], "stop")) {
        return zyflow::stop();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
        zyflow::info();
        return 0;
    }

out_error:
    PX4_ERR("unrecognized command");
    zyflow::usage();
    return -1;
}
