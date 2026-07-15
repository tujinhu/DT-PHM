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
 * Driver for the nloop uwb location sensor
 */

#include <termios.h>
#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <conversion/rotation.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/uORB.h>
#include "nloopuwb_parser.h"
#define NLOOPUWB_MEASURE_INTERVAL 5000 /*   5 ms */

class NLoopUwb : public px4::ScheduledWorkItem
{
public:
	NLoopUwb(const char *port);
	virtual ~NLoopUwb();

	virtual int init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

private:
	char                     _port[20];
	Rotation            	 _rotation;
	float                    _heading_offset;
	int                      _cycle_interval;
	int                      _fd;
	uint8_t                  _linebuf[256];
	unsigned                 _linebuf_index;
	NLOOPUWB_PARSE_STATE    _parse_state;

	hrt_abstime              _last_read;
	hrt_abstime              _prev_frame_time;

	uint32_t                 _frame_counts;


	vehicle_odometry_s           _report;
	orb_advert_t             _vehicle_odometry_pub;

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
	int publishreport(struct nlinkframe *pframe);



};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int nloopuwb_main(int argc, char *argv[]);

NLoopUwb::NLoopUwb(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_rotation(Rotation(0)),
	_heading_offset(0),
	_cycle_interval(2000),
	_fd(-1),
	_linebuf_index(0),
	_parse_state(NLOOPUWB_PARSE_STATE0_UNSYNC),
	_last_read(0),
	_prev_frame_time(0),
	_frame_counts(0),
	_report(),
	_vehicle_odometry_pub(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "uloopuwb_read")),
	_comms_errors(perf_alloc(PC_COUNT, "uloopuwb_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

NLoopUwb::~NLoopUwb()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
NLoopUwb::init()
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
		unsigned speed = B115200;

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
		param_t rot = param_find("GPS_YAW_OFFSET");

		if (rot != PARAM_INVALID) {
			float val = 0;
			param_get(rot, &val);

			_heading_offset = val;
			PX4_INFO("rotation is %f",(double)_heading_offset);
		}

		/* Initialise report structure */
		/* No gyro on this board */
		//_report.gyro_x_rate_integral = NAN;
		//_report.gyro_y_rate_integral = NAN;
		//_report.gyro_z_rate_integral = NAN;

		/* Conservative specs according to datasheet */
		//_report.max_flow_rate = 5.0f;           // Datasheet: 7.4 rad/s
		//_report.min_ground_distance = 0.1f;     // Datasheet: 80mm
		//_report.max_ground_distance = 30.0f;    // Datasheet: infinity

		/* Integrated flow is sent at 66fps */
		//_report.frame_count_since_last_readout = 1;
		//_report.integration_timespan = 10526;	// microseconds

		/* Get a publish handle on the optical flow topic */
		_vehicle_odometry_pub = orb_advertise(ORB_ID(vehicle_visual_odometry), &_report);
		if (_vehicle_odometry_pub == nullptr) {
			PX4_ERR("Failed to create vehicle_odometry object");
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
NLoopUwb::collect()
{
	perf_begin(_sample_perf);
	/* clear buffer if last read was too long ago */
	//int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	uint8_t readbuf[256] {};
	unsigned readlen = sizeof(readbuf) - 1;

	int ret = 0;
	struct nlinkframe dframe;
	memset(&dframe,0,sizeof(nlinkframe));

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
	//PX4_ERR("uwb uart bytes_available,%d",bytes_available);

	if (!bytes_available) {
		return -EAGAIN;
	}
	do {
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return -EAGAIN;

		}
		_last_read = hrt_absolute_time();
		//PX4_ERR("r %d bytes",ret);
		for (int i = 0; i < ret; i++) {
			//PX4_ERR("0x%x",readbuf[i]);

			if(nlinkuwb_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &dframe)==0)
			{
				_frame_counts++;
				nlinkuwb_parse_msgbuf(_linebuf, &dframe);
				// PX4_INFO("x=%d,y=%d,z=%d,parse success!",dframe.pos_x,dframe.pos_y,dframe.pos_z);
			 	publishreport(&dframe);
			}

		}
		bytes_available -= ret;

	} while (bytes_available>0);

	perf_end(_sample_perf);
	return PX4_OK;
}
int NLoopUwb::publishreport(struct nlinkframe *pframe)
{
	uint64_t timestamp = hrt_absolute_time();
	//_previous_collect_timestamp = timestamp;


	struct vehicle_odometry_s visual_odom{};

	visual_odom.timestamp = timestamp;
	visual_odom.timestamp_sample = timestamp;

	/*
	θ anti-clockwise is +
	x'=x·cos(θ)+y·sin(θ)
	y'=y·cos(θ)-x·sin(θ)
	*/
	float cos_th = cosf((float)_heading_offset);
	float sin_th = sinf((float)_heading_offset);
	float pox = pframe->pos_x/1000.0;
	float poy = pframe->pos_y/1000.0;
	float x_h = pox * cos_th + poy * sin_th;
	float y_h = poy * cos_th - pox * sin_th;
	visual_odom.x = x_h;
	visual_odom.y = y_h;
	visual_odom.z = -pframe->pos_z/1000.0;

	visual_odom.velocity_covariance[0] = NAN;
	visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_X_VARIANCE] =  pframe->eop_x/100.0;
	visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_Y_VARIANCE] =  pframe->eop_y/100.0;
	visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_Z_VARIANCE] =  pframe->eop_z/100.0;


	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	visual_odom.local_frame = visual_odom.LOCAL_FRAME_NED;

	visual_odom.vx = pframe->vel_x/10000.0;
	visual_odom.vy = pframe->vel_y/10000.0;
	visual_odom.vz = pframe->vel_z/10000.0;

	/* publish it */
	//PX4_INFO("uwb publish:px:%f,py:%f,pz:%f,vx:%f,vy:%f,vz:%f,",(double)visual_odom.x,(double)visual_odom.y,(double)visual_odom.z,(double)visual_odom.vx,(double)visual_odom.vy,(double)visual_odom.vz);
	orb_publish(ORB_ID(vehicle_visual_odometry), _vehicle_odometry_pub, &visual_odom);

	// print()
	//_reports->force(&report);

	/* notify anyone waiting for data */
	//poll_notify(POLLIN);
	//perf_end(_sample_perf);

	return PX4_OK;
}
void
NLoopUwb::start()
{
	ScheduleNow();
}

void
NLoopUwb::stop()
{
	ScheduleClear();
}

void
NLoopUwb::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY);
	}

	if (collect() == -EAGAIN) {
		/* Reschedule earlier to grab the missing bits, time to transmit 9 bytes @ 19200 bps */
		ScheduleDelayed(2000);
		return;
	}

	ScheduleDelayed(NLOOPUWB_MEASURE_INTERVAL);
}

void
NLoopUwb::print_info()
{
	PX4_INFO("Using port '%s'", _port);
	PX4_INFO("published reports '%d'", _frame_counts);
	// perf_print_counter(_sample_perf);
	// perf_print_counter(_comms_errors);
}

/**
 * Local functions in support of the shell command.
 */
namespace nloopuwb
{

NLoopUwb   *g_dev;

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
	g_dev = new NLoopUwb(port);

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

Serial bus driver for the NoopLoop UWB sensor.

Most boards are configured to enable/start the driver on a specified UART using the SENS_UWB_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/pmw3901.html#thone-thoneflow-3901u

### Examples

Attempt to start driver on a specified serial device.
$ nloopuwb start -d /dev/ttyS1
Stop driver
$ nloopuwb stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("nloopuwb", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("optical_flow");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");
}

} // namespace

int
nloopuwb_main(int argc, char *argv[])
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
            return nloopuwb::start(device_path);

        } else {
            PX4_WARN("Please specify device path!");
            nloopuwb::usage();
            return -1;
        }
    }

    /*
     * Stop the driver
     */
    if (!strcmp(argv[myoptind], "stop")) {
        return nloopuwb::stop();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
        nloopuwb::info();
        return 0;
    }

out_error:
    PX4_ERR("unrecognized command");
    nloopuwb::usage();
    return -1;
}
