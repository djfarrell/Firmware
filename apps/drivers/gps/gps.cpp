/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on UART6
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>

#include <drivers/drv_gps.h>

#include <uORB/topics/vehicle_gps_position.h>

#include "ubx.h"

#define SEND_BUFFER_LENGTH 100
#define TIMEOUT 1000000 //1s

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



class GPS : public device::CDev
{
public:
	GPS();
	~GPS();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	int					_task_should_exit;
	int					_serial_fd;		///< serial interface to GPS
	int					_baudrate;
	uint8_t				_send_buffer[SEND_BUFFER_LENGTH];
	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	volatile int		_task;		///< worker task

	bool				_config_needed;
	bool				_healthy;
	gps_driver_mode_t	_mode;
	unsigned 			_messages_received;

	GPS_Helper*			_Helper;

	struct vehicle_gps_position_s	*_reports;

	orb_advert_t		_gps_topic;

	void			recv();

	void			config();

	static void		task_main_trampoline(void *arg);


	/**
	 * worker task
	 */
	void			task_main(void);

	/**
	 * Send a reset command to the GPS
	 */
	void			cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);

namespace
{

GPS	*g_dev;

}


GPS::GPS() :
	CDev("gps", GPS_DEVICE_PATH),
	_task_should_exit(false),
	_baudrate(38400),
	_config_needed(true),
	_healthy(false),
	_mode(GPS_DRIVER_MODE_UBX),
	_messages_received(0),
	_Helper(nullptr),
	_reports(nullptr)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = true;
	debug("[gps driver] instantiated");
}

GPS::~GPS()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;
}

int
GPS::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the IO interface task */
	_task = task_create("gps", SCHED_PRIORITY_SLOW_DRIVER, 4096, (main_t)&GPS::task_main_trampoline, nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	if (_gps_topic < 0)
		debug("failed to create GPS object");

	ret = OK;
out:
	return ret;
}

int
GPS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {
	case GPS_CONFIGURE_UBX:
		//TODO: add configure ubx
		break;
	case GPS_CONFIGURE_MTK:
		//TODO: add configure mtk
		break;
	case GPS_CONFIGURE_NMEA:
		//TODO: add configure nmea
		break;
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	return ret;
}

void
GPS::recv()
{
	uint8_t buf[32];
	int count;

	/*
	 * We are here because poll says there is some data, so this
	 * won't block even on a blocking device.  If more bytes are
	 * available, we'll go back to poll() again...
	 */
	count = ::read(_serial_fd, buf, sizeof(buf));

	/* pass received bytes to the packet decoder */
	for (int i = 0; i < count; i++) {
		_messages_received += _Helper->parse(buf[i]);
	}
}

void
GPS::config()
{
	int length = 0;

	switch (_mode) {
	case GPS_DRIVER_MODE_UBX:
		_Helper->configure(_send_buffer, length, SEND_BUFFER_LENGTH);
		break;
	default:
		break;
	}

	if(length != ::write(_serial_fd, _send_buffer, length)) {
		debug("write config failed");
		return;
	}
}

void
GPS::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
GPS::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open("/dev/ttyS3", O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %d", errno);
		goto out;
	}

	/* 38400bps, no parity, one stop bit */
	{
		struct termios t;

		tcgetattr(_serial_fd, &t);
		cfsetspeed(&t, 38400);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(_serial_fd, TCSANOW, &t);
	}

	switch (_mode) {
	case GPS_DRIVER_MODE_UBX:
		_Helper = new UBX();
		break;
	default:
		break;
	}


	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	debug("ready");

	/* lock against the ioctl handler */
	lock();

	/* loop handling received serial bytes */
	while (!_task_should_exit) {

		/* 1st: try ubx */

		/* 2nd: try mtk19 */

		/* 3dr: try mtk16 */

		/* 4th: try nmea */

		/* sleep waiting for data, but no more than 100ms */
		unlock();
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		lock();

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			continue;
		} else if (ret == 0) {
			_healthy = false;
		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				recv();
			}
		}

		if (!_healthy) {
			config();
		}
	}

out:
	debug("exiting");

	/* kill the HX stream */
//	if (_io_stream != nullptr)
//		hx_stream_free(_io_stream);

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

void
GPS::cmd_reset()
{
	_healthy = false;
}

void
GPS::print_info()
{

}

/**
 * Local functions in support of the shell command.
 */
namespace gps
{

GPS	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new GPS;

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GPS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		printf("Could not open device path: %s\n", GPS_DEVICE_PATH);
		goto fail;
	}
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GPS_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace


int
gps_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		gps::start();

	if (!strcmp(argv[1], "stop"))
		gps::stop();
	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		gps::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		gps::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		gps::info();

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'");
}
