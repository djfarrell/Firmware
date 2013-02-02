/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

#define NUMBER_OF_BAUDRATES 4
#define CONFIG_TIMEOUT 2000000

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

	bool				_task_should_exit;						///< flag to make the main worker task exit
	int					_serial_fd;								///< serial interface to GPS
	unsigned			_baudrate;								///< current baudrate
	unsigned			_baudrates_to_try[NUMBER_OF_BAUDRATES]; ///< try different baudrates that GPS could be set to
	volatile int		_task;									///< worker task
	bool				_config_needed;							///< flag to signal that configuration of GPS is needed
	bool 				_baudrate_changed;						///< flag to signal that the baudrate with the GPS has changed
	bool				_mode_changed;							///< flag that the GPS mode has changed
	gps_driver_mode_t	_mode;									///< current mode
	GPS_Helper			*_Helper;								///< Class for either UBX, MTK or NMEA helper
	struct vehicle_gps_position_s _report;						///< uORB topic for gps position
	orb_advert_t		_report_pub;							///< uORB pub for gps position


	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void				config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	void				set_baudrate(void);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

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
	_baudrates_to_try(GPS_BAUDRATES_TO_TRY),
	_config_needed(true),
	_baudrate_changed(false),
	_mode_changed(true),
	_mode(GPS_DRIVER_MODE_UBX),
	_Helper(nullptr)
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
		if (_mode != GPS_DRIVER_MODE_UBX) {
			_mode = GPS_DRIVER_MODE_UBX;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_MTK19:
		if (_mode != GPS_DRIVER_MODE_MTK19) {
			_mode = GPS_DRIVER_MODE_MTK19;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_MTK16:
		if (_mode != GPS_DRIVER_MODE_MTK16) {
			_mode = GPS_DRIVER_MODE_MTK16;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_NMEA:
		if (_mode != GPS_DRIVER_MODE_NMEA) {
			_mode = GPS_DRIVER_MODE_NMEA;
			_mode_changed = true;
		}
		break;
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	return ret;
}

void
GPS::config()
{
	int length = 0;
	uint8_t	send_buffer[SEND_BUFFER_LENGTH];

	_Helper->configure(_config_needed, _baudrate_changed, _baudrate, send_buffer, length, SEND_BUFFER_LENGTH);

	/* The config message is sent sent at the old baudrate */
	if (length > 0) {
		if (length != ::write(_serial_fd, send_buffer, length)) {
			debug("write config failed");
			return;
		}
	}

	/* Sometimes the baudrate needs to be changed, for instance UBX with factory settings are changed
	 * from 9600 to 38400
	 */
	if (_baudrate_changed) {
		set_baudrate();
		_baudrate_changed = false;
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

	/* Prepare the gps position topic */
	_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report);
	memset(&_report, 0, sizeof(_report));

	/* Buffer to read from the serial port */
	uint8_t buf[32];

	/* Poll descriptor */
	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;
	debug("ready");

	uint64_t time_before_configuration = hrt_absolute_time();

	/* Start with the first baudrate in the list */
	int baud_i = 0;


	/* Open the serial port */
	_serial_fd = ::open("/dev/ttyS3", O_RDWR); //TODO make the device dynamic depending on startup parameters
	if (_serial_fd < 0) {
		log("failed to open serial port: %d", errno);
		goto out;
	}

	/* Set the first baudrate */
	_baudrate = _baudrates_to_try[baud_i];
	set_baudrate();

	/* Lock against the ioctl handler */
	lock(); // XXX do we need these locks?


	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_mode_changed) {

			if (_Helper != nullptr) {
				delete(_Helper);
				_Helper = nullptr; // XXX is this needed?
			}

			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_Helper = new UBX();
				break;
			case GPS_DRIVER_MODE_MTK19:
				//_Helper = new MTK19();
				break;
			case GPS_DRIVER_MODE_MTK16:
				//_Helper = new MTK16();
				break;
			case GPS_DRIVER_MODE_NMEA:
				//_Helper = new NMEA();
				break;
			default:
				break;
			}
			_mode_changed = false;
		}

		/* If a configuration does not finish in the config timeout, change the baudrate */
		if (_config_needed && time_before_configuration + CONFIG_TIMEOUT < hrt_absolute_time()) {
			baud_i = (baud_i+1)%NUMBER_OF_BAUDRATES;
			_baudrate = _baudrates_to_try[baud_i];
			set_baudrate();
			time_before_configuration = hrt_absolute_time();
		}

		/* Hack: during configuration, the timeout should be small, so that we can send config messages in between parsing,
		 * but during normal operation, it should never timeout because updates should arrive with 5Hz */
		int poll_timeout;
		if (_config_needed) {
			poll_timeout = 50;
		} else {
			poll_timeout = 250;
		}

		unlock();
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), poll_timeout);
		lock();

		if (ret < 0) {
			/* this would be bad... */
			log("poll error %d", errno);
		} else if (ret == 0) {
			if (!_config_needed) {
				_config_needed = true;
				debug("Lost GPS module: timeout");
			}
			config();
		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				int count;

				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_serial_fd, buf, sizeof(buf));

				/* Pass received bytes to the packet decoder (one byte at a time) */
				int j;
				int ret_parse = 0;
				for (j = 0; j < count; j++) {
					ret_parse += _Helper->parse(buf[j], &_report);
				}

				if (ret_parse < 0) {
					/* This means something went wrong in the parser, let's reconfigure */
					if (!_config_needed) {
						_config_needed = true;
						debug("Lost GPS module");
					}
					config();
				} else if (ret_parse > 0) {
					/* Looks like we got a valid position update, stop configuring and publish it */
					if (_config_needed) {
						_config_needed = false;
						debug("Found GPS module");
					}
					orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &_report);
				}
				/* else if ret_parse == 0, just keep parsing */
			}
		}
	}
out:
	debug("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

void
GPS::set_baudrate()
{
	usleep(100000); // XXX The wait seems to be necessairy, more testing needed

	/* no parity, one stop bit */
	struct termios t;
	if (tcgetattr(_serial_fd, &t) != 0)
		debug("Baudrate change failed (tcgetattr fail)");
	if (cfsetspeed(&t, _baudrate) != 0)
		debug("Baudrate change failed (cfsetspeed fail)");
	t.c_cflag &= ~(CSTOPB | PARENB);
	if (tcsetattr(_serial_fd, TCSANOW, &t) != 0)
		debug("Baudrate change failed (tcsetattr_fail)");

	usleep(10000);  // XXX The wait seems to be necessairy, more testing needed

	/* The configuration needs to be reset at this point */
	_Helper->reset();
}

void
GPS::cmd_reset()
{
	 //XXX: What's the right thing to do here? Reconfigure or restart the thread?
}

void
GPS::print_info()
{
	switch (_mode) {
	case GPS_DRIVER_MODE_UBX:
		printf("Protocol: UBX");
		break;
	case GPS_DRIVER_MODE_MTK19:
		printf("Protocol: MTK 1.9");
		break;
	case GPS_DRIVER_MODE_MTK16:
		printf("Protocol: MTK 1.6");
		break;
	case GPS_DRIVER_MODE_NMEA:
		printf("Protocol: NMEA");
		break;
	default:
		break;
	}
	printf(", Baudrate: %d, Status: %s\n", _baudrate, (_config_needed) ? "NOT OK" : "OK");
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

/**
 * Stop the driver.
 */
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
