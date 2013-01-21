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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_gps.h>

#include <uORB/topics/vehicle_gps_position.h>

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

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	struct work_s		_work;

	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	struct vehicle_gps_position_s	*_reports;

	orb_advert_t		_gps_topic;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Send a reset command to the GPS
	 */
	int			cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);


GPS::GPS() :
	CDev("gps", GPS_DEVICE_PATH),
	_num_reports(0),
	_next_report(0),
	_oldest_report(0),
	_reports(nullptr)
{
	// enable debug() calls
	_debug_enabled = true;
	debug("[gps driver] instantiated");

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

GPS::~GPS()
{
	/* make sure we are truly inactive */
	stop();
}

int
GPS::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_num_reports = 2;
	_reports = new struct vehicle_gps_position_s[_num_reports];

	if (_reports == nullptr)
		goto out;

	_oldest_report = _next_report = 0;

	/* get a publish handle on the baro topic */
	memset(&_reports[0], 0, sizeof(_reports[0]));
	_gps_topic = orb_advertise(ORB_ID(vehicle_gps_position), &_reports[0]);

	if (_gps_topic < 0)
		debug("failed to create sensor_baro object");

	ret = OK;
out:
	return ret;
}

int
GPS::probe()
{

}

ssize_t
GPS::read(struct file *filp, char *buffer, size_t buflen)
{

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
	case SENSORIOCRESET:
		//TODO: add reset
		break;
	}
}

void
GPS::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_oldest_report = _next_report = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&GPS::cycle_trampoline, this, 1);
}

void
GPS::stop()
{
	work_cancel(HPWORK, &_work);
}

void
GPS::cycle_trampoline(void *arg)
{
	GPS *dev = (GPS *)arg;

	dev->cycle();
}

void
GPS::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* reset the collection state machine and try again */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
//		if ((_measure_phase != 0) &&
//			(_measure_ticks > USEC2TICK(MS5611_CONVERSION_INTERVAL))) {
//
//			/* schedule a fresh cycle call when we are ready to measure again */
//			work_queue(HPWORK,
//				   &_work,
//				   (worker_t)&MS5611::cycle_trampoline,
//				   this,
//				   _measure_ticks - USEC2TICK(MS5611_CONVERSION_INTERVAL));
//
//			return;
//		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MS5611::cycle_trampoline,
		   this,
		   USEC2TICK(MS5611_CONVERSION_INTERVAL));

}

int
GPS::measure()
{

}

int
GPS::collect()
{

}

int
GPS::cmd_reset()
{

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
 * Print a little info about the driver.
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
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		gps::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
