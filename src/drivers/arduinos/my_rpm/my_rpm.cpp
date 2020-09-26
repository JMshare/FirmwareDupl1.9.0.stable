/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file my_rpm.cpp
 * @author Juraj Mihalik
 *
 * Driver for the arduino scales connected via I2C.
 * Based on the Sonar mb12xx rangefinder driver in PX4 v 1.9.0
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <containers/Array.hpp>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <mathlib/mathlib.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/my_rpm_topic.h>

#include <board_config.h>

#define I2C_BUFF_SIZE (3 + 2*2 + 2*34)
#define MY_RPM_BASE_DEVICE_PATH	"/dev/rpm"
#define MY_RPM_MAX_SENSORS	12	// Maximum number of sensors on bus

/* Configuration Constants */
#define MY_RPM_BUS_DEFAULT		PX4_I2C_BUS_EXPANSION
#define MY_RPM_BASEADDR 	0x70 /* 7-bit address */
#define MY_RPM_DEVICE_PATH	"/dev/my_rpm"

/* MY_RPM Registers addresses */

#define MY_RPM_TAKE_RANGE_REG	0x51		/* Measure Register */

/* Device limits */
#define MY_RPM_MIN_RPM 	0
#define MY_RPM_MAX_RPM 	99999

#define MY_RPM_CONVERSION_INTERVAL 	50*1000UL /* 50ms for one sensor */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	5*1000UL /* 5ms between each sensor measurement */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MY_RPM : public device::I2C
{
public:
	MY_RPM(uint8_t id = 1,
	       int bus = MY_RPM_BUS_DEFAULT, int address = MY_RPM_BASEADDR);
	virtual 			~MY_RPM();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	uint8_t 			_id;
	uint64_t   			_min_rpm;
	uint64_t  			_max_rpm;
	work_s				_work{};
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_my_rpm_topic_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	px4::Array<uint8_t, MY_RPM_MAX_SENSORS>	addr_ind; 	/* temp rpm i2c address vector */

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max rpm thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MY_RPM_MIN_RPM
	* and MY_RPM_MAX_RPM
	*/
	void				set_minimum_rpm(float min);
	void				set_maximum_rpm(float max);
	float				get_minimum_rpm();
	float				get_maximum_rpm();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int my_rpm_main(int argc, char *argv[]);

MY_RPM::MY_RPM(uint8_t id, int bus, int address) :
	I2C("MY_RPM", MY_RPM_DEVICE_PATH, bus, address, 100000),
	_id(id),
	_min_rpm(MY_RPM_MIN_RPM),
	_max_rpm(MY_RPM_MAX_RPM),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_my_rpm_topic_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "my_rpm_read")),
	_comms_errors(perf_alloc(PC_COUNT, "my_rpm_com_err")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
}

MY_RPM::~MY_RPM()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MY_RPM_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
MY_RPM::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(my_rpm_topic_s));

	_index_counter = MY_RPM_BASEADDR;	/* set temp sonar i2c address to base adress */
	set_device_address(_index_counter);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(MY_RPM_BASE_DEVICE_PATH);


	struct my_rpm_topic_s ds_report;

	_my_rpm_topic_topic = orb_advertise_multi(ORB_ID(my_rpm_topic), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_my_rpm_topic_topic == nullptr) {
		PX4_ERR("failed to create my_rpm_topic object");
	}

	// XXX we should find out why we need to wait 200 ms here
	px4_usleep(200000);

	/* check for connected sensors on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on*/
	for (unsigned counter = 0; counter <= MY_RPM_MAX_SENSORS; counter++) {
		_index_counter = MY_RPM_BASEADDR - counter;	/* set temp rpm i2c address to base adress - counter */
		set_device_address(_index_counter);			/* set I2c port to temp rpm i2c adress */
		int ret2 = measure();

		if (ret2 == 0) { /* sensor is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			PX4_DEBUG("sensor added");
		}
	}

	_index_counter = MY_RPM_BASEADDR;
	set_device_address(_index_counter); /* set i2c port back to base adress for rest of driver */

	/* if only one sonar detected, no special timing is required between firing, so use default */
	if (addr_ind.size() == 1) {
		_cycling_rate = MY_RPM_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = TICKS_BETWEEN_SUCCESIVE_FIRES;
	}

	/* show the connected sensors in terminal */
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		PX4_DEBUG("sensor %d with address %d added", (i + 1), addr_ind[i]);
	}

	PX4_DEBUG("Number of sensors connected: %lu", addr_ind.size());

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int
MY_RPM::probe()
{
	return measure();
}

void
MY_RPM::set_minimum_rpm(float min)
{
	_min_rpm = min;
}

void
MY_RPM::set_maximum_rpm(float max)
{
	_max_rpm = max;
}

float
MY_RPM::get_minimum_rpm()
{
	return _min_rpm;
}

float
MY_RPM::get_maximum_rpm()
{
	return _max_rpm;
}

int
MY_RPM::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_cycling_rate);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_cycling_rate)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
MY_RPM::read(device::file_t *filp, char *buffer, size_t buflen)
{

	unsigned count = buflen / sizeof(struct my_rpm_topic_s);
	struct my_rpm_topic_s *rbuf = reinterpret_cast<struct my_rpm_topic_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(_cycling_rate * 2);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
MY_RPM::measure()
{

	int ret;

	/*
	 * Send the command to begin a measurement.
	 */

	uint8_t cmd = MY_RPM_TAKE_RANGE_REG;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
MY_RPM::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[I2C_BUFF_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], I2C_BUFF_SIZE);

	struct my_rpm_topic_s report;
	report.timestamp = hrt_absolute_time();
	report.rpm = val[0] + val[1]*100 + val[2]*10000;
	report.current1_sign = val[3];
	report.current1 = val[4];
	report.current2_sign = val[5];
	report.current2 = val[6];

	int telem_start = 7;
	report.telem1_1 = val[telem_start+0];
	report.telem1_2 = val[telem_start+1];
	report.telem1_3 = val[telem_start+2];
	report.telem1_4 = val[telem_start+3];
	report.telem1_5 = val[telem_start+4];
	report.telem1_6 = val[telem_start+5];
	report.telem1_7 = val[telem_start+6];
	report.telem1_8 = val[telem_start+7];
	report.telem1_9 = val[telem_start+8];
	report.telem1_10 = val[telem_start+9];
	report.telem1_11 = val[telem_start+10];
	report.telem1_12 = val[telem_start+11];
	report.telem1_13 = val[telem_start+12];
	report.telem1_14 = val[telem_start+13];
	report.telem1_15 = val[telem_start+14];
	report.telem1_16 = val[telem_start+15];
	report.telem1_17 = val[telem_start+16];
	report.telem1_18 = val[telem_start+17];
	report.telem1_19 = val[telem_start+18];
	report.telem1_20 = val[telem_start+19];
	report.telem1_21 = val[telem_start+20];
	report.telem1_22 = val[telem_start+21];
	report.telem1_23 = val[telem_start+22];
	report.telem1_24 = val[telem_start+23];
	report.telem1_25 = val[telem_start+24];
	report.telem1_26 = val[telem_start+25];
	report.telem1_27 = val[telem_start+26];
	report.telem1_28 = val[telem_start+27];
	report.telem1_29 = val[telem_start+28];
	report.telem1_30 = val[telem_start+29];
	report.telem1_31 = val[telem_start+30];
	report.telem1_32 = val[telem_start+31];
	report.telem1_33 = val[telem_start+32];
	report.telem1_34 = val[telem_start+33];

	telem_start = telem_start + 34;
	report.telem2_1 = val[telem_start+0];
	report.telem2_2 = val[telem_start+1];
	report.telem2_3 = val[telem_start+2];
	report.telem2_4 = val[telem_start+3];
	report.telem2_5 = val[telem_start+4];
	report.telem2_6 = val[telem_start+5];
	report.telem2_7 = val[telem_start+6];
	report.telem2_8 = val[telem_start+7];
	report.telem2_9 = val[telem_start+8];
	report.telem2_10 = val[telem_start+9];
	report.telem2_11 = val[telem_start+10];
	report.telem2_12 = val[telem_start+11];
	report.telem2_13 = val[telem_start+12];
	report.telem2_14 = val[telem_start+13];
	report.telem2_15 = val[telem_start+14];
	report.telem2_16 = val[telem_start+15];
	report.telem2_17 = val[telem_start+16];
	report.telem2_18 = val[telem_start+17];
	report.telem2_19 = val[telem_start+18];
	report.telem2_20 = val[telem_start+19];
	report.telem2_21 = val[telem_start+20];
	report.telem2_22 = val[telem_start+21];
	report.telem2_23 = val[telem_start+22];
	report.telem2_24 = val[telem_start+23];
	report.telem2_25 = val[telem_start+24];
	report.telem2_26 = val[telem_start+25];
	report.telem2_27 = val[telem_start+26];
	report.telem2_28 = val[telem_start+27];
	report.telem2_29 = val[telem_start+28];
	report.telem2_30 = val[telem_start+29];
	report.telem2_31 = val[telem_start+30];
	report.telem2_32 = val[telem_start+31];
	report.telem2_33 = val[telem_start+32];
	report.telem2_34 = val[telem_start+33];


	if (ret < 0) {
		report.status = 1; // fail
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
	}
	else{
		report.status = 0; // OK
		ret = OK;
	}


	/* TODO: set proper ID */
	report.id = _id;

	/* publish it, if we are the primary */
	if (_my_rpm_topic_topic != nullptr) {
		orb_publish(ORB_ID(my_rpm_topic), _my_rpm_topic_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	
	perf_end(_sample_perf);
	return ret;
}

void
MY_RPM::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MY_RPM::cycle_trampoline, this, 5);
}

void
MY_RPM::stop()
{
	work_cancel(HPWORK, &_work);
}

void
MY_RPM::cycle_trampoline(void *arg)
{

	MY_RPM *dev = (MY_RPM *)arg;

	dev->cycle();

}

void
MY_RPM::cycle()
{
	if (_collect_phase) {
		_index_counter = addr_ind[_cycle_counter]; /*sonar from previous iteration collect is now read out */
		set_device_address(_index_counter);

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/* change i2c adress to next sonar */
		_cycle_counter = _cycle_counter + 1;

		if (_cycle_counter >= addr_ind.size()) {
			_cycle_counter = 0;
		}

		/* Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
		   Otherwise the next sonar would fire without the first one having received its reflected sonar pulse */

		if (_measure_ticks > USEC2TICK(_cycling_rate)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MY_RPM::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_cycling_rate));
			return;
		}
	}

	/* Measurement (firing) phase */

	/* ensure sonar i2c adress is still correct */
	_index_counter = addr_ind[_cycle_counter];
	set_device_address(_index_counter);

	/* Perform measurement */
	if (OK != measure()) {
		PX4_DEBUG("measure error sonar adress %d", _index_counter);
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MY_RPM::cycle_trampoline,
		   this,
		   USEC2TICK(_cycling_rate));

}

void
MY_RPM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}








/**
 * Local functions in support of the shell command.
 */
namespace my_rpm
{

MY_RPM	*g_dev;

int 	start(uint8_t id);
int 	start_bus(uint8_t id, int i2c_bus);
int 	stop();
int 	test();
int 	reset();
int 	info();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start(uint8_t id)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(id, i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(uint8_t id, int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new MY_RPM(id, i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(MY_RPM_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	px4_close(fd);
	return PX4_OK;

fail:

	if (fd >= 0) {
		px4_close(fd);
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct my_rpm_topic_s report;
	ssize_t sz;
	int ret;

	int fd = px4_open(MY_RPM_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'my_rpm start' if the driver is not running)", MY_RPM_DEVICE_PATH);
		return PX4_ERROR;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	PX4_INFO("PASS");
	return PX4_OK;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(MY_RPM_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	px4_close(fd);
	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
my_rpm_usage()
{
	PX4_INFO("usage: my_rpm command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", MY_RPM_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("\t-I --id (%d)", 1);
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|reset|info");
}

int
my_rpm_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t id = 1;
	bool start_all = false;

	int i2c_bus = MY_RPM_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:I:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'M':
			id = (uint8_t)atoi(myoptarg);
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return my_rpm::start(id);

		} else {
			return my_rpm::start_bus(id, i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return my_rpm::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return my_rpm::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return my_rpm::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return my_rpm::info();
	}

out_error:
	my_rpm_usage();
	return PX4_ERROR;
}
