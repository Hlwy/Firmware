/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file roboclaw_main.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf
 *
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h> // strcmp
 #include <px4_platform_common/module.h>

 #include <systemlib/err.h> // errx
 #include <parameters/param.h>
 #include "PxRoboclawDevice.hpp"
 #include "RoboclawDriver.h"
 #include "RoboClaw.hpp"

volatile static bool thread_should_exit = false; // Daemon exit flag
volatile static bool thread_running = false; // Daemon status flag
volatile static bool test_thread_running = false; // Daemon status flag
volatile static int daemon_task; // Handle of deamon task / thread


/**
 * Deamon management function.
 */
extern "C" __EXPORT int roboclaw_main(int argc, char *argv[]);
/**
 * Mainloop of deamon.
 */
int roboclaw_thread_main(int argc, char *argv[]);
int roboclaw_test_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage();

static void usage()
{
	PRINT_MODULE_USAGE_NAME("roboclaw", "driver");

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

This driver communicates over UART with the [Roboclaw motor driver](http://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf).
It performs two tasks:

 - Control the motors based on the `actuator_controls_0` UOrb topic.
 - Read the wheel encoders and publish the raw data in the `wheel_encoders` UOrb topic

In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and
your flight controller's UART port should be connected to the Roboclaw as shown in the documentation. For Pixhawk 4,
use the `UART & I2C B` port, which corresponds to `/dev/ttyS3`.

### Implementation

The main loop of this module (Located in `RoboClaw.cpp::task_main()`) performs 2 tasks:

 1. Write `actuator_controls_0` messages to the Roboclaw as they become available
 2. Read encoder data from the Roboclaw at a constant, fixed rate.

Because of the latency of UART, this driver does not write every single `actuator_controls_0` message to the Roboclaw
immediately. Instead, it is rate limited based on the parameter `RBCLW_WRITE_PER`.

On startup, this driver will attempt to read the status of the Roboclaw to verify that it is connected. If this fails,
the driver terminates immediately.

### Examples

The command to start this driver is:

 $ roboclaw start <device> <baud>

`<device>` is the name of the UART port. On the Pixhawk 4, this is `/dev/ttyS3`.
`<baud>` is te baud rate.

All available commands are:

 - `$ roboclaw start <device> <baud>`
 - `$ roboclaw status`
 - `$ roboclaw stop`
	)DESCR_STR");
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
 /**
int roboclaw_main(int argc, char *argv[])
{

	if (argc < 4) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("roboclaw already running\n");
			// this is not an error
			return 0;
		}

		RoboClaw::taskShouldExit = false;
		deamon_task = px4_task_spawn_cmd("roboclawlistener actuator_controls_0 -r 10",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 10,
						 2000,
						 roboclaw_thread_main,
						 (char *const *)argv);
		return 0;

	} else if (!strcmp(argv[1], "stop")) {

		RoboClaw::taskShouldExit = true;
		return 0;

	} else if (!strcmp(argv[1], "status")) {

		if (thread_running) {
			printf("\troboclaw app is running\n");

		} else {
			printf("\troboclaw app not started\n");
		}

		return 0;
	}

	usage();
	return 1;
}*/
int roboclaw_main(int argc, char *argv[])
{
	const char *command = nullptr;

	// parse
	if (argc == 1) { command = "help";
	} else if (argc > 1) { command = argv[1];
	} else { errx(1, "wrong number of args"); }

	// handle
	if (!strcmp(command, "help")) {
		warnx("usage: (start|stop|status|test|reset)");
		return OK;

	} else if (!strcmp(command, "start")) {
		if (thread_running) {
			warnx("already running");
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("roboclaw",
          						 SCHED_DEFAULT,
          						 SCHED_PRIORITY_MAX - 10,
          						 2000,
          						 roboclaw_thread_main,
					     (argv) ? (char* const *)&argv[2] : (char* const*)NULL);
		return 0;

	} else if (!strcmp(command, "stop")) {
		thread_should_exit = true;
		int loop_count = 0;

		while (thread_running == true) {
			usleep(1000000);

			if (loop_count++ > 5) {
				warnx("forcing deletion");
				task_delete(daemon_task);
			}

			warnx("waiting for process to exit");
		}

		return 0;

	} else if (!strcmp(command, "status")) {
		if (thread_running) {
			warnx("is running");
			return 0;

		} else {
			warnx("not started");
			return -1;
		}

	} else if (!strcmp(command, "test")) {
		if (thread_running) {
			warnx("must stop first");
			return -1;

		} else if (test_thread_running) {
			warnx("test already running");

		} else {
			daemon_task = px4_task_spawn_cmd("roboclaw_test",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_MAX - 30,
						     2000,
						     roboclaw_test_main,
						     (argv) ? (char* const*)&argv[2] :
						     (char* const*)NULL);
			return 0;
		}

	} else if (!strcmp(command, "reset")) {
		if (thread_running) {
			warnx("not implemented");
			return 0;

		} else {
			warnx("not started");
			return -1;
		}

	} else {
		errx(1, "unknown command: %s", command);
          usage();
	}

	return OK;
}

int roboclaw_thread_main(int argc, char *argv[])
{
	printf("[roboclaw] starting\n");
	const char *port = "/dev/ttyS3";
	uint8_t address = 128;
	uint32_t timeout = 10; // 1 second
	bool doack = false; // do ack for writes

	// parse
	if (argc == 3) {
		port = argv[1];
		address = strtoul(argv[2], nullptr, 0);
	} else if (argc != 1) { errx(1, "wrong number of args"); }

	warnx("starting");
	thread_running = true;
	Px4RoboClawDevice roboclaw(port, address, timeout, doack);
	while (!thread_should_exit) { roboclaw.update(); }
     // roboclaw.taskMain();
	warnx("exiting.");
	thread_running = false;
	return 0;
}


int roboclaw_test_main(int argc, char *argv[])
{
	test_thread_running = true;
	// defaults
	const char *port = "/dev/ttyS3";
	uint8_t address = 128;
	//bool doack = false; // do ack for writes

	// parse
	if (argc == 3) {
		port = argv[1];
		address = strtoul(argv[2], nullptr, 0);

	} else if (argc != 1) {
		errx(1, "wrong number of args");
	}

	printf("starting new test.\n");

	// open port
	int uart = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (uart < 0) {
		errx(1, "failed to open port: %s", port);
		return 0;
	}

	// setup uart
	warnx("setting up uart");
	struct termios uart_config;
	int ret = tcgetattr(uart, &uart_config);

	if (ret < 0) { errx(1, "failed to get attr"); }
	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, B38400);
	if (ret < 0) { errx(1, "failed to set input speed"); }
	ret = cfsetospeed(&uart_config, B38400);
	if (ret < 0) { errx(1, "failed to set output speed"); }
	ret = tcsetattr(uart, TCSANOW, &uart_config);
	if (ret < 0) { errx(1, "failed to set attr"); }
	// clear old data
	tcflush(uart, TCIOFLUSH);

	// message data
	uint8_t get_version = 21;
	char msg[200];
	char buf[10];

	// request version
	printf("requesting version\n");
	write(uart, &address, 1);
	write(uart, &get_version, 1);

     printf("reading version with poll\n");
     struct pollfd uartPoll;
     uint32_t tout = 1000;
     uartPoll.fd = uart;
     uartPoll.events = POLLIN;
	// read version with poll
     while (true) {
          int pollrc = poll(&uartPoll, 1, tout);
          if (pollrc < 1) { break; }
          ret = ::read(uart, buf, sizeof(buf));
          if (ret < 1) { break; }
          strncat(msg, (const char *)buf, ret);
     }
     printf("poll msg: %s\n", msg);
     msg[0] = '\0';

	// request version
	printf("requesting version\n");
	write(uart, &address, 1);
	write(uart, &get_version, 1);
     // read version w/o poll
     printf("reading version w/o poll\n");
     msg[0] = '\0';
     for (int i = 0; i < 10000; i++) {
          ret = ::read(uart, buf, sizeof(buf));
          if (ret < 1) { continue; }
          strncat(msg, (const char *)buf, ret);
     }
     printf("no poll msg: %s\n", msg);
     // close uart
     close(uart);

     RoboClawAlt roboclawTest(port, "38400");
     while (!thread_should_exit) { roboclawTest.drive(0.5); }
     roboclawTest.drive(0.0);

	test_thread_running = false;
	return 0;
}
