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
 * @file RoboClaw.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "RoboClaw.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>


#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>
#include <math.h>

// The RoboClaw has a serial communication timeout of 10ms.
// Add a little extra to account for timing inaccuracy
#define TIMEOUT_US 10500
#define DEFAULT_WHEEL_DIAMETER 0.1905
#define DEFAULT_MOTOR_COUNT 6

// If a timeout occurs during serial communication, it will immediately try again this many times
#define TIMEOUT_RETRIES 3

// If a timeout occurs while disarmed, it will try again this many times. This should be a higher number,
// because stopping when disarmed is pretty important.
#define STOP_RETRIES 10

// Number of bytes returned by the Roboclaw when sending command 78, read both encoders
#define ENCODER_MESSAGE_SIZE 10

// Number of bytes for commands 18 and 19, read speeds.
#define ENCODER_SPEED_MESSAGE_SIZE 7

bool RoboClaw::taskShouldExit = false;

RoboClaw::RoboClaw(const char *deviceName, const char *baudRateParam):
	_uart(0), _uart_set(), _uart_timeout{.tv_sec = 0, .tv_usec = TIMEOUT_US}
{
	printf("Roboclaw starting...\n");
	_param_handles.actuator_write_period_ms = 	param_find("RBCLW_WRITE_PER");
	_param_handles.encoder_read_period_ms = 	param_find("RBCLW_READ_PER");
	_param_handles.counts_per_rev = 			param_find("RBCLW_COUNTS_REV");
	_param_handles.serial_baud_rate = 			param_find(baudRateParam);
	_param_handles.address = 				param_find("RBCLW_ADDRESS");
	_parameters_update();

	int tmpBaud;
	param_get(_param_handles.serial_baud_rate, &tmpBaud);
	printf("Roboclaw parameters updated:\n");
	printf(" -- RBCLW_PORT         = %s\n", deviceName);
	printf(" -- RBCLW_ADDRESS      = %d\n", int(_parameters.address));
	printf(" -- RBCLW_BAUD         = %d\n", tmpBaud);
	printf(" -- RBCLW_WRITE_PER    = %d\n", int(_parameters.actuator_write_period_ms));
	printf(" -- RBCLW_READ_PER     = %d\n", int(_parameters.encoder_read_period_ms));
	printf(" -------------------------- \n");
	printf("  \n");
	// start serial port
	int err = this->open_serial_port(deviceName, B38400);
	switch(err){
		case -1: err(1, "could not open %s", deviceName); break;
		case -2: err(1, "failed to get attr"); break;
		case -3: err(1, "failed to set input speed"); break;
		case -4: err(1, "failed to set output speed"); break;
		case -5: err(1, "failed to set attr"); break;
	}
	if(err >= 0) printf("Roboclaw port successfully created.\n");

	// setup default settings, reset encoders
	resetEncoders();
}

RoboClaw::~RoboClaw()
{
	// Unadvertise the distance sensor topic.
	if (_encoder_data_pub != nullptr) {
		orb_unadvertise(_encoder_data_pub);
	}
	setMotorDutyCycle(MOTOR_1, 0.0);
	setMotorDutyCycle(MOTOR_2, 0.0);
	close(_uart);
}

int RoboClaw::open_serial_port(const char* port, speed_t baud){
	int ret = 0;
	struct termios uart_config{};

	_uart = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if(_uart < 0){ return -1; }
	ret = tcgetattr(_uart, &uart_config);
	if(ret < 0){ return -2; }

	// Configure serial port options
	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	// uart_config.c_cflag &= ~HUPCL;
	// uart_config.c_lflag &= ~ICANON;

	ret = cfsetispeed(&uart_config, baud);
	if(ret < 0){ return -3; }
	ret = cfsetospeed(&uart_config, baud);
	if(ret < 0){ return -4; }
	// Apply serial port configuration
	ret = tcsetattr(_uart, TCSANOW, &uart_config);
	if(ret < 0){ return -5; }
	FD_ZERO(&_uart_set);
	return ret;
}

void RoboClaw::taskMain()
{
	// Make sure the Roboclaw is actually connected, so I don't just spam errors if it's not.
	uint8_t rbuff[4];
	// int err_code = _transaction(CMD_READ_STATUS, nullptr, 0, &rbuff[0], sizeof(rbuff), false, true);
	_transaction(CMD_READ_STATUS, nullptr, 0, &rbuff[0], sizeof(rbuff), false, true);

	// This main loop performs two different tasks, asynchronously:
	// - Send actuator_controls_0 to the Roboclaw as soon as they are available
	// - Read the encoder values at a constant rate
	// To do this, the timeout on the poll() function is used.
	// waitTime is the amount of time left (int microseconds) until the next time I should read from the encoders.
	// It is updated at the end of every loop. Sometimes, if the actuator_controls_0 message came in right before
	// I should have read the encoders, waitTime will be 0. This is fine. When waitTime is 0, poll() will return
	// immediately with a timeout. (Or possibly with a message, if one happened to be available at that exact moment)
	uint64_t actuatorsLastWritten = 0;
	uint64_t encoderTaskLastRun = 0;
	int waitTime = 0;

	_actuatorsSub = orb_subscribe(ORB_ID(actuator_outputs));
	orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);

	_armedSub = orb_subscribe(ORB_ID(actuator_armed));
	_paramSub = orb_subscribe(ORB_ID(parameter_update));

	pollfd fds[3];
	fds[0].fd = _paramSub;
	fds[0].events = POLLIN;
	fds[1].fd = _actuatorsSub;
	fds[1].events = POLLIN;
	fds[2].fd = _armedSub;
	fds[2].events = POLLIN;

	// memset((void *) &_wheelEncoderMsg[0], 0, sizeof(_wheelEncoderMsg));
	// _wheelEncoderMsg[0].pulses_per_rev = _parameters.counts_per_rev;
	// _wheelEncoderMsg[1].pulses_per_rev = _parameters.counts_per_rev;
	// memset((void *) &_wheelEncoderMsg, 0, sizeof(_wheelEncoderMsg));
	// for(int i = 0; i < DEFAULT_MOTOR_COUNT; i++){
	// 	_wheelEncoderMsg.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
	// 	_wheelEncoderMsg.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
	// 	_wheelEncoderMsg.position[i] = 0;
	// 	_wheelEncoderMsg.qpps[i] = 0;
	// 	_wheelEncoderMsg.distance[i] = 0.0;
	// 	_wheelEncoderMsg.velocity[i] = 0.0;
	// }
	// for(int i = 0; i < DEFAULT_MOTOR_COUNT; i++){
	// 	_wheelEncoderMsg.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
	// 	_wheelEncoderMsg.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
	// 	_wheelEncoderMsg.position[i] = _encoderCounts[i];
	// 	_wheelEncoderMsg.qpps[i] = _motorSpeeds[i];
	// 	_wheelEncoderMsg.distance[i] = (float) ((int32_t) _encoderCounts[i]) / (float) _wheelEncoderMsg.pulses_per_meter[i];
	// 	_wheelEncoderMsg.velocity[i] = (float) ((int32_t) _motorSpeeds[i]) / (float) _wheelEncoderMsg.pulses_per_meter[i];
	// }

	while (!taskShouldExit) {
		int pret = poll(fds, sizeof(fds) / sizeof(pollfd), waitTime / 1000);
		bool actuators_timeout = int(hrt_absolute_time() - actuatorsLastWritten) > 2000 * _parameters.actuator_write_period_ms;

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_update), _paramSub, &_paramUpdate);
			_parameters_update();
		}
		if(_parameters.counts_per_rev == 0) _parameters.counts_per_rev = 1;

		// No timeout, update on either the actuator controls or the armed state
		if (pret > 0 && (fds[1].revents & POLLIN || fds[2].revents & POLLIN || actuators_timeout)) {
			orb_copy(ORB_ID(actuator_outputs), _actuatorsSub, &_actuatorControls);
			orb_copy(ORB_ID(actuator_armed), _armedSub, &_actuatorArmed);

			int drive_ret = 0, turn_ret = 0;
			const bool disarmed = !_actuatorArmed.armed || _actuatorArmed.lockdown || _actuatorArmed.manual_lockdown
					      || _actuatorArmed.force_failsafe || actuators_timeout;

			if (disarmed) {
				// If disarmed, I want to be certain that the stop command gets through.
				int tries = 0;

				while (tries < STOP_RETRIES && ((drive_ret = drive(0.0)) <= 0 || (turn_ret = turn(0.0)) <= 0)) {
					PX4_ERR("Error trying to stop: Drive: %d, Turn: %d", drive_ret, turn_ret);
					tries++;
					px4_usleep(TIMEOUT_US);
				}

			} else {
				drive_ret = drive(_actuatorControls.output[0]);
				turn_ret = turn(_actuatorControls.output[1]);

				if (drive_ret <= 0 || turn_ret <= 0) {
					PX4_ERR("Error controlling RoboClaw. Drive err: %d. Turn err: %d", drive_ret, turn_ret);
				}
			}

			actuatorsLastWritten = hrt_absolute_time();

		} else {
			// A timeout occurred, which means that it's time to update the encoders
			encoderTaskLastRun = hrt_absolute_time();

			wheel_encoders_data_s encoder_data;
			encoder_data.timestamp = encoderTaskLastRun;
			if (readEncoder() > 0) {
				// _wheelEncoderMsg.timestamp = encoderTaskLastRun;
				// for(int i = 0; i < 2; i++){
				// 	// tmpMsg.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
				// 	_wheelEncoderMsg.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
				// 	_wheelEncoderMsg.position[i] = _encoderCounts[i];
				// 	_wheelEncoderMsg.qpps[i] = _motorSpeeds[i];
				// 	_wheelEncoderMsg.distance[i] = (float) ((int32_t) _encoderCounts[i]) / (float) _wheelEncoderMsg.pulses_per_meter[i];
				// 	_wheelEncoderMsg.velocity[i] = (float) ((int32_t) _motorSpeeds[i]) / (float) _wheelEncoderMsg.pulses_per_meter[i];
				// }
				// // _wheelEncodersAdv.publish(_wheelEncoderMsg);
				// if (_wheelEncodersAdv == nullptr) {
				// 	int instance;
				// 	_wheelEncodersAdv = orb_advertise(ORB_ID(wheel_encoders_data), &_wheelEncoderMsg, &instance, ORB_PRIO_DEFAULT);
				// 	PX4_INFO("Roboclaw encoders uorb topic \'wheel_encoders_data\' was advertised.");
				// } else {
				// 	// printf("pos1,spd1,pos2,spd2: %10.2f %10.2f %10.2f %10.2f\n", double(getMotorPosition(MOTOR_1)), double(getMotorSpeed(MOTOR_1)), double(getMotorPosition(MOTOR_2)), double(getMotorSpeed(MOTOR_2)));
				// 	orb_publish(ORB_ID(wheel_encoders_data), _wheelEncodersAdv, &_wheelEncoderMsg);
				// 	PX4_INFO("Roboclaw encoders uorb topic \'wheel_encoders_data\' was published");
				// }

				for(int i = 0; i < 2; i++){
					encoder_data.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
					encoder_data.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
					encoder_data.position[i] = _encoderCounts[i];
					encoder_data.qpps[i] = _motorSpeeds[i];
					encoder_data.distance[i] = (float) ((int32_t) _encoderCounts[i]) / (float) encoder_data.pulses_per_meter[i];
					encoder_data.velocity[i] = (float) ((int32_t) _motorSpeeds[i]) / (float) encoder_data.pulses_per_meter[i];
				}
				PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' encoder data was updated successfully");
				int instance_id;
				orb_publish_auto(ORB_ID(encoders_data), &_encoder_data_pub, &encoder_data, &instance_id, ORB_PRIO_DEFAULT);
				PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' was published");
			} else {
				PX4_ERR("Error reading encoders");
				for(int i = 0; i < 2; i++){
					encoder_data.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
					encoder_data.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
					encoder_data.position[i] = 0;
					encoder_data.qpps[i] = 0;
					encoder_data.distance[i] = (float) ((int32_t) 0) / (float) encoder_data.pulses_per_meter[i];
					encoder_data.velocity[i] = (float) ((int32_t) 0) / (float) encoder_data.pulses_per_meter[i];
				}
				// PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' encoder data was updated successfully");
				int instance_id;
				orb_publish_auto(ORB_ID(encoders_data), &_encoder_data_pub, &encoder_data, &instance_id, ORB_PRIO_DEFAULT);
				PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' was published with null values");
			}
			// int instance_id;
			// orb_publish_auto(ORB_ID(encoders_data), &_encoder_data_pub, &encoder_data, &instance_id, ORB_PRIO_DEFAULT);
			// PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' was AUTO published");
		}

		waitTime = _parameters.encoder_read_period_ms * 1000 - (hrt_absolute_time() - encoderTaskLastRun);
		waitTime = waitTime < 0 ? 0 : waitTime;
	}

	orb_unsubscribe(_actuatorsSub);
	orb_unsubscribe(_armedSub);
	orb_unsubscribe(_paramSub);
}

int RoboClaw::readEncoder()
{

	uint8_t rbuff_pos[ENCODER_MESSAGE_SIZE];
	// I am saving space by overlapping the two separate motor speeds, so that the final buffer will look like:
	// [<speed 1> <speed 2> <status 2> <checksum 2>]
	// And I just ignore all of the statuses and checksums. (The _transaction() function internally handles the
	// checksum)
	uint8_t rbuff_speed[ENCODER_SPEED_MESSAGE_SIZE + 4];

	bool success = false;

	for (int retry = 0; retry < TIMEOUT_RETRIES && !success; retry++) {
		success = _transaction(CMD_READ_BOTH_ENCODERS, nullptr, 0, &rbuff_pos[0], ENCODER_MESSAGE_SIZE, false,
				       true) == ENCODER_MESSAGE_SIZE;
		success = success && _transaction(CMD_READ_SPEED_1, nullptr, 0, &rbuff_speed[0], ENCODER_SPEED_MESSAGE_SIZE, false,
						  true) == ENCODER_SPEED_MESSAGE_SIZE;
		success = success && _transaction(CMD_READ_SPEED_2, nullptr, 0, &rbuff_speed[4], ENCODER_SPEED_MESSAGE_SIZE, false,
						  true) == ENCODER_SPEED_MESSAGE_SIZE;
	}

	if (!success) {
		PX4_ERR("Error reading encoders");
		return -1;
	}

	uint32_t count;
	uint32_t speed;
	uint8_t *count_bytes;
	uint8_t *speed_bytes;

	for (int motor = 0; motor <= 1; motor++) {
		count = 0;
		speed = 0;
		count_bytes = &rbuff_pos[motor * 4];
		speed_bytes = &rbuff_speed[motor * 4];

		// Data from the roboclaw is big-endian. This converts the bytes to an integer, regardless of the
		// endianness of the system this code is running on.
		for (int byte = 0; byte < 4; byte++) {
			count = (count << 8) + count_bytes[byte];
			speed = (speed << 8) + speed_bytes[byte];
		}

		// The Roboclaw stores encoder counts as unsigned 32-bit ints. This can overflow, especially when starting
		// at 0 and moving backward. The Roboclaw has overflow flags for this, but in my testing, they don't seem
		// to work. This code detects overflow manually.
		// These diffs are the difference between the count I just read from the Roboclaw and the last
		// count that was read from the roboclaw for this motor. fwd_diff assumes that the wheel moved forward,
		// and rev_diff assumes it moved backward. If the motor actually moved forward, then rev_diff will be close
		// to 2^32 (UINT32_MAX). If the motor actually moved backward, then fwd_diff will be close to 2^32.
		// To detect and account for overflow, I just assume that the smaller diff is correct.
		// Strictly speaking, if the wheel rotated more than 2^31 encoder counts since the last time I checked, this
		// will be wrong. But that's 1.7 million revolutions, so it probably won't come up.
		uint32_t fwd_diff = count - _lastEncoderCount[motor];
		uint32_t rev_diff = _lastEncoderCount[motor] - count;
		// At this point, abs(diff) is always <= 2^31, so this cast from unsigned to signed is safe.
		int32_t diff = fwd_diff <= rev_diff ? fwd_diff : -int32_t(rev_diff);
		_encoderCounts[motor] += diff;
		_lastEncoderCount[motor] = count;

		_motorSpeeds[motor] = speed;
	}

	return 1;
}

void RoboClaw::printStatus(char *string, size_t n)
{
	snprintf(string, n, "pos1,spd1,pos2,spd2: %10.2f %10.2f %10.2f %10.2f\n",
		 double(getMotorPosition(MOTOR_1)),
		 double(getMotorSpeed(MOTOR_1)),
		 double(getMotorPosition(MOTOR_2)),
		 double(getMotorSpeed(MOTOR_2)));
}

float RoboClaw::getMotorPosition(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _encoderCounts[0];

	} else if (motor == MOTOR_2) {
		return _encoderCounts[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

float RoboClaw::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motorSpeeds[0];

	} else if (motor == MOTOR_2) {
		return _motorSpeeds[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

int RoboClaw::setMotorSpeed(e_motor motor, float value)
{
	e_command command;

	// send command
	if (motor == MOTOR_1) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_1;

		} else {
			command = CMD_DRIVE_REV_1;
		}

	} else if (motor == MOTOR_2) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_2;

		} else {
			command = CMD_DRIVE_REV_2;
		}

	} else {
		return -1;
	}

	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{

	e_command command;

	// send command
	if (motor == MOTOR_1) {
		command = CMD_SIGNED_DUTYCYCLE_1;

	} else if (motor == MOTOR_2) {
		command = CMD_SIGNED_DUTYCYCLE_2;

	} else {
		return -1;
	}

	return _sendSigned16Bit(command, value);
}

int RoboClaw::drive(float value)
{
	e_command command = value >= 0 ? CMD_DRIVE_FWD_MIX : CMD_DRIVE_REV_MIX;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::turn(float value)
{
	e_command command = value >= 0 ? CMD_TURN_LEFT : CMD_TURN_RIGHT;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::resetEncoders()
{
	return _sendNothing(CMD_RESET_ENCODERS);
}

int RoboClaw::_sendUnsigned7Bit(e_command command, float data)
{
	data = fabs(data);

	if (data > 1.0f) {
		data = 1.0f;
	}

	auto byte = (uint8_t)(data * INT8_MAX);
	uint8_t recv_byte;
	return _transaction(command, &byte, 1, &recv_byte, 1);
}

int RoboClaw::_sendSigned16Bit(e_command command, float data)
{
	if(data > 1.0f){ data = 1.0f; }
	else if(data < -1.0f){ data = -1.0f; }

	auto buff = (uint16_t)(data * INT16_MAX);
	uint8_t recv_buff;
	return _transaction(command, (uint8_t *) &buff, 2, &recv_buff, 1);
}

int RoboClaw::_sendNothing(e_command command)
{
	uint8_t recv_buff;
	return _transaction(command, nullptr, 0, &recv_buff, 1);
}

void RoboClaw::crc_clear(){ _crc = 0; }
uint16_t RoboClaw::crc_get(){ return _crc; }
void RoboClaw::crc_update(const uint8_t data){
	_crc = _crc ^ ((uint16_t)data << 8);
	for(int i = 0;i < 8;i++){
		if(_crc & 0x8000) _crc = (_crc << 1) ^ 0x1021;
		else _crc <<= 1;
	}
}
uint16_t RoboClaw::_calcCRC(const uint8_t *buf, size_t n, uint16_t init, bool verbose){
	uint16_t crc = init;
	if(verbose) printf("[INFO] _calcCRC() --- %d", (int)crc);
	for (uint8_t byte = 0; byte < n; byte++) {
		if(verbose) printf(" %d", (int)buf[byte]);
		crc = crc ^ (((uint16_t) buf[byte]) << 8);
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) { crc = (crc << 1) ^ 0x1021;
			} else { crc = crc << 1; }
		}
	}
	return crc;
}

int RoboClaw::_transaction(e_command cmd, uint8_t *wbuff, size_t wbytes,
	uint8_t *rbuff, size_t rbytes, bool send_checksum, bool recv_checksum)
{
	int err_code = 0;
	crc_clear();
	tcflush(_uart, TCIOFLUSH); // flush  buffers
	// WRITE
	uint8_t buf[wbytes + 4];
	buf[0] = (uint8_t) _parameters.address;
	buf[1] = cmd;
	crc_update((uint8_t) _parameters.address);
	crc_update(cmd);

	if(wbuff){ memcpy(&buf[2], wbuff, wbytes); }
	wbytes = wbytes + (send_checksum ? 4 : 2);
	if(send_checksum){
		uint16_t sum = _calcCRC(buf, wbytes - 2);
		buf[wbytes - 2] = (sum >> 8) & 0xFF;
		buf[wbytes - 1] = sum & 0xFFu;
	}
	int count = write(_uart, buf, wbytes);
	if(count < (int) wbytes){ // Did not successfully send all bytes.
		PX4_ERR("Only wrote %d out of %d bytes", count, (int) wbytes);
		return -1;
	} else{
		// printf(" - Wrote %d bytes --", count);
		// for(int i = 0; i < count; i++){ printf(" %#04x", buf[i]); } printf("\n");
	}

	// READ
	size_t bytes_read = 0;
	uint8_t *rbuff_curr = rbuff;
	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);

	// select(...) returns as soon as even 1 byte is available. read(...) returns immediately, no matter how many
	// bytes are available. I need to keep reading until I get the number of bytes I expect.
	while(bytes_read < rbytes){
		// select(...) may change this timeout struct (because it is not const). So I reset it every time.
		_uart_timeout.tv_sec = 0;
		_uart_timeout.tv_usec = TIMEOUT_US;
		err_code = select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout);

		// An error code of 0 means that select timed out, which is how the Roboclaw indicates an error.
		if(err_code <= 0) {
			printf(" -- Error during read = %d.\n", err_code);
			return err_code;
		}

		size_t nBytes = rbytes - bytes_read;
		err_code = read(_uart, rbuff_curr, nBytes);
		// printf(" ---- reading for %d bytes...\n", int(nBytes));
		if(err_code <= 0) {
			printf(" ---- Error during 2nd read = %d.\n", err_code);
			return err_code;
		} else{
			bytes_read += err_code;
			rbuff_curr += err_code;
			// printf(" ---- Read returned = %d byte(s) = %d (%#04x).\n", err_code, int(rbuff[bytes_read]), rbuff[bytes_read]);
		}
	}
	//TODO: Clean up this mess of IFs and returns
	if(recv_checksum){
		if(bytes_read < 2){ printf(" -- Unable to read 2 bytes.\n"); return -1; }

		// The checksum sent back by the roboclaw is calculated based on the address and command bytes as well
		// as the data returned.
		uint16_t checksum_calc = _calcCRC(buf, 2);
		checksum_calc = _calcCRC(rbuff_curr, bytes_read - 2, checksum_calc);
		// printf(" -- Checksum original -- (%d or %#08x).\n", (int)checksum_calc, checksum_calc);
		// uint16_t checksum_calc2 = crc_get();
		// printf(" -- Checksum test -- (%d or %#08x).\n", (int)checksum_calc2, checksum_calc2);
		uint16_t checksum_recv = (rbuff_curr[bytes_read - 2] << 8) + rbuff_curr[bytes_read - 1];
		// printf(" ---- Received bytes ->");
		// for(int i = 0; i < (int) bytes_read;i++){
		// 	printf("  %d (%#04x),", (int) rbuff_curr[i], rbuff_curr[i]);
		// }
		// printf("\n");
		if(checksum_calc == checksum_recv){
			// printf(" -- Successfully read %d bytes.\n",bytes_read);
			return bytes_read;
		} else {
			// printf(" -- Checksum recieved (%d or %#08x) doesn't match expected checksum (%d or %#08x).\n",
			// 	(int)checksum_recv, checksum_recv,
			// 	(int)checksum_calc, checksum_calc
			// );
			return -10;
		}
	} else{
		if(bytes_read == 1 && rbuff[0] == 0xFF){
			// printf(" -- Successfully read 1 byte.\n");
			return 1;
		} else{
			printf(" -- Unknown error = -11.\n");
			return -11;
		}
	}
}

void RoboClaw::_parameters_update()
{
	param_get(_param_handles.counts_per_rev, &_parameters.counts_per_rev);
	param_get(_param_handles.encoder_read_period_ms, &_parameters.encoder_read_period_ms);
	param_get(_param_handles.actuator_write_period_ms, &_parameters.actuator_write_period_ms);
	param_get(_param_handles.address, &_parameters.address);

	if (_actuatorsSub > 0) {
		orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);
	}

	int baudRate;
	param_get(_param_handles.serial_baud_rate, &baudRate);

	switch (baudRate) {
	case 2400:
		_parameters.serial_baud_rate = B2400;
		break;

	case 9600:
		_parameters.serial_baud_rate = B9600;
		break;

	case 19200:
		_parameters.serial_baud_rate = B19200;
		break;

	case 38400:
		_parameters.serial_baud_rate = B38400;
		break;

	case 57600:
		_parameters.serial_baud_rate = B57600;
		break;

	case 115200:
		_parameters.serial_baud_rate = B115200;
		break;

	case 230400:
		_parameters.serial_baud_rate = B230400;
		break;

	case 460800:
		_parameters.serial_baud_rate = B460800;
		break;

	default:
		_parameters.serial_baud_rate = B2400;
	}
}
