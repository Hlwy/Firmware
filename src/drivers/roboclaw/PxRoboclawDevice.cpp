#include "PxRoboclawDevice.hpp"
#include <inttypes.h> // uint16_t
#include <stdlib.h> // strtoul
#include <string.h> // strcmp
#include <unistd.h> // usleep
#include <sched.h> // task_create
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <math.h> // fabs
#include <stdio.h> // snprintf
// px4 includes
#include <systemlib/err.h> // errx
#include <systemlib/mavlink_log.h>
#include <drivers/device/device.h> // device::CDev
// #include <drivers/drv_sensor.h> // ioctl flags
#include <drivers/drv_hrt.h> // hrt_absolute_time
#include <uORB/topics/actuator_controls.h> // actuator_controls
#include <uORB/topics/actuator_outputs.h> // actuator_outputs
#include <uORB/topics/wheel_encoders_data.h> // encoders
// roboclaw
// #include <nuttx/poll.h>
#include <sys/types.h>


#define V_INT(V) (V*10) // note that in docs V_MIN has different equation, but doesn't match firmware
#define DUTY_INT(DUTY) (DUTY*1500)
#define DEFAULT_WHEEL_DIAMETER 0.1905
#define DEFAULT_COUNTS_PER_METER 9656
#define TIMEOUT_US 10500
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

static const uint8_t scaleVolts = 10;
static const uint16_t scaleDuty = 1500;
bool Px4RoboClawDevice::taskShouldExit = false;

Px4RoboClawDevice::Px4RoboClawDevice(const char *port, uint8_t address, uint32_t tout, bool doack)
	: CDev("roboclaw", "/dev/roboclaw"), m_roboclaw(port, 128, tout, doack),
	m_controls{ORB_ID(actuator_controls_0)},
	// // publication
	m_outputs{ORB_ID(actuator_outputs)},
	// m_encoders{ORB_ID(wheel_encoders_data)},
	// data
	m_mavlink_fd(-1), m_address(address), m_error(ROBO_ERROR_NONE),
	m_timeErrorSent(0), m_timeActuatorCommand(0), m_timeUpdate(0),
	m_motor1Overflows(0), m_motor2Overflows(0), m_accel(DUTY_INT(20))
{
	loop_cnt = 0;
	printf("Roboclaw starting...\n");
	_param_handles.actuator_write_period_ms = 	param_find("RBCLW_WRITE_PER");
	_param_handles.encoder_read_period_ms = 	param_find("RBCLW_READ_PER");
	_param_handles.counts_per_rev = 			param_find("RBCLW_COUNTS_REV");
	// _param_handles.serial_baud_rate = 			param_find(baudRateParam);
	_param_handles.address = 				param_find("RBCLW_ADDRESS");
	_parameters_update();

	// int tmpBaud;
	// param_get(_param_handles.serial_baud_rate, &tmpBaud);
	printf("Roboclaw parameters updated:\n");
	// printf(" -- RBCLW_PORT         = %s\n", deviceName);
	printf(" -- RBCLW_ADDRESS      = %d\n", int(_parameters.address));
	// printf(" -- RBCLW_BAUD         = %d\n", tmpBaud);
	printf(" -- RBCLW_WRITE_PER    = %d\n", int(_parameters.actuator_write_period_ms));
	printf(" -- RBCLW_READ_PER     = %d\n", int(_parameters.encoder_read_period_ms));
	printf(" -------------------------- \n");
	printf("  \n");


	// _actuatorsSub = orb_subscribe(ORB_ID(actuator_outputs));
	// orb_set_interval(_actuatorsSub, 10);
	// _armedSub = orb_subscribe(ORB_ID(actuator_armed));
	// _paramSub = orb_subscribe(ORB_ID(parameter_update));

	init();
}

Px4RoboClawDevice::~Px4RoboClawDevice()
{
	// orb_unsubscribe(_actuatorsSub);
	// orb_unsubscribe(_armedSub);
	if (_encoder_data_pub != nullptr) { orb_unadvertise(_encoder_data_pub); }
	// stop motors
	m_roboclaw.DutyM1((uint16_t) 0);
	m_roboclaw.DutyM2((uint16_t) 0);
	// close mavlink port
	::close(m_mavlink_fd);
}

int Px4RoboClawDevice::init(){
	int ret = device::CDev::init();
	if (ret < 0) { return ret; }
	if (m_mavlink_fd == -1) {
		m_mavlink_fd = ::open("/dev/mavlink", 0);
		if (m_mavlink_fd < 0) { return m_mavlink_fd; }
	}
	// stop motors
	m_roboclaw.SetLogicVoltages(6.0 * scaleVolts, 13.0 * scaleVolts);
	m_roboclaw.SetMainVoltages(10.0 * scaleVolts, 13.0 * scaleVolts);
	// m_roboclaw.WriteNVM();
	m_roboclaw.DutyM1((uint16_t) 10);
	m_roboclaw.DutyM2((uint16_t) 10);
	m_roboclaw.ResetEncoders();
	char version[200];
	m_roboclaw.ReadVersion(version);
	printf("ROBOCLAW VERSION: %s\r\n", version);
	return OK;
}

void Px4RoboClawDevice::update()
{
	// get current time
	uint64_t now = hrt_absolute_time();
	// ack
	bool error_status_valid = false;
	// read error status
	uint8_t error = m_roboclaw.ReadError(&error_status_valid);
	// if failed to read error status, use last
	if (!error_status_valid) {
		// printf("[ROBO] ERR STAT READ FAILED\r\n");
		// // mavlink_log_info(m_mavlink_fd, "[ROBO] ERR STAT READ FAILED");
		// warnx("[ROBO] ERR STAT READ FAILED");
		error = m_error;
	}
	// if there is no error now
	if (error == ROBO_ERROR_NONE) {
		// if there was an error, notify ground station
		if (m_error != ROBO_ERROR_NONE) {warnx("[ROBO] OK");}
	} else {
		// if it is new or we haven't sent message for a second,
		// send to ground station
		if (error != m_error || ((now - m_timeErrorSent) > 1000000)) {

			// general handling of errors
			char error_msg[200];
			errorToString(error, error_msg, 200);
			printf("ROBOCLAW ERROR: %s\r\n", error_msg);
			// mavlink_log_info(m_mavlink_fd, error_msg);
			warnx("%s", error_msg);
			m_timeErrorSent = now;

			// handle logic battery voltage error
			if (error & ROBO_ERROR_LOGIC_BATTERY_LOW ||
			    error & ROBO_ERROR_LOGIC_BATTERY_HIGH) {
				uint16_t voltage = m_roboclaw.ReadLogicBatteryVoltage();
				uint16_t min = 0;
				uint16_t max = 0;

				if (m_roboclaw.ReadMinMaxLogicVoltages( min, max)) {
					printf("[ROBO] ERR LOGIC VOLTAGE\r\n");
					// mavlink_log_info(m_mavlink_fd, "[ROBO] ERR LOGIC VOLTAGE");
					warnx("logic voltage out of range: "
					      "min: %5.1f max: %5.1f V: %5.1f",
					      min / 10.0, max / 10.0, voltage / 10.0);
				}
			}

			// handle main battery voltage error
			if (error & ROBO_ERROR_MAIN_BATTERY_LOW ||
			    error & ROBO_ERROR_MAIN_BATTERY_HIGH) {
				uint16_t voltage = m_roboclaw.ReadMainBatteryVoltage();
				uint16_t min = 0;
				uint16_t max = 0;

				if (m_roboclaw.ReadMinMaxMainVoltages( min, max)) {
					printf("[ROBO] ERR MAIN VOLTAGE\r\n");
					// mavlink_log_info(m_mavlink_fd, "[ROBO] ERR MAIN VOLTAGE");
					warnx("[ROBO] main voltage out of range: "
					      "min: %5.1f max: %5.1f V: %5.1f",
					      min / 10.0, max / 10.0, voltage / 10.0);
				}
			}

			// handle overcurrent error
			if (error & ROBO_ERROR_M1_OVERCURRENT ||
			    error & ROBO_ERROR_M2_OVERCURRENT) {
				int16_t current1;
				int16_t current2;

				if (m_roboclaw.ReadCurrents( current1, current2)) {
					printf("[ROBO] OVER CURRENT\r\n");
					// mavlink_log_info(m_mavlink_fd, "[ROBO] OVER CURRENT");
					warnx("[ROBO] motor overcurrent: "
					      "c1: %6.2f c2: %6.2f", current1 / 100.0, current2 / 100.0);
				}
			}

			// handle temperature error
			if (error & ROBO_ERROR_TEMPERATURE) {
				uint16_t temperature;

				if (m_roboclaw.ReadTemp( temperature)) {
					printf("[ROBO] OVER TEMP\r\n");
					// mavlink_log_info(m_mavlink_fd, "[ROBO] OVER TEMP");
					warnx("[ROBO] temperature over max: %7.1f", temperature / 10.0);
				}
			}
		}
	}
	// save error for next update
	m_error = error;

	// calculate rate
	int32_t timeElapsed = now - m_timeUpdate;
	m_timeUpdate = now;
	float freq = 1.0e6 / timeElapsed;
	if (freq < 30.0f && m_timeUpdate != 0) {
		printf("[ROBO] slow, %10.2f Hz\r\n", double(freq));
		// mavlink_log_info(m_mavlink_fd, "[ROBO] slow, %10.2f Hz", double(freq));
		warnx("[ROBO] slow, %10.2f Hz", double(freq));
	}
	// default commands to zero unless we receive data

	// float motor1 = 0;
	// float motor2 = 0;
	float control[2];
	if ((m_error == ROBO_ERROR_NONE)) {
		if (m_controls.updated()) {
			m_timeActuatorCommand = now;
			m_controls.copy(&_driveCntrls);
			control[0] = _driveCntrls.control[0];
			control[1] = _driveCntrls.control[1];
			printf("Received Raw Commands = %f, %f\r\n", (double)control[0], (double)control[1]);

			int16_t duty[2];
			for (int i = 0; i < 2; i++) {
				if (control[i] > 1) { control[i] = 1.0; }
				if (control[i] < -1) { control[i] = -1.0; }
				// duty[i] = INT16_MAX * control[i];
				duty[i] = 5000 * control[i];
				// duty[i] = scaleDuty * control[i];
			}
			// printf("Outputting Scaled Controls = %d, %d\r\n", int(duty[0] + scaleDuty), int(duty[1] + scaleDuty));
			printf("Outputting Scaled Controls = %d, %d\r\n", int(duty[0]), int(duty[1]));

			// m_roboclaw.DutyM1((uint16_t) duty[0]);
			// m_roboclaw.DutyM2((uint16_t) duty[1]);
			m_roboclaw.SpeedM1M2((uint32_t) 500,(uint32_t) 500);
			// m_roboclaw.DutyM1((uint16_t) duty[0] + scaleDuty);
			// m_roboclaw.DutyM2((uint16_t) duty[1] + scaleDuty);
		} else{ control[0] = 0; control[1] = 0; }
	} else{ control[0] = 0; control[1] = 0; }

	// warn if not getting control packets
	if (now - m_timeActuatorCommand > 1000000) {
		// warnx("not receiving control packets");
		m_timeActuatorCommand = now;
	}

	// int64_t pos1 = 0, pos2 = 0;
	// float dist1 = 0.0, dist2 = 0.0;
	// uint8_t status_counts1 = 0, status_counts2 = 0;
	bool valid_encoder1 = false, valid_encoder2 = false;
	// uint32_t gain = (uint32_t) _parameters.counts_per_rev;
	// uint32_t gain = (uint32_t) DEFAULT_COUNTS_PER_METER;
	uint8_t status_counts = 0;
	uint32_t counts = m_roboclaw.ReadEncM1( &status_counts, &valid_encoder1);
	// uint32_t counts1 = m_roboclaw.ReadEncM1( &status_counts1, &valid_encoder1);
	// uint32_t counts2 = m_roboclaw.ReadEncM2( &status_counts2, &valid_encoder2);
	if (valid_encoder1) {
		encoder_data.position[0] = encoderToInt64(counts, status_counts, &m_motor1Overflows);
		encoder_data.distance[0] = (float) (encoder_data.position[0]) / (float) DEFAULT_COUNTS_PER_METER;
		// printf("Encoders #%d: -- V1 (C1) = %d (%d)\r\n", loop_cnt, (int) spd1, (int) pos1);
	}
	counts = m_roboclaw.ReadEncM2( &status_counts, &valid_encoder2);
	if (valid_encoder2) {
		encoder_data.position[1] = encoderToInt64(counts, status_counts, &m_motor2Overflows);
		encoder_data.distance[1] = (float) (encoder_data.position[1]) / (float) DEFAULT_COUNTS_PER_METER;
		// printf("Encoders #%d: -- V2 (C2) = %d (%d)\r\n", loop_cnt, (int) spd2, (int) pos2);
	}

	// uint8_t status_speed = 0;
	// uint8_t status_speed1 = 0, status_speed2 = 0;
	bool valid_speed1 = false, valid_speed2 = false;
	// float vel1 = 0.0, vel2 = 0.0;
	// int32_t spd1 = m_roboclaw.ReadSpeedM1(&status_speed1,&valid_speed1);
	// int32_t spd2 = m_roboclaw.ReadSpeedM2(&status_speed2,&valid_speed2);
	int32_t spd = m_roboclaw.ReadSpeedM1(&status_counts,&valid_speed1);
	if (valid_speed1) {
		encoder_data.qpps[0] = spd;
		encoder_data.velocity[0] = (float) (spd) / (float) DEFAULT_COUNTS_PER_METER;
		// printf("[ROBO] ENCD 1 READ PPS = %f\r\n", tmpQpps);
	}
	spd = m_roboclaw.ReadSpeedM2(&status_counts,&valid_speed2);
	if (valid_speed2) {
		encoder_data.qpps[1] = spd;
		encoder_data.velocity[1] = (float) (spd) / (float) DEFAULT_COUNTS_PER_METER;
		// printf("[ROBO] ENCD 2 READ PPS = %f\r\n", tmpQpps);
	}

	bool all_data_valid = valid_encoder1 && valid_encoder2 && valid_speed1 && valid_speed2;
	if (all_data_valid) {
		// printf("[ROBO] data good\r\n");
		// wheel_encoders_data_s encoder_data = {0};
		encoder_data.timestamp = now;
		for(int i = 0; i < 2; i++){
			encoder_data.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
			encoder_data.pulses_per_meter[i] = (uint32_t) DEFAULT_COUNTS_PER_METER;
		}
		// PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' encoder data was updated successfully");
		int instance_id;
		orb_publish_auto(ORB_ID(encoders_data), &_encoder_data_pub, &encoder_data, &instance_id, ORB_PRIO_DEFAULT);
	}
	// if (readEncoder() > 0) {
	// 	for(int i = 0; i < 2; i++){
	// 		encoder_data.wheel_diameter[i] = DEFAULT_WHEEL_DIAMETER;
	// 		encoder_data.pulses_per_meter[i] = (uint32_t) _parameters.counts_per_rev;
	// 		encoder_data.position[i] = _encoderCounts[i];
	// 		encoder_data.qpps[i] = _motorSpeeds[i];
	// 		encoder_data.distance[i] = _encoderDistances[i];
	// 		encoder_data.velocity[i] = _motorVelocities[i];
	// 	}
	// 	// PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' encoder data was updated successfully");
	// 	int instance_id;
	// 	orb_publish_auto(ORB_ID(encoders_data), &_encoder_data_pub, &encoder_data, &instance_id, ORB_PRIO_DEFAULT);
	// 	// PX4_INFO("Roboclaw encoders uorb topic \'encoders_data\' was published");
	// 	// send new actuator commands to motors if there is no error
	// }

	// // float motor1 = 0;
	// // float motor2 = 0;
	// float control[2];
	// if ((m_error == ROBO_ERROR_NONE) && all_data_valid) {
	// 	if (m_controls.updated()) {
	// 		m_timeActuatorCommand = now;
	// 		m_controls.copy(&_driveCntrls);
	// 		control[0] = _driveCntrls.control[0];
	// 		control[1] = _driveCntrls.control[1];
	// 		printf("Received Raw Commands = %f, %f\r\n", (double)control[0], (double)control[1]);
	//
	// 		int16_t duty[2];
	// 		for (int i = 0; i < 2; i++) {
	// 			if (control[i] > 1) { control[i] = 1.0; }
	// 			if (control[i] < -1) { control[i] = -1.0; }
	// 			// duty[i] = INT16_MAX * control[i];
	// 			duty[i] = 5000 * control[i];
	// 			// duty[i] = scaleDuty * control[i];
	// 		}
	// 		// printf("Outputting Scaled Controls = %d, %d\r\n", int(duty[0] + scaleDuty), int(duty[1] + scaleDuty));
	// 		printf("Outputting Scaled Controls = %d, %d\r\n", int(duty[0]), int(duty[1]));
	//
	// 		// m_roboclaw.DutyM1((uint16_t) duty[0]);
	// 		// m_roboclaw.DutyM2((uint16_t) duty[1]);
	// 		m_roboclaw.SpeedM1M2((uint32_t) 500,(uint32_t) 500);
	// 		// m_roboclaw.DutyM1((uint16_t) duty[0] + scaleDuty);
	// 		// m_roboclaw.DutyM2((uint16_t) duty[1] + scaleDuty);
	// 	} else{ control[0] = 0; control[1] = 0; }
	// } else{ control[0] = 0; control[1] = 0; }
	//
	// // warn if not getting control packets
	// if (now - m_timeActuatorCommand > 1000000) {
	// 	// warnx("not receiving control packets");
	// 	m_timeActuatorCommand = now;
	// }

	// int16_t duty[2];
	// for (int i = 0; i < 2; i++) {
	// 	if (control[i] > 1) { control[i] = 1; }
	// 	if (control[i] < -1) { control[i] = -1; }
	// 	duty[i] = scaleDuty * control[i];
	// }
	// printf("Outputting Scaled Controls = %d, %d\r\n", int(duty[0] + scaleDuty), int(duty[1] + scaleDuty));

	// publish duty cycles
	// m_outputs.get().timestamp = now;
	// m_outputs.get().output[0] = duty[0] + scaleDuty; // have to add scale so positive (like servo)
	// m_outputs.get().output[1] = duty[1] + scaleDuty;
	// m_outputs.get().noutputs = 2;
	// m_outputs.update(); // publish

	// send command to motor
	// m_roboclaw.DutyAccelM1M2((uint16_t) duty[0],(uint32_t) m_accel, (uint16_t) duty[1],(uint32_t) m_accel); // note, don't want to add scaleDuty here
	// m_roboclaw.DutyM1((uint16_t) duty[0] + scaleDuty);
	// m_roboclaw.DutyM2((uint16_t) duty[1] + scaleDuty);
}


int Px4RoboClawDevice::readEncoder()
{
	// read encoders
	// int32_t spd1 = 0, spd2 = 0;
	// uint32_t counts1 = 0, counts2 = 0;
	// bool valid_encoders = m_roboclaw.ReadEncoders(counts1,counts2);
	// if(!valid_encoders){ counts1 = 0; counts2 = 0; }
	int64_t pos1 = 0, pos2 = 0;
	float dist1 = 0.0, dist2 = 0.0;
	uint8_t status_counts1 = 0, status_counts2 = 0;
	bool valid_encoder1 = false, valid_encoder2 = false;
	uint32_t gain = (uint32_t) _parameters.counts_per_rev;
	uint32_t counts1 = m_roboclaw.ReadEncM1( &status_counts1, &valid_encoder1);
	uint32_t counts2 = m_roboclaw.ReadEncM2( &status_counts2, &valid_encoder2);
	if (valid_encoder1) {
		pos1 = encoderToInt64(counts1, status_counts1, &m_motor1Overflows);
		dist1 = (float) (pos1) / (float) gain;
		// printf("Encoders #%d: -- V1 (C1) = %d (%d)\r\n", loop_cnt, (int) spd1, (int) pos1);
	}
	if (valid_encoder2) {
		pos2 = encoderToInt64(counts2, status_counts2, &m_motor2Overflows);
		dist2 = (float) (pos2) / (float) gain;
		// printf("Encoders #%d: -- V2 (C2) = %d (%d)\r\n", loop_cnt, (int) spd2, (int) pos2);
	}

	uint8_t status_speed1 = 0, status_speed2 = 0;
	bool valid_speed1 = false, valid_speed2 = false;
	float vel1 = 0.0, vel2 = 0.0;
	int32_t spd1 = m_roboclaw.ReadSpeedM1(&status_speed1,&valid_speed1);
	int32_t spd2 = m_roboclaw.ReadSpeedM2(&status_speed2,&valid_speed2);
	if (valid_speed1) {
		vel1 = (float) (spd1) / (float) gain;
		// printf("[ROBO] ENCD 1 READ PPS = %f\r\n", tmpQpps);
	}
	if (valid_speed2) {
		vel2 = (float) (spd2) / (float) gain;
		// printf("[ROBO] ENCD 2 READ PPS = %f\r\n", tmpQpps);
	}
	// int64_t _encoderCounts[2] {counts1, counts2};
	// int64_t _encoderCounts[2] {pos1, pos2};
	// int32_t _motorSpeeds[2] {spd1, spd2};

	bool all_data_valid = valid_encoder1 && valid_encoder2 && valid_speed1 && valid_speed2;
	if (all_data_valid) {
		printf("[ROBO] data good\r\n");
		_encoderCounts[0] += pos1;
		_encoderCounts[1] += pos2;
		_motorSpeeds[0] = spd1;
		_motorSpeeds[1] = spd2;
		_encoderDistances[0] = dist1;
		_encoderDistances[1] = dist2;
		_motorVelocities[0] = vel1;
		_motorVelocities[1] = vel2;
		return 1;
	}
	// else{
	// 	_motorSpeeds[0] = 0;
	// 	_motorSpeeds[1] = 0;
	// 	_encoderDistances[0] = 0.0;
	// 	_encoderDistances[1] = 0.0;
	// 	_motorVelocities[0] = 0.0;
	// 	_motorVelocities[1] = 0.0;
	// }
	return -1;
}

void Px4RoboClawDevice::errorToString(uint8_t error, char *msg, size_t n){
	char buf[200] = "[ROBO] ";
	const char *m1oc = "m1 oc,";
	const char *m2oc = "m2 oc,";
	const char *estop = "estop,";
	const char *temp = " temp,";
	const char *mbatl = " main low,";
	const char *mbath = " main high,";
	const char *lbatl = " logic low,";
	const char *lbath = " logic high,";

	if (error & ROBO_ERROR_M1_OVERCURRENT) { strncat(buf, m1oc, strnlen(m1oc, 20)); }
	if (error & ROBO_ERROR_M2_OVERCURRENT) { strncat(buf, m2oc, strnlen(m2oc, 20)); }
	if (error & ROBO_ERROR_ESTOP) { strncat(buf, estop, strnlen(estop, 20)); }
	if (error & ROBO_ERROR_TEMPERATURE) { strncat(buf, temp, strnlen(temp, 20)); }
	if (error & ROBO_ERROR_MAIN_BATTERY_LOW) { strncat(buf, mbatl, strnlen(mbatl, 20)); }
	if (error & ROBO_ERROR_MAIN_BATTERY_HIGH) { strncat(buf, mbath, strnlen(mbath, 20)); }
	if (error & ROBO_ERROR_LOGIC_BATTERY_LOW) { strncat(buf, lbatl, strnlen(lbatl, 20)); }
	if (error & ROBO_ERROR_LOGIC_BATTERY_HIGH) { strncat(buf, lbath, strnlen(lbath, 20)); }
	strncpy(msg, buf, n);
}
int64_t Px4RoboClawDevice::encoderToInt64(uint32_t count, uint8_t status, int32_t *overflows){
	static int64_t overflowAmount = 0x100000000LL;
	if (status & ROBO_ENCODER_OVERFLOW) { (*overflows) += 1; }
	if (status & ROBO_ENCODER_UNDERFLOW) { (*overflows) -= 1; }
	return int64_t(count) + (*overflows) * overflowAmount;
}
void Px4RoboClawDevice::_parameters_update()
{
	param_get(_param_handles.counts_per_rev, &_parameters.counts_per_rev);
	param_get(_param_handles.encoder_read_period_ms, &_parameters.encoder_read_period_ms);
	param_get(_param_handles.actuator_write_period_ms, &_parameters.actuator_write_period_ms);
	param_get(_param_handles.address, &_parameters.address);

	if (_actuatorsSub > 0) { orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms); }

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
