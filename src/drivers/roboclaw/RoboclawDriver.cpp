#include "RoboclawDriver.h"

// px4 added
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <systemlib/err.h> // errx
#include <unistd.h> // usleep

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

RoboClaw::RoboClaw(const char *port, uint8_t address, uint32_t tout, bool doack){
	_add = address;
	_timeout = tout;
	ack = doack;
	uart = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	// uart = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);

	// setup uart
	struct termios uart_config;
	int ret = tcgetattr(uart, &uart_config);
	if (ret < 0) { printf("failed to get attr\r\n"); }

	ret = cfsetispeed(&uart_config, B38400);
	if (ret < 0) { printf("failed to set input speed\r\n"); exit(-1);}
	ret = cfsetospeed(&uart_config, B38400);
	if (ret < 0) { printf("failed to set output speed\r\n"); exit(-1);}

	uart_config.c_cflag |= (CLOCAL | CREAD);		//<Set baud rate
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~CRTSCTS;
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_iflag |= (IXON | IXOFF | IXANY);
	uart_config.c_oflag &= ~OPOST;
	// uart_config.c_iflag = IGNPAR;
	// uart_config.c_oflag = 0;
	// uart_config.c_oflag &= ~ONLCR; // no CR for every LF

	// tcflush(uart, TCIFLUSH);
	ret = tcsetattr(uart, TCSANOW, &uart_config);
	if (ret < 0) { printf("failed to set attr\r\n"); exit(-1);}

	// setup uart polling
	uartPoll[0].fd = uart;
	uartPoll[0].events = POLLIN;
	// flush();
}
RoboClaw::~RoboClaw(){ close(uart); }


int RoboClaw::write(uint8_t byte){
	if(::write(uart, &byte, 1) < 0) { return -1; }
     else { return 1; }
}

void RoboClaw::flush(){
	tcflush(uart, TCIFLUSH);
}
uint8_t RoboClaw::read(uint32_t timeout){
     uint8_t byte = 0;
	int pollrc = poll(uartPoll, 1, timeout);

	if (pollrc > 0) {
		if (uartPoll[0].revents & POLLIN) {
			int ret = ::read(uart, &byte, 1);
			if (ret < 0) byte = 0;
		}
	} else if (pollrc < 0) { printf("poll error\r\n");
	} else { printf("poll timeout\r\n"); }

	return byte;
}
void RoboClaw::crc_clear(){ crc = 0; }
void RoboClaw::crc_update(uint8_t data){
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++){
		if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
		else crc <<= 1;
	}
}
uint16_t RoboClaw::crc_get(){ return crc; }

bool RoboClaw::write_n(uint8_t cnt, ... ){
	uint8_t trys = max_retry_attempts;
	do{
		crc_clear(); flush();
		//send data with crc
		va_list marker;
		va_start(marker, cnt);     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint8_t data = va_arg(marker, int);
			crc_update(data);
			write(data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t tmpcrc = crc_get();
		write(tmpcrc>>8);
		write(tmpcrc);
		if(read(_timeout)==0xFF)
		return true;
	}while(trys--);
	return false;
}
bool RoboClaw::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...){
	uint32_t value = 0;
	uint8_t trys = max_retry_attempts;
	int16_t data;
	do{
		flush();

		data = 0;
		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		//send data with crc
		va_list marker;
		va_start(marker, cmd);     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);

			if(data!=-1){
				data = read(_timeout);
				crc_update(data);
				value=(uint32_t)data<<24;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(_timeout);
				crc_update(data);
				value|=(uint32_t)data<<16;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(_timeout);
				crc_update(data);
				value|=(uint32_t)data<<8;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(_timeout);
				crc_update(data);
				value|=(uint32_t)data;
			}
			else{
				break;
			}

			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					return crc_get()==ccrc;
				}
			}
		}
	}while(trys--);
	return false;
}

uint8_t RoboClaw::Read1(uint8_t address,uint8_t cmd,bool *valid){
	if(valid) *valid = false;

	uint8_t value = 0;
	uint8_t trys = max_retry_attempts;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(_timeout);
		crc_update(data);
		value=data;

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}
uint16_t RoboClaw::Read2(uint8_t address,uint8_t cmd,bool *valid){
	if(valid) *valid = false;

	uint16_t value = 0;
	uint8_t trys = max_retry_attempts;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(_timeout);
		crc_update(data);
		value = (uint16_t) data << 8;

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint16_t)data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	return false;
}
uint32_t RoboClaw::Read4(uint8_t address, uint8_t cmd, bool *valid){
	if(valid) *valid = false;

	uint32_t value = 0;
	uint8_t trys = max_retry_attempts;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(_timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}
uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid){
	if(valid) *valid = false;

	uint32_t value = 0;
	uint8_t trys = max_retry_attempts;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(_timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			if(status)
			*status = data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	return false;
}


bool RoboClaw::ForwardM1(uint8_t speed){ return write_n(3,_add,M1FORWARD,speed); }
bool RoboClaw::BackwardM1(uint8_t speed){ return write_n(3,_add,M1BACKWARD,speed); }
bool RoboClaw::SetMinVoltageMainBattery(uint8_t voltage){ return write_n(3,_add,SETMINMB,voltage); }
bool RoboClaw::SetMaxVoltageMainBattery(uint8_t voltage){ return write_n(3,_add,SETMAXMB,voltage); }
bool RoboClaw::ForwardM2(uint8_t speed){ return write_n(3,_add,M2FORWARD,speed); }
bool RoboClaw::BackwardM2(uint8_t speed){ return write_n(3,_add,M2BACKWARD,speed); }
bool RoboClaw::ForwardBackwardM1(uint8_t speed){ return write_n(3,_add,M17BIT,speed); }
bool RoboClaw::ForwardBackwardM2(uint8_t speed){ return write_n(3,_add,M27BIT,speed); }
bool RoboClaw::ForwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDFORWARD,speed); }
bool RoboClaw::BackwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDBACKWARD,speed); }
bool RoboClaw::TurnRightMixed(uint8_t speed){ return write_n(3,_add,MIXEDRIGHT,speed); }
bool RoboClaw::TurnLeftMixed(uint8_t speed){ return write_n(3,_add,MIXEDLEFT,speed); }
bool RoboClaw::ForwardBackwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDFB,speed); }
bool RoboClaw::LeftRightMixed(uint8_t speed){ return write_n(3,_add,MIXEDLR,speed); }
uint32_t RoboClaw::ReadEncM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1ENC,status,valid); }
uint32_t RoboClaw::ReadEncM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2ENC,status,valid); }
uint32_t RoboClaw::ReadSpeedM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1SPEED,status,valid); }
uint32_t RoboClaw::ReadSpeedM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2SPEED,status,valid); }
bool RoboClaw::ResetEncoders(){ return write_n(2,_add,RESETENC); }

bool RoboClaw::ReadVersion(char *version){
	int8_t data;
	uint8_t trys=max_retry_attempts;
	do{
		flush();
		data = 0;

		crc_clear();
		write(_add);
		crc_update(_add);
		write(GETVERSION);
		crc_update(GETVERSION);

		uint8_t i;
		for(i=0;i<48;i++){
			if(data!=-1){
				data=read(_timeout);
				version[i] = data;
				crc_update(version[i]);
				if(version[i]==0){
					uint16_t ccrc;
					data = read(_timeout);
					if(data!=-1){
						ccrc = data << 8;
						data = read(_timeout);
						if(data!=-1){
							ccrc |= data;
							return crc_get()==ccrc;
						}
					}
					break;
				}
			}
			else break;
		}
	}while(trys--);

	return false;
}

bool RoboClaw::SetEncM1(int32_t val){ return write_n(6,_add,SETM1ENCCOUNT,SetDWORDval(val)); }
bool RoboClaw::SetEncM2(int32_t val){ return write_n(6,_add,SETM2ENCCOUNT,SetDWORDval(val)); }
uint16_t RoboClaw::ReadMainBatteryVoltage(bool *valid){ return Read2(_add,GETMBATT,valid); }
uint16_t RoboClaw::ReadLogicBatteryVoltage(bool *valid){ return Read2(_add,GETLBATT,valid); }
bool RoboClaw::SetMinVoltageLogicBattery(uint8_t voltage){ return write_n(3,_add,SETMINLB,voltage); }
bool RoboClaw::SetMaxVoltageLogicBattery(uint8_t voltage){ return write_n(3,_add,SETMAXLB,voltage); }

bool RoboClaw::SetM1VelocityPID(float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,_add,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClaw::SetM2VelocityPID(float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,_add,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1ISPEED,status,valid); }
uint32_t RoboClaw::ReadISpeedM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2ISPEED,status,valid); }
bool RoboClaw::DutyM1(uint16_t duty){ return write_n(4,_add,M1DUTY,SetWORDval(duty)); }
bool RoboClaw::DutyM2(uint16_t duty){ return write_n(4,_add,M2DUTY,SetWORDval(duty)); }
bool RoboClaw::DutyM1M2(uint16_t duty1, uint16_t duty2){ return write_n(6,_add,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2)); }
bool RoboClaw::SpeedM1(uint32_t speed){ return write_n(6,_add,M1SPEED,SetDWORDval(speed)); }
bool RoboClaw::SpeedM2(uint32_t speed){ return write_n(6,_add,M2SPEED,SetDWORDval(speed)); }
bool RoboClaw::SpeedM1M2(uint32_t speed1, uint32_t speed2){ return write_n(10,_add,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2)); }
bool RoboClaw::SpeedAccelM1(uint32_t accel, uint32_t speed){ return write_n(10,_add,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); }
bool RoboClaw::SpeedAccelM2(uint32_t accel, uint32_t speed){ return write_n(10,_add,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); }
bool RoboClaw::SpeedAccelM1M2(uint32_t accel, uint32_t speed1, uint32_t speed2){ return write_n(14,_add,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2)); }
bool RoboClaw::SpeedDistanceM1(uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(11,_add,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboClaw::SpeedDistanceM2(uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(11,_add,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboClaw::SpeedDistanceM1M2(uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){ return write_n(19,_add,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); }
bool RoboClaw::SpeedAccelDistanceM1(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(15,_add,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboClaw::SpeedAccelDistanceM2(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(15,_add,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboClaw::SpeedAccelDistanceM1M2(uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){ return write_n(23,_add,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); }

bool RoboClaw::ReadBuffers(uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(_add,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

bool RoboClaw::ReadPWMs(int16_t &pwm1, int16_t &pwm2){
	bool valid;
	uint32_t value = Read4(_add,GETPWMS,&valid);
	if(valid){
		pwm1 = value>>16;
		pwm2 = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::ReadCurrents(int16_t &current1, int16_t &current2){
	bool valid;
	uint32_t value = Read4(_add,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
	return valid;
}
bool RoboClaw::SpeedAccelM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,_add,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}
bool RoboClaw::SpeedAccelDistanceM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,_add,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}
bool RoboClaw::DutyAccelM1(uint16_t duty, uint32_t accel){ return write_n(8,_add,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel)); }
bool RoboClaw::DutyAccelM2(uint16_t duty, uint32_t accel){ return write_n(8,_add,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel)); }
bool RoboClaw::DutyAccelM1M2(uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){ return write_n(14,_add,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2)); }

/** DEPRECATED:
bool RoboClaw::ReadM1VelocityPID(float* Kp_fp,float* Ki_fp,float* Kd_fp,uint32_t* qpps){
	uint32_t Kp,Ki,Kd;
	uint32_t _qpps;
	bool valid = read_n(4,_add,READM1PID,&Kp,&Ki,&Kd,&_qpps);
	float _Kp_fp = ((float)Kp)/65536;
	float _Ki_fp = ((float)Ki)/65536;
	float _Kd_fp = ((float)Kd)/65536;
	if(*Kp_fp) *Kp_fp = _Kp_fp;
	if(*Ki_fp) *Ki_fp = _Ki_fp;
	if(*Kd_fp) *Kd_fp = _Kd_fp;
	return valid;
}
bool RoboClaw::ReadM2VelocityPID(float* Kp_fp,float* Ki_fp,float* Kd_fp,uint32_t* qpps){
	uint32_t Kp,Ki,Kd;
	uint32_t _qpps;
	bool valid = read_n(4,_add,READM2PID,&Kp,&Ki,&Kd,&_qpps);
	float _Kp_fp = ((float)Kp)/65536;
	float _Ki_fp = ((float)Ki)/65536;
	float _Kd_fp = ((float)Kd)/65536;
	if(*Kp_fp) *Kp_fp = _Kp_fp;
	if(*Ki_fp) *Ki_fp = _Ki_fp;
	if(*Kd_fp) *Kd_fp = _Kd_fp;
	return valid;
}
*/
bool RoboClaw::ReadM1VelocityPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,_add,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}
bool RoboClaw::ReadM2VelocityPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,_add,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw::SetMainVoltages(uint16_t min,uint16_t max){ return write_n(6,_add,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max)); }
bool RoboClaw::SetLogicVoltages(uint16_t min,uint16_t max){ return write_n(6,_add,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max)); }

bool RoboClaw::ReadMinMaxMainVoltages(uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(_add,GETMINMAXMAINVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::ReadMinMaxLogicVoltages(uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(_add,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::SetM1PositionPID(float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,_add,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::SetM2PositionPID(float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,_add,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::ReadM1PositionPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,_add,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw::ReadM2PositionPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,_add,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw::SpeedAccelDeccelPositionM1(uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,_add,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}
bool RoboClaw::SpeedAccelDeccelPositionM2(uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,_add,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}
bool RoboClaw::SpeedAccelDeccelPositionM1M2(uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,_add,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClaw::SetM1DefaultAccel(uint32_t accel){ return write_n(6,_add,SETM1DEFAULTACCEL,SetDWORDval(accel)); }
bool RoboClaw::SetM2DefaultAccel(uint32_t accel){ return write_n(6,_add,SETM2DEFAULTACCEL,SetDWORDval(accel)); }
bool RoboClaw::SetPinFunctions(uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){ return write_n(5,_add,SETPINFUNCTIONS,S3mode,S4mode,S5mode); }
bool RoboClaw::GetPinFunctions(uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
	uint8_t val1,val2,val3;
	uint8_t trys=max_retry_attempts;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(_add);
		crc_update(_add);
		write(GETPINFUNCTIONS);
		crc_update(GETPINFUNCTIONS);

		data = read(_timeout);
		crc_update(data);
		val1=data;

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			val2=data;
		}

		if(data!=-1){
			data = read(_timeout);
			crc_update(data);
			val3=data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(_timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(_timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						S3mode = val1;
						S4mode = val2;
						S5mode = val3;
						return true;
					}
				}
			}
		}
	}while(trys--);
	return false;
}

bool RoboClaw::SetDeadBand(uint8_t Min, uint8_t Max){ return write_n(4,_add,SETDEADBAND,Min,Max); }
bool RoboClaw::GetDeadBand(uint8_t &Min, uint8_t &Max){
	bool valid;
	uint16_t value = Read2(_add,GETDEADBAND,&valid);
	if(valid){
		Min = value>>8;
		Max = value;
	}
	return valid;
}

bool RoboClaw::ReadEncoders(uint32_t &enc1,uint32_t &enc2){
	bool valid = read_n(2,_add,GETENCODERS,&enc1,&enc2);
	return valid;
}

bool RoboClaw::ReadISpeeds(uint32_t &ispeed1,uint32_t &ispeed2){
	bool valid = read_n(2,_add,GETISPEEDS,&ispeed1,&ispeed2);
	return valid;
}

bool RoboClaw::RestoreDefaults(){
	return write_n(2,_add,RESTOREDEFAULTS);
}

bool RoboClaw::ReadTemp(uint16_t &temp){
	bool valid;
	temp = Read2(_add,GETTEMP,&valid);
	return valid;
}

bool RoboClaw::ReadTemp2(uint16_t &temp){
	bool valid;
	temp = Read2(_add,GETTEMP2,&valid);
	return valid;
}

uint16_t RoboClaw::ReadError(bool *valid){ return Read2(_add,GETERROR,valid); }

bool RoboClaw::ReadEncoderModes(uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(_add,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

bool RoboClaw::SetM1EncoderMode(uint8_t mode){ return write_n(3,_add,SETM1ENCODERMODE,mode); }
bool RoboClaw::SetM2EncoderMode(uint8_t mode){ return write_n(3,_add,SETM2ENCODERMODE,mode); }
bool RoboClaw::WriteNVM(){ return write_n(6,_add,WRITENVM, SetDWORDval(0xE22EAB7A) ); }
bool RoboClaw::ReadNVM(){ return write_n(2,_add,READNVM); }

bool RoboClaw::SetConfig(uint16_t config){ return write_n(4,_add,SETCONFIG,SetWORDval(config)); }
bool RoboClaw::GetConfig(uint16_t &config){
	bool valid;
	uint16_t value = Read2(_add,GETCONFIG,&valid);
	if(valid) config = value;
	return valid;
}

bool RoboClaw::SetM1MaxCurrent(uint32_t max){ return write_n(10,_add,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0)); }
bool RoboClaw::SetM2MaxCurrent(uint32_t max){ return write_n(10,_add,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0)); }

bool RoboClaw::ReadM1MaxCurrent(uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,_add,GETM1MAXCURRENT,&tmax,&dummy);
	if(valid) max = tmax;
	return valid;
}

bool RoboClaw::ReadM2MaxCurrent(uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,_add,GETM2MAXCURRENT,&tmax,&dummy);
	if(valid) max = tmax;
	return valid;
}

bool RoboClaw::SetPWMMode(uint8_t mode){ return write_n(3,_add,SETPWMMODE,mode); }
bool RoboClaw::GetPWMMode(uint8_t &mode){
	bool valid;
	uint8_t value = Read1(_add,GETPWMMODE,&valid);
	if(valid){ mode = value; }
	return valid;
}
