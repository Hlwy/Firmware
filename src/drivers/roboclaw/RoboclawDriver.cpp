#include "RoboclawDriver.h"

// px4 added
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <systemlib/err.h> // errx
#include <unistd.h> // usleep
#include <string.h>
#include <drivers/drv_hrt.h>

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

int RoboclawDriver::transaction(e_command cmd, uint8_t *wbuff, size_t wbytes,
			   uint8_t *rbuff, size_t rbytes, bool send_checksum, bool recv_checksum)
{
	int err_code = 0;

	// WRITE

	tcflush(uart, TCIFLUSH); // flush  buffers
	uint8_t buf[wbytes + 4];
	buf[0] = (uint8_t) _add;
	buf[1] = cmd;

	if (wbuff) {
		memcpy(&buf[2], wbuff, wbytes);
	}

	wbytes = wbytes + (send_checksum ? 4 : 2);

	if (send_checksum) {
		uint16_t sum = calcCRC(buf, wbytes - 2);
		buf[wbytes - 2] = (sum >> 8) & 0xFF;
		buf[wbytes - 1] = sum & 0xFFu;
	}

	int count = ::write(uart, buf, wbytes);

	if (count < (int) wbytes) { // Did not successfully send all bytes.
		PX4_ERR("Only wrote %d out of %d bytes", count, (int) wbytes);
		return -1;
	}

	// READ

	FD_ZERO(&_uart_set);
	FD_SET(uart, &_uart_set);

	uint8_t *rbuff_curr = rbuff;
	size_t bytes_read = 0;

	// select(...) returns as soon as even 1 byte is available. read(...) returns immediately, no matter how many
	// bytes are available. I need to keep reading until I get the number of bytes I expect.
	while (bytes_read < rbytes) {
		// select(...) may change this timeout struct (because it is not const). So I reset it every time.
		_uart_timeout.tv_sec = 0;
		_uart_timeout.tv_usec = 10500;
		err_code = select(uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout);

		// An error code of 0 means that select timed out, which is how the Roboclaw indicates an error.
		if (err_code <= 0) {
			return err_code;
		}

		err_code = ::read(uart, rbuff_curr, rbytes - bytes_read);

		if (err_code <= 0) {
			return err_code;

		} else {
			bytes_read += err_code;
			rbuff_curr += err_code;
		}
	}

	//TODO: Clean up this mess of IFs and returns

	if (recv_checksum) {
		if (bytes_read < 2) {
			return -1;
		}

		// The checksum sent back by the roboclaw is calculated based on the address and command bytes as well
		// as the data returned.
		uint16_t checksum_calc = calcCRC(buf, 2);
		checksum_calc = calcCRC(rbuff, bytes_read - 2, checksum_calc);
		uint16_t checksum_recv = (rbuff[bytes_read - 2] << 8) + rbuff[bytes_read - 1];

		if (checksum_calc == checksum_recv) {
			return bytes_read;

		} else {
			return -10;
		}

	} else {
		if (bytes_read == 1 && rbuff[0] == 0xFF) {
			return 1;

		} else {
			return -11;
		}
	}
}
uint16_t RoboclawDriver::calcCRC(const uint8_t *buf, size_t n, uint16_t init)
{
	uint16_t crc = init;

	for (size_t byte = 0; byte < n; byte++) {
		crc = crc ^ (((uint16_t) buf[byte]) << 8);

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

RoboclawDriver::RoboclawDriver() : _uart_set(), _uart_timeout{.tv_sec = 0, .tv_usec = 10500} {}
RoboclawDriver::RoboclawDriver(const char *port, uint8_t address, int baud, uint32_t tout, bool doack) :
     _uart_set(), _uart_timeout{.tv_sec = 0, .tv_usec = 10500}
{
     ack = doack;
     _add = address;
	_timeout = tout;

     int err = open_serial_port(port, baud);
     switch(err){
          case -1:{ printf("failed to get attr\r\n"); exit(err); }
          case -2:{ printf("failed to set input speed\r\n"); exit(err); }
          case -3:{ printf("failed to set output speed\r\n"); exit(err); }
          case -4:{ printf("failed to set attr\r\n"); exit(err); }
     }

	// setup uart polling
	uartPoll[0].fd = uart;
	uartPoll[0].events = POLLIN;
}
RoboclawDriver::~RoboclawDriver(){ close(uart); }

int RoboclawDriver::init_serial(const char *port, uint8_t address, int baud, uint32_t tout, bool doack){
     ack = doack;
     _add = address;
	_timeout = tout;

     int err = open_serial_port(port, baud);
     switch(err){
          case -1:{ printf("failed to get attr\r\n"); return err; }
          case -2:{ printf("failed to set input speed\r\n"); return err; }
          case -3:{ printf("failed to set output speed\r\n"); return err; }
          case -4:{ printf("failed to set attr\r\n"); return err; }
     }

	// setup uart polling
	uartPoll[0].fd = uart;
	uartPoll[0].events = POLLIN;
     return 0;
}
int RoboclawDriver::open_serial_port(const char *port, int baud){
     speed_t baudRate;
     switch (baud) {
     	case 2400:{   baudRate = B2400; break; }
     	case 9600:{   baudRate = B9600; break; }
     	case 19200:{  baudRate = B19200; break; }
     	case 38400:{  baudRate = B38400; break; }
     	case 57600:{  baudRate = B57600; break; }
     	case 115200:{ baudRate = B115200; break; }
     	case 230400:{ baudRate = B230400; break; }
     	case 460800:{ baudRate = B460800; break; }
     	default:{     baudRate = B2400; }
	}

     uart = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	// setup uart
	struct termios uart_config;
	int ret = tcgetattr(uart, &uart_config);
	if(ret < 0){ return -1; }
	ret = cfsetispeed(&uart_config, baudRate);
	if(ret < 0){ return -2; }
	ret = cfsetospeed(&uart_config, baudRate);
	if(ret < 0){ return -3; }

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
	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = tcsetattr(uart, TCSANOW, &uart_config);
	if(ret < 0){  return -4; }
     return 0;
}
void RoboclawDriver::flush(){ tcflush(uart, TCIFLUSH); }
int RoboclawDriver::write(uint8_t byte){
	if(::write(uart, &byte, 1) < 0) { return -1; }
     else { return 1; }
}
uint8_t RoboclawDriver::read(uint32_t timeout){
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
void RoboclawDriver::crc_clear(){ crc = 0; }
void RoboclawDriver::crc_update(uint8_t data){
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++){
		if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
		else crc <<= 1;
	}
}
uint16_t RoboclawDriver::crc_get(){ return crc; }

bool RoboclawDriver::write_n(uint8_t cnt, ... ){
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
		if(read(_timeout)==0xFF) return true;
	}while(trys--);
	return false;
}
bool RoboclawDriver::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...){
	uint32_t value = 0;
	uint8_t trys = max_retry_attempts;
	do{
		crc_clear();      flush();
		write(address);   crc_update(address);
		write(cmd);       crc_update(cmd);

		//send data with crc
		va_list marker; va_start(marker, cmd);     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);

               uint8_t data = read(_timeout); crc_update(data);
               value = (uint32_t) data << 24;
               data = read(_timeout);         crc_update(data);
               value |= (uint32_t) data << 16;
               data = read(_timeout);         crc_update(data);
               value |= (uint32_t) data << 8;
               data = read(_timeout);         crc_update(data);
               value |= (uint32_t) data;
			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */
          uint8_t data = read(_timeout);
          bool success = ((crc_get() & 0x7F) == data);
		if(!success){ flush(); }
          else{ return success; }
	}while(trys--);
	return false;
}

uint8_t RoboclawDriver::Read1(uint8_t address,uint8_t cmd,bool *valid){
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
uint16_t RoboclawDriver::Read2(uint8_t address,uint8_t cmd,bool *valid){
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
uint32_t RoboclawDriver::Read4(uint8_t address, uint8_t cmd, bool *valid){
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
uint32_t RoboclawDriver::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid){
	uint8_t trys = max_retry_attempts;
	do{
          uint32_t value = 0;
		crc_clear();      flush();
		write(address);   crc_update(address);
		write(cmd);       crc_update(cmd);

		uint8_t data = read(_timeout); crc_update(data);
		value = (uint32_t) data << 24;
          data = read(_timeout);         crc_update(data);
          value |= (uint32_t) data << 16;
          data = read(_timeout);         crc_update(data);
          value |= (uint32_t) data << 8;
          data = read(_timeout);         crc_update(data);
          value |= (uint32_t) data;
          data = read(_timeout);     crc_update(data);
          if(status) *status = data;

          data = read(_timeout);
	     bool success = ((crc_get() & 0x7F) == data);
          if(success){
               if(valid) *valid = true;
               return value;
          } else{ flush(); }
	}while(trys--);
     if(valid) *valid = false;
	return false;
}

/** ===========================================================================
*
* =========================================================================== */
bool RoboclawDriver::ForwardM1(uint8_t speed){ return write_n(3,_add,M1FORWARD,speed); }
bool RoboclawDriver::BackwardM1(uint8_t speed){ return write_n(3,_add,M1BACKWARD,speed); }
bool RoboclawDriver::ForwardM2(uint8_t speed){ return write_n(3,_add,M2FORWARD,speed); }
bool RoboclawDriver::BackwardM2(uint8_t speed){ return write_n(3,_add,M2BACKWARD,speed); }
bool RoboclawDriver::ForwardBackwardM1(uint8_t speed){ return write_n(3,_add,M17BIT,speed); }
bool RoboclawDriver::ForwardBackwardM2(uint8_t speed){ return write_n(3,_add,M27BIT,speed); }
bool RoboclawDriver::ForwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDFORWARD,speed); }
bool RoboclawDriver::BackwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDBACKWARD,speed); }
bool RoboclawDriver::TurnRightMixed(uint8_t speed){ return write_n(3,_add,MIXEDRIGHT,speed); }
bool RoboclawDriver::TurnLeftMixed(uint8_t speed){ return write_n(3,_add,MIXEDLEFT,speed); }
bool RoboclawDriver::ForwardBackwardMixed(uint8_t speed){ return write_n(3,_add,MIXEDFB,speed); }
bool RoboclawDriver::LeftRightMixed(uint8_t speed){ return write_n(3,_add,MIXEDLR,speed); }

bool RoboclawDriver::SetEncM1(int32_t val){ return write_n(6,_add,SETM1ENCCOUNT,SetDWORDval(val)); }
bool RoboclawDriver::SetEncM2(int32_t val){ return write_n(6,_add,SETM2ENCCOUNT,SetDWORDval(val)); }
uint32_t RoboclawDriver::ReadEncM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1ENC,status,valid); }
uint32_t RoboclawDriver::ReadEncM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2ENC,status,valid); }
uint32_t RoboclawDriver::ReadSpeedM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1SPEED,status,valid); }
uint32_t RoboclawDriver::ReadSpeedM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2SPEED,status,valid); }
uint32_t RoboclawDriver::ReadISpeedM1(uint8_t *status,bool *valid){ return Read4_1(_add,GETM1ISPEED,status,valid); }
uint32_t RoboclawDriver::ReadISpeedM2(uint8_t *status,bool *valid){ return Read4_1(_add,GETM2ISPEED,status,valid); }
bool RoboclawDriver::ReadEncoders(uint32_t &enc1,uint32_t &enc2){
	bool valid = read_n(2,_add,GETENCODERS,&enc1,&enc2);
	return valid;
}
bool RoboclawDriver::ReadISpeeds(uint32_t &ispeed1,uint32_t &ispeed2){
	bool valid = read_n(2,_add,GETISPEEDS,&ispeed1,&ispeed2);
	return valid;
}
uint16_t RoboclawDriver::ReadError(bool *valid){ return Read2(_add,GETERROR,valid); }
bool RoboclawDriver::ResetEncoders(){ return write_n(2,_add,RESETENC); }

bool RoboclawDriver::DutyM1(uint16_t duty){ return write_n(4,_add,M1DUTY,SetWORDval(duty)); }
bool RoboclawDriver::DutyM2(uint16_t duty){ return write_n(4,_add,M2DUTY,SetWORDval(duty)); }
bool RoboclawDriver::DutyM1M2(uint16_t duty1, uint16_t duty2){ return write_n(6,_add,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2)); }
bool RoboclawDriver::SpeedM1(uint32_t speed){ return write_n(6,_add,M1SPEED,SetDWORDval(speed)); }
bool RoboclawDriver::SpeedM2(uint32_t speed){ return write_n(6,_add,M2SPEED,SetDWORDval(speed)); }
bool RoboclawDriver::SpeedM1M2(uint32_t speed1, uint32_t speed2){ return write_n(10,_add,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2)); }
bool RoboclawDriver::SpeedAccelM1(uint32_t accel, uint32_t speed){ return write_n(10,_add,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); }
bool RoboclawDriver::SpeedAccelM2(uint32_t accel, uint32_t speed){ return write_n(10,_add,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); }
bool RoboclawDriver::SpeedAccelM1M2(uint32_t accel, uint32_t speed1, uint32_t speed2){ return write_n(14,_add,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2)); }
bool RoboclawDriver::SpeedDistanceM1(uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(11,_add,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboclawDriver::SpeedDistanceM2(uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(11,_add,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboclawDriver::SpeedDistanceM1M2(uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){ return write_n(19,_add,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); }
bool RoboclawDriver::SpeedAccelDistanceM1(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(15,_add,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboclawDriver::SpeedAccelDistanceM2(uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){ return write_n(15,_add,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); }
bool RoboclawDriver::SpeedAccelDistanceM1M2(uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){ return write_n(23,_add,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); }

bool RoboclawDriver::SetM1DefaultAccel(uint32_t accel){ return write_n(6,_add,SETM1DEFAULTACCEL,SetDWORDval(accel)); }
bool RoboclawDriver::SetM2DefaultAccel(uint32_t accel){ return write_n(6,_add,SETM2DEFAULTACCEL,SetDWORDval(accel)); }
bool RoboclawDriver::SpeedAccelDeccelPositionM1(uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,_add,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}
bool RoboclawDriver::SpeedAccelDeccelPositionM2(uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,_add,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}
bool RoboclawDriver::SpeedAccelDeccelPositionM1M2(uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,_add,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}
bool RoboclawDriver::SpeedAccelM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,_add,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}
bool RoboclawDriver::SpeedAccelDistanceM1M2_2(uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,_add,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}
bool RoboclawDriver::DutyAccelM1(uint16_t duty, uint32_t accel){ return write_n(8,_add,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel)); }
bool RoboclawDriver::DutyAccelM2(uint16_t duty, uint32_t accel){ return write_n(8,_add,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel)); }
bool RoboclawDriver::DutyAccelM1M2(uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){ return write_n(14,_add,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2)); }

bool RoboclawDriver::SetM1VelocityPID(float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,_add,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}
bool RoboclawDriver::SetM2VelocityPID(float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,_add,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}
bool RoboclawDriver::ReadM1VelocityPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
     uint32_t Kp,Ki,Kd;
     bool valid = read_n(4,_add,READM1PID,&Kp,&Ki,&Kd,&qpps);
     Kp_fp = ((float)Kp)/65536;
     Ki_fp = ((float)Ki)/65536;
     Kd_fp = ((float)Kd)/65536;
     return valid;
}
bool RoboclawDriver::ReadM2VelocityPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
     uint32_t Kp,Ki,Kd;
     bool valid = read_n(4,_add,READM2PID,&Kp,&Ki,&Kd,&qpps);
     Kp_fp = ((float)Kp)/65536;
     Ki_fp = ((float)Ki)/65536;
     Kd_fp = ((float)Kd)/65536;
     return valid;
}
bool RoboclawDriver::SetM1PositionPID(float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,_add,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}
bool RoboclawDriver::SetM2PositionPID(float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,_add,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}
bool RoboclawDriver::ReadM1PositionPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,_add,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}
bool RoboclawDriver::ReadM2PositionPID(float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,_add,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboclawDriver::SetMinVoltageMainBattery(uint8_t voltage){ return write_n(3,_add,SETMINMB,voltage); }
bool RoboclawDriver::SetMaxVoltageMainBattery(uint8_t voltage){ return write_n(3,_add,SETMAXMB,voltage); }
uint16_t RoboclawDriver::ReadMainBatteryVoltage(bool *valid){ return Read2(_add,GETMBATT,valid); }
uint16_t RoboclawDriver::ReadLogicBatteryVoltage(bool *valid){ return Read2(_add,GETLBATT,valid); }
bool RoboclawDriver::SetMinVoltageLogicBattery(uint8_t voltage){ return write_n(3,_add,SETMINLB,voltage); }
bool RoboclawDriver::SetMaxVoltageLogicBattery(uint8_t voltage){ return write_n(3,_add,SETMAXLB,voltage); }
bool RoboclawDriver::SetMainVoltages(uint16_t min,uint16_t max){ return write_n(6,_add,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max)); }
bool RoboclawDriver::SetLogicVoltages(uint16_t min,uint16_t max){ return write_n(6,_add,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max)); }
bool RoboclawDriver::ReadMinMaxMainVoltages(uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(_add,GETMINMAXMAINVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}
bool RoboclawDriver::ReadMinMaxLogicVoltages(uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(_add,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}
bool RoboclawDriver::SetM1MaxCurrent(uint32_t max){ return write_n(10,_add,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0)); }
bool RoboclawDriver::SetM2MaxCurrent(uint32_t max){ return write_n(10,_add,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0)); }
bool RoboclawDriver::ReadM1MaxCurrent(uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,_add,GETM1MAXCURRENT,&tmax,&dummy);
	if(valid) max = tmax;
	return valid;
}
bool RoboclawDriver::ReadM2MaxCurrent(uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,_add,GETM2MAXCURRENT,&tmax,&dummy);
	if(valid) max = tmax;
	return valid;
}
bool RoboclawDriver::ReadCurrents(int16_t &current1, int16_t &current2){
	bool valid;
	uint32_t value = Read4(_add,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
	return valid;
}

bool RoboclawDriver::SetPinFunctions(uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){ return write_n(5,_add,SETPINFUNCTIONS,S3mode,S4mode,S5mode); }
bool RoboclawDriver::SetDeadBand(uint8_t Min, uint8_t Max){ return write_n(4,_add,SETDEADBAND,Min,Max); }
bool RoboclawDriver::GetDeadBand(uint8_t &Min, uint8_t &Max){
	bool valid;
	uint16_t value = Read2(_add,GETDEADBAND,&valid);
	if(valid){
		Min = value>>8;
		Max = value;
	}
	return valid;
}
bool RoboclawDriver::ReadPWMs(int16_t &pwm1, int16_t &pwm2){
	bool valid;
	uint32_t value = Read4(_add,GETPWMS,&valid);
	if(valid){
		pwm1 = value>>16;
		pwm2 = value&0xFFFF;
	}
	return valid;
}
bool RoboclawDriver::ReadBuffers(uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(_add,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}
bool RoboclawDriver::RestoreDefaults(){ return write_n(2,_add,RESTOREDEFAULTS); }
bool RoboclawDriver::ReadTemp(uint16_t &temp){
	bool valid;
	temp = Read2(_add,GETTEMP,&valid);
	return valid;
}
bool RoboclawDriver::ReadTemp2(uint16_t &temp){
	bool valid;
	temp = Read2(_add,GETTEMP2,&valid);
	return valid;
}
bool RoboclawDriver::ReadEncoderModes(uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(_add,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}
bool RoboclawDriver::SetM1EncoderMode(uint8_t mode){ return write_n(3,_add,SETM1ENCODERMODE,mode); }
bool RoboclawDriver::SetM2EncoderMode(uint8_t mode){ return write_n(3,_add,SETM2ENCODERMODE,mode); }
bool RoboclawDriver::WriteNVM(){ return write_n(6,_add,WRITENVM, SetDWORDval(0xE22EAB7A) ); }
bool RoboclawDriver::ReadNVM(){ return write_n(2,_add,READNVM); }
bool RoboclawDriver::SetConfig(uint16_t config){ return write_n(4,_add,SETCONFIG,SetWORDval(config)); }
bool RoboclawDriver::GetConfig(uint16_t &config){
	bool valid;
	uint16_t value = Read2(_add,GETCONFIG,&valid);
	if(valid) config = value;
	return valid;
}
bool RoboclawDriver::SetPWMMode(uint8_t mode){ return write_n(3,_add,SETPWMMODE,mode); }
bool RoboclawDriver::GetPWMMode(uint8_t &mode){
	bool valid;
	uint8_t value = Read1(_add,GETPWMMODE,&valid);
	if(valid){ mode = value; }
	return valid;
}
bool RoboclawDriver::ReadVersion(char *version){
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
bool RoboclawDriver::GetPinFunctions(uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
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
