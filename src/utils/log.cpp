#include <stdint.h>

#include "log.h"
#include "log_messages.h"
#include "timer.h"


LOG::LOG(MTD* mtd):
	_mtd(mtd),
	_record(true)
{

}

void LOG::init()
{
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER;
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	/* fill message format packet for each format and write it */
	for (unsigned i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		write(&log_msg_format, sizeof(log_msg_format));
	}
}

void LOG::write(void* pkt, uint16_t len)
{
//.    _mtd->write((uint8_t*)pkt, len);
}

uint16_t LOG::read(uint32_t offset, uint8_t* data, uint16_t len)
{
	return _mtd->read(offset, data, len);
}

void LOG::write_att(Att_Est_Q &att, uint16_t rate)
{
    struct log_ATT_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_ATT_MSG),
		.roll = att.get_roll(),
		.pitch = att.get_pitch(),
		.yaw = att.get_yaw(),
		.roll_sp = 0.0f,
		.pitch_sp = 0.0f,
		.yaw_sp = 0.0f,
		.roll_rate = att.get_roll_rate(),
		.pitch_rate = att.get_pitch_rate(),
		.yaw_rate = att.get_yaw_rate(),
		.roll_rate_sp = 0.0f,
		.pitch_rate_sp = 0.0f,
		.yaw_rate_sp = 0.0f,
    };
    
    if(Timer_elapsedTime(&_timer[LOG_ATT_MSG]) > 1*1000*1000/rate)
    {
        _timer[LOG_ATT_MSG] = Timer_getTime();
        write(&pkt, sizeof(pkt));
    }
    
}

void LOG::write_imu(SENSORS &sens, uint16_t rate)
{
    struct log_IMU_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
		.acc_x = sens.inertialSensor->get_acc_x(),
		.acc_y = sens.inertialSensor->get_acc_y(),
		.acc_z = sens.inertialSensor->get_acc_z(),
		.gyro_x = sens.inertialSensor->get_gyro_x(),
		.gyro_y = sens.inertialSensor->get_gyro_y(),
		.gyro_z = sens.inertialSensor->get_gyro_z(),
		.mag_x = sens.compass->get_mag_x(),
		.mag_y = sens.compass->get_mag_y(),
		.mag_z = sens.compass->get_mag_z(),
		.temp_acc = 0.0f,
		.temp_gyro = 0.0f,
		.temp_mag = 0.0f,
    };
    if(Timer_elapsedTime(&_timer[LOG_IMU_MSG]) > 1*1000*1000/rate)
    {
        _timer[LOG_IMU_MSG] = Timer_getTime();
        write(&pkt, sizeof(pkt));
    }
}

void LOG::write_sens(SENSORS &sens, uint16_t rate)
{
    struct log_SENS_s pkt = {
    	LOG_PACKET_HEADER_INIT(LOG_SENS_MSG),
		.baro_pres = sens.baro->get_press(),
		.baro_alt = sens.baro->get_altitude(),
		.baro_temp = sens.baro->get_temp(),
    };
    if(Timer_elapsedTime(&_timer[LOG_SENS_MSG]) > 1*1000*1000/rate)
    {
        _timer[LOG_SENS_MSG] = Timer_getTime();
        write(&pkt, sizeof(pkt));
    }
}











