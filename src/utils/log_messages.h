/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file sdlog2_messages.h
 *
 * Log messages and structures definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Roman Bapst <roman@px4.io>
 */

#ifndef SDLOG2_MESSAGES_H_
#define SDLOG2_MESSAGES_H_

#define LOG_PACKET_HEADER_LEN	   3
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msg_type;
#define LOG_PACKET_HEADER_INIT(id) .head1 = HEAD_BYTE1, .head2 = HEAD_BYTE2, .msg_type = id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

struct log_format_s {
	uint8_t type;
	uint8_t length;		// full packet length including header
	char name[5];
	char format[16];
	char labels[64];
};

#define LOG_FORMAT(_name, _format, _labels) { \
		.type = LOG_##_name##_MSG, \
			.length = sizeof(struct log_##_name##_s), \
				  .name = #_name, \
					  .format = _format, \
						    .labels = _labels \
	}


#define LOG_FORMAT_MSG	  0x80

#define LOG_PACKET_SIZE(_name)	LOG_PACKET_HEADER_LEN + sizeof(struct log_##_name##_s)

/* define message formats */

enum msg_id
{
    LOG_ATT_MSG = 0,
    LOG_IMU_MSG,
    LOG_SENS_MSG,
};


#pragma pack(push, 1)

struct log_ATT_s {
	LOG_PACKET_HEADER
	float roll;
	float pitch;
	float yaw;
	float roll_sp;
	float pitch_sp;
	float yaw_sp;    
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float roll_rate_sp;
	float pitch_rate_sp;
	float yaw_rate_sp;    
};


struct log_IMU_s {
	LOG_PACKET_HEADER
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float temp_acc;
	float temp_gyro;
	float temp_mag;
};


struct log_SENS_s {
	LOG_PACKET_HEADER
	float baro_pres;
	float baro_alt;
	float baro_temp;
};

///* --- LPOS - LOCAL POSITION --- */
//#define LOG_LPOS_MSG 6
//struct log_LPOS_s {
//  LOG_PACKET_HEADER
//	float x;
//	float y;
//	float z;
//	float x_sp;
//	float y_sp;
//	float z_sp;
//	float vx;
//	float vy;
//	float vz;
//	float vx_sp;
//	float vy_sp;
//	float vz_sp;
//	float ground_dist;
//};



#pragma pack(pop)


/* construct list of all message formats */
static const struct log_format_s log_formats[] = {
	LOG_FORMAT(ATT, "ffffffffffff",	"r,p,y,rsp,psp,ysp,rr,pr,yr,rrs,prs,yrs"),
	LOG_FORMAT(IMU, "ffffffffffff", "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM"),
	LOG_FORMAT(SENS, "fff", "BaroPres,BaroAlt,BaroTemp"),
//	LOG_FORMAT(LPOS, "fffffffffffff", "x,y,z,xsp,ysp,zsp,vx,vy,vz,vxs,vys,vzs,Dist"),
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);


#endif /* SDLOG2_MESSAGES_H_ */
