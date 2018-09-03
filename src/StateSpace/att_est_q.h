#pragma once

#include "mathlib.h"
#include "LowPassFilter2p.hpp"
#include "link_mavlink.h"
#include "sensors.h"


class Att_Est_Q
{
public:
	Att_Est_Q(SENSORS* sensor);
	~Att_Est_Q();

	bool init();
	void run();

//	bool is_init() {return _inited;}

    float get_roll() { return _roll; }
    float get_pitch() { return _pitch; }
    float get_yaw() { return _yaw; }

    float get_roll_rate() { return _roll_rate; }
    float get_pitch_rate() { return _pitch_rate; }
    float get_yaw_rate() { return _yaw_rate; }

    void get_q(math::Quaternion &q) { q = _q; }
    
    float get_bias_x() {return _gyro_bias(0);}
    float get_bias_y() {return _gyro_bias(1);}
    float get_bias_z() {return _gyro_bias(2);}

    float get_corr_acc_x() {return _corr_acc(0);}
    float get_corr_acc_y() {return _corr_acc(1);}
    float get_corr_all_x() {return _corr_all(0);}
    float get_corr_all_y() {return _corr_all(1);}


private:
    float _roll;
    float _pitch;
    float _yaw;

    float _roll_rate;
    float _pitch_rate;
    float _yaw_rate;

    math::Quaternion	_q;

	math::Vector<3>	_gyro;
	math::Vector<3>	_accel;
	math::Vector<3>	_mag;

	math::Vector<3>	_gyro_bias;
	math::Vector<3>	_rates;
	math::Vector<3> _corr_acc;     ///for test
	math::Vector<3> _corr_all;     ///for test

	math::LowPassFilter2p	_lp_accel_x;
	math::LowPassFilter2p	_lp_accel_y;
	math::LowPassFilter2p	_lp_accel_z;
	math::LowPassFilter2p	_lp_gyro_x;
	math::LowPassFilter2p	_lp_gyro_y;
	math::LowPassFilter2p	_lp_gyro_z;

	bool _use_compass;
	bool _mag_decl_auto;
	float _mag_decl;
	uint64_t _last_time;
	const float _dt_max;
	const float _bias_max;
	const float _w_accel;
	const float _w_mag;
	const float _w_gyro_bias;

	LINK_MAVLINK* _link;
    SENSORS* _sensors;

	bool _inited;

	void update_mag_declination(float new_declination);
	bool update(float dt);
};

