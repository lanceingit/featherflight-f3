#include "att_est_q.h"
#include "timer.h"
#include "geo.h"
#include <cmath>
#include <string>

#include "copter.h"
//#include "eprintf.h"

//using namespace matrix;
using math::Vector;
using math::Matrix;
using math::Quaternion;

extern COPTER copter;

#if 0
#define LINK_DEBUG(a) _link->send_text(a)
#else
#define LINK_DEBUG(a)
#endif

Att_Est_Q::Att_Est_Q(SENSORS* sensor) :
	_lp_accel_x(625.0f, 30.0f),
	_lp_accel_y(625.0f, 30.0f),
	_lp_accel_z(625.0f, 30.0f),
	_lp_gyro_x(625.0f, 30.0f),
	_lp_gyro_y(625.0f, 30.0f),
	_lp_gyro_z(625.0f, 30.0f),
	_use_compass(false),
	_mag_decl_auto(true),
	_mag_decl(0.0f),
	_last_time(0),
	_dt_max(0.02f),
	_bias_max(10.05f),
	_w_accel(0.2f),
	_w_mag(0.1f),
	_w_gyro_bias(0.1f),
//	_link(link_mavlink),
    _sensors(sensor),
	_inited(false)
{
}


Att_Est_Q::~Att_Est_Q()
{
}

bool Att_Est_Q::init()
{
	//char buf[100];

	_accel.data[0] = _sensors->inertialSensor->get_acc_x();
	_accel.data[1] = _sensors->inertialSensor->get_acc_y();
	_accel.data[2] = _sensors->inertialSensor->get_acc_z();


	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector<3> k = -_accel;
	k.normalize();

	if (_accel.length() < 0.01f || _accel.length() > 12) {
		LINK_DEBUG("init: degenerate accel!");
	}

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
    Vector<3> i = {1, 0 ,0};

	if(_use_compass)
    {
        _mag.data[0] = _sensors->compass->get_mag_x();
        _mag.data[1] = _sensors->compass->get_mag_y();
        _mag.data[2] = _sensors->compass->get_mag_z();
        //esprintf(buf, "mag0:%.3f 1:%.3f 2:%.3f", (double)_mag(0),(double)_mag(1),(double)_mag(2));
        LINK_DEBUG(buf);
        if (_mag.length() < 0.01f) {
            LINK_DEBUG("init: degenerate mag!");
        }
        
        i = (_mag - k * (_mag * k));
        i.normalize();        
    }
    


	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector<3> j = k % i;

	// Fill rotation matrix
	Matrix<3, 3> R;
	R.set_row(0, i);
	R.set_row(1, j);
	R.set_row(2, k);

	// Convert to quaternion
	_q.from_dcm(R);

	// Compensate for magnetic declination
	Quaternion decl_rotation;
	decl_rotation.from_yaw(_mag_decl);
	_q = decl_rotation * _q;

	_q.normalize();

	if (std::isfinite(_q(0)) && std::isfinite(_q(1)) &&
			std::isfinite(_q(2)) && std::isfinite(_q(3)) &&
	    _q.length() > 0.95f && _q.length() < 1.05f) {

		_inited = true;
	}
	else
	{
		_inited = false;
		LINK_DEBUG("q init definite");
	}

	return _inited;
}

void Att_Est_Q::run()
{
	if(_sensors->inertialSensor->ready())
	{
//		_gyro.data[0] = _sensors->inertialSensor->get_gyro_x();
//		_gyro.data[1] = _sensors->inertialSensor->get_gyro_y();
//		_gyro.data[2] = _sensors->inertialSensor->get_gyro_z();
		_gyro.data[0] = _lp_gyro_x.apply(_sensors->inertialSensor->get_gyro_x());
		_gyro.data[1] = _lp_gyro_y.apply(_sensors->inertialSensor->get_gyro_y());
		_gyro.data[2] = _lp_gyro_z.apply(_sensors->inertialSensor->get_gyro_z());

//		_accel.data[0] = _sensors->inertialSensor->get_acc_x();
//		_accel.data[1] = _sensors->inertialSensor->get_acc_y();
//		_accel.data[2] = _sensors->inertialSensor->get_acc_z();
		_accel.data[0] = _lp_accel_x.apply(_sensors->inertialSensor->get_acc_x());
		_accel.data[1] = _lp_accel_y.apply(_sensors->inertialSensor->get_acc_y());
		_accel.data[2] = _lp_accel_z.apply(_sensors->inertialSensor->get_acc_z());

		if (_accel.length() < 0.01f) {
			LINK_DEBUG("WARNING: degenerate accel!");
			return;
		}
	}
	else
	{
		return;
	}


	if(_use_compass)
	{
		_mag.data[0] = _sensors->compass->get_mag_x();
		_mag.data[1] = _sensors->compass->get_mag_y();
		_mag.data[2] = _sensors->compass->get_mag_z();

		if (_mag.length() < 0.01f) {
			//PX4_DEBUG("WARNING: degenerate mag!");
			LINK_DEBUG("WARNING: degenerate mag!");
			return;
		}

//		if (_mag_decl_auto && _sensors.gps.eph < 20.0f) {
//			/* set magnetic declination automatically */
//			//update_mag_declination(math::radians(get_mag_declination(_sensors.gps.lat, _sensors.gps.lon)));
//		}
	}

	/* time from previous iteration */
	uint64_t now = Timer_getTime();
	float dt = (_last_time > 0) ? ((now  - _last_time) / 1000000.0f) : 0.00001f;
	_last_time = now;

	if (dt > _dt_max) {
		dt = _dt_max;
	}

	if (!update(dt)) {
//		LINK_DEBUG("att update error");
		return;
	}
//	else
//	{
//		LINK_DEBUG("att update");
//	}

	{
		Vector<3> euler = _q.to_euler();
		//matrix::Eulerf euler = matrix::Quatf(_q);

		_roll_rate = _rates(0);
		_pitch_rate = _rates(1);
		_yaw_rate = _rates(2);

	    _roll = euler(0);
	    _pitch = euler(1);
	    _yaw = euler(2);

//	    esprintf(buf, "r:%.2f p:%.2f y:%.2f", (double)_roll, (double)_pitch, (double)_yaw);
//	    LINK_DEBUG(buf);
	}

}


void Att_Est_Q::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	if (fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		Quaternion decl_rotation;
		decl_rotation.from_yaw(new_declination - _mag_decl);
		_q = decl_rotation * _q;
		_mag_decl = new_declination;
	}
}


bool Att_Est_Q::update(float dt)
{
    
    
	if (!_inited) {

		return init();
	}

	Quaternion q_last = _q;

	// Angular rate of correction
	Vector<3> corr;
	float spinRate = _gyro.length();

	if (_use_compass) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector<3> mag_earth = _q.conjugate(_mag);
		float mag_err = _wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
//		float gainMult = 1.0f;
//		const float fifty_dps = 0.873f;
//
//		if (spinRate > fifty_dps) {
//			gainMult = math::min(spinRate / fifty_dps, 10.0f);
//		}

		// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mag_err)) * _w_mag;
	}

	_q.normalize();


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector<3> k = _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector<3> k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);



	corr += (k % _accel.normalized()) * _w_accel;
	//_corr_acc = corr;

	// Gyro bias estimation
	if (spinRate < 0.175f) {
		_gyro_bias += corr * (_w_gyro_bias * dt);

		for (int i = 0; i < 3; i++) {
			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}

	}

	_rates = _gyro + _gyro_bias;

	// Feed forward gyro
	corr += _rates;
	//_corr_all = corr;

	// Apply correction to state
	_q += _q.derivative(corr) * dt;

	// Normalize quaternion
	_q.normalize();

	if (!(std::isfinite(_q(0)) && std::isfinite(_q(1)) &&
			std::isfinite(_q(2)) && std::isfinite(_q(3)))) {
		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		LINK_DEBUG("q definite");
		return false;
	}

	return true;
}

