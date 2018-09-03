#pragma once

#include "LowPassFilter2p.hpp"
#include "rotation.h"


class InertialSensor
{

public:
	InertialSensor(uint16_t accel_rate, uint16_t accel_freq,
			       uint16_t gyro_rate, uint16_t gyro_freq,
				   enum Rotation rotation):
	_accel_filter_x(accel_rate, accel_freq),
	_accel_filter_y(accel_rate, accel_freq),
	_accel_filter_z(accel_rate, accel_freq),
	_gyro_filter_x(gyro_rate, gyro_freq),
	_gyro_filter_y(gyro_rate, gyro_freq),
	_gyro_filter_z(gyro_rate, gyro_freq),
	_ready(false),
	_rotation(rotation),
	_update(false)
	{}

	~InertialSensor() {}

	virtual bool init(void)=0;
	virtual void read()=0;

    void get_accel(float *acc) { acc[0] = _acc[0]; acc[1] = _acc[1]; acc[2] = _acc[2];}
    void get_gyro(float *gyro) {gyro[0] = _gyro[0]; gyro[1] = _gyro[1]; gyro[2] = _gyro[2];}

    float get_acc_x() {return _acc[0];}
    float get_acc_y() {return _acc[1];}
    float get_acc_z() {return _acc[2];}

    float get_gyro_x() {return _gyro[0];}
    float get_gyro_y() {return _gyro[1];}
    float get_gyro_z() {return _gyro[2];}

    void set_accel(float *acc) { _acc[0] = acc[0]; _acc[1] = acc[1]; _acc[2] = acc[2];}
    void set_gyro(float *gyro) {_gyro[0] = gyro[0]; _gyro[1] = gyro[1]; _gyro[2] = gyro[2];}

    void set_acc_x(float acc) {_acc[0] = acc;}
    void set_acc_y(float acc) {_acc[1] = acc;}
    void set_acc_z(float acc) {_acc[2] = acc;}

    void set_gyro_x(float gyro) {_gyro[0] = gyro;}
    void set_gyro_y(float gyro) {_gyro[1] = gyro;}
    void set_gyro_z(float gyro) {_gyro[2] = gyro;}

    //void set_gyro_offset(float offset[3]) {_gyro_offset = offset}
    void set_gyro_offset_x(float offset) {_gyro_offset[0] = offset;}
    void set_gyro_offset_y(float offset) {_gyro_offset[1] = offset;}
    void set_gyro_offset_z(float offset) {_gyro_offset[2] = offset;}

    bool ready() { return _ready;}
    //void set_ready() {_ready=true;}

    bool update() { return _update;}
    void clean_update() {_update = false;}

protected:
    float _acc[3];
    float _gyro_raw[3];
    float _gyro[3];
    float _gyro_offset[3];

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

    bool _ready;
    enum Rotation _rotation;
    bool _update;

private:

};
