#pragma once

#include "inertialSensor.h"
#include "baro.h"
#include "compass.h"

class SENSORS
{

public:
	SENSORS(InertialSensor* _inertialSensor,
			Compass* _compass,
			Baro* _baro
			)
			:
			inertialSensor(_inertialSensor),
			compass(_compass),
			baro(_baro)
	{
	}
	~SENSORS(){}

	InertialSensor* inertialSensor;
//	GPS gps;
	Compass* compass;
	Baro* baro;

//	Vision vision;
//	Flow flow;
//    GroundDistance groundDistance;


private:

};
