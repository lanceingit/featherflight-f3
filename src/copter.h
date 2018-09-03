#pragma once

#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"

#include "sensors.h"

#include "link_mavlink.h"


class COPTER
{
    
public:
    
	COPTER(SENSORS* _sensors
			):
	   sensors(_sensors)
	{}
//	COPTER()
//	{}
    

//    MPU6050 mpu6050;
    MS5611 ms5611;
    HMC5883 hmc5883;
    LINK_MAVLINK link_mavlink;

    SENSORS* sensors;

private:    
    
    
};

