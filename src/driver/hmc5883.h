#pragma once

#include "i2c.h"
#include "compass.h"

class HMC5883 : public Compass
{
   
public:    
    HMC5883();
    bool init(void);
    void read(void);

//    float get_magx() {return _mag[0];}
//    float get_magy() {return _mag[1];}
//    float get_magz() {return _mag[2];}

private:

    #define MAG_ADDRESS 0x1E
    #define MAG_DATA_REGISTER 0x03
    #define HMC58X3_R_CONFA 0
    #define HMC58X3_R_CONFB 1
    #define HMC58X3_R_MODE 2
    #define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
    #define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
    #define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
    #define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
    #define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
    #define HMC_POS_BIAS 1
    #define HMC_NEG_BIAS 2

//    float _mag[3];

    I2C* _i2c; 
        
};
