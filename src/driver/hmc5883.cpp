#include <stdbool.h>
#include <stdint.h>

#include "hmc5883.h"


HMC5883::HMC5883()
{
    _i2c = new I2C();
}



bool HMC5883::init(void)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = _i2c->read(MAG_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;
    
    
    _i2c->write(MAG_ADDRESS, HMC58X3_R_CONFB, (3 << 5));
    _i2c->write(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);
    
    return true;
   
}

void HMC5883::read(void)
{
    uint8_t buf[6];

    bool ack = _i2c->read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    if (!ack) {
        return ;
    }
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    _mag[0] = (int16_t)(buf[0] << 8 | buf[1]) * (1.0f / 660.0f);
    _mag[1] = (int16_t)(buf[2] << 8 | buf[3]) * (1.0f / 660.0f);
    _mag[2] = (int16_t)(buf[4] << 8 | buf[5]) * (1.0f / 660.0f);

}
