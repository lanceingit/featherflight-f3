#include <stdbool.h>
#include <stdint.h>

#include "stm32f30x.h"

#include "timer.h"
#include "mpu6050.h"


MPU6050::MPU6050(enum Rotation rotation):
    InertialSensor(MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ,
                   MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ,
				   rotation)
{
    _i2c = new I2C();
}


bool MPU6050::init(void)
{
    bool ack;
    uint8_t sig;
    
    _i2c->init();

    ack = _i2c->read(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig);
    if (!ack)
        return false;

    if (sig != (MPU6050_ADDRESS & 0x7e))
        return false;

    
    
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1

    Timer_delayUs(100*1000);
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x01); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    Timer_delayUs(100*1000);
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_2, 0x0); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    Timer_delayUs(100*1000);
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
     //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure
    Timer_delayUs(15*1000);
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_CONFIG, 3); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    Timer_delayUs(100*1000);
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 8g (4096 LSB/g)
    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);

//    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG2, 3);
    
//    ack = _i2c->write(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG,
//            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
//    
//    
    
    
    _ready = true;
    
    return true;
}



void MPU6050::read()
{
    uint8_t buf[14];


    if (!_i2c->read(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 14, buf)) {
        return;
    }

    float x_in_new  = (float)((int16_t)((buf[0] << 8) | buf[1]))*(9.80665f /2048);
    float y_in_new = (float)((int16_t)((buf[2] << 8) | buf[3]))*(9.80665f /2048);
    float z_in_new = (float)((int16_t)((buf[4] << 8) | buf[5]))*(9.80665f /2048);
    

    rotate_3f(_rotation, x_in_new, y_in_new, z_in_new);

    _acc[0] = _accel_filter_x.apply(x_in_new);
    _acc[1] = _accel_filter_y.apply(y_in_new);
    _acc[2] = _accel_filter_z.apply(z_in_new);
    
    
    float x_gyro_in_new = (float)((int16_t)((buf[8] << 8) | buf[9]))*(0.0174532 / 16.4);
    float y_gyro_in_new = (float)((int16_t)((buf[10] << 8) | buf[11]))*(0.0174532 / 16.4);
    float z_gyro_in_new = (float)((int16_t)((buf[12] << 8) | buf[13]))*(0.0174532 / 16.4);  

    rotate_3f(_rotation, x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);

    _gyro_raw[0] = x_gyro_in_new;
    _gyro_raw[1] = y_gyro_in_new;
    _gyro_raw[2] = z_gyro_in_new;
    
    x_gyro_in_new -= _gyro_offset[0];
    y_gyro_in_new -= _gyro_offset[1];
    z_gyro_in_new -= _gyro_offset[2];

	_gyro[0] = _gyro_filter_x.apply(x_gyro_in_new);
	_gyro[1] = _gyro_filter_y.apply(y_gyro_in_new);
	_gyro[2] = _gyro_filter_z.apply(z_gyro_in_new);    

	_update = true;
}
