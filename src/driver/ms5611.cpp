#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "timer.h"

#include "ms5611.h"



MS5611::MS5611()
{
    _i2c = new I2C();
}


bool MS5611::init(void)
{

    bool ack = false;
    uint8_t sig;

    _i2c->write(MS5611_ADDR, CMD_RESET, 1);
    Timer_delayUs(100*1000);

    
    ack = _i2c->read(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
    if (!ack)
        return false;

    if(read_prom() == false)
    {
        return false;
    }        
    
    _measure_phase = 0;
    _collect_phase = false;
    
    return true;
}


void MS5611::read()
{
    if (_collect_phase) 
    {
        uint32_t raw;
        raw = read_adc();

        if(_measure_phase == 0) 
        {
            int32_t dT = (int32_t)raw - ((int32_t)_prom_buf.s.c5_reference_temp << 8);
            _TEMP = 2000 + (int32_t)(((int64_t)dT * _prom_buf.s.c6_temp_coeff_temp) >> 23);
			_OFF  = ((int64_t)_prom_buf.s.c2_pressure_offset << 16) + (((int64_t)_prom_buf.s.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_prom_buf.s.c1_pressure_sens << 15) + (((int64_t)_prom_buf.s.c3_temp_coeff_pres_sens * dT) >> 8);
			if (_TEMP < 2000) 
            {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)_TEMP - 2000);
				int64_t OFF2 = 5 * f >> 1;
				int64_t SENS2 = 5 * f >> 2;

				if (_TEMP < -1500) {

					int64_t f2 = POW2(_TEMP + 1500);
					OFF2 += 7 * f2;
					SENS2 += 11 * f2 >> 1;
				}

				_TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}
        }
        else
        {
            int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;
//            _P = P * 0.01f;
//            _T = _TEMP * 0.01f;
        
            _temperature = _TEMP / 100.0f;
            _pressure = P / 100.0f;		/* convert to millibar */
            const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
            const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
            const double g  = 9.80665;	/* gravity constant in m/s/s */
            const double R  = 287.05;	/* ideal gas constant in J/kg/K */

            /* current pressure at MSL in kPa */
            double p1 = MSL_PRESSURE / 1000.0;

            /* measured pressure in kPa */
            double p = P / 1000.0;

            /*
             * Solve:
             *
             *     /        -(aR / g)     \
             *    | (p / p1)          . T1 | - T1
             *     \                      /
             * h = -------------------------------  + h1
             *                   a
             */
            _altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

        }
        
        _measure_phase++;
        if(_measure_phase >= MS5611_MEASUREMENT_RATIO+1)
        {
            _measure_phase = 0;
        }
        
        _collect_phase = false;
        return;
    }
    
    
    if(_measure_phase == 0)
    {
        _i2c->write(MS5611_ADDR, ADDR_CMD_CONVERT_D2_OSR1024, 1); 
    }
    else
    {
        _i2c->write(MS5611_ADDR, ADDR_CMD_CONVERT_D1_OSR1024, 1); 
    }
        
    _collect_phase = true;

}

uint32_t MS5611::read_adc(void)
{
    uint8_t rxbuf[3];
    _i2c->read(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

bool MS5611::read_prom(void)
{
    uint8_t rxbuf[2] = { 0, 0 };

    for (uint8_t i = 0; i < PROM_NB; i++)
    {
        _i2c->read(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
        _prom_buf.c[i] = rxbuf[0] << 8 | rxbuf[1];    
    
    }
    
    if (crc(_prom_buf.c) != 0)
        return false;    
    
    return true;
}

int8_t MS5611::crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}
