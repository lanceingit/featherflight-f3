#pragma once



class I2C
{
public:
    
    void init(void);
    bool read(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
    bool write(uint8_t addr_, uint8_t reg, uint8_t data);

private:


    #define I2C_SHORT_TIMEOUT   ((uint32_t)0x1000)
    #define I2C_LONG_TIMEOUT    ((uint32_t)(10 * I2C_SHORT_TIMEOUT))

    uint32_t i2cTimeout;


};
