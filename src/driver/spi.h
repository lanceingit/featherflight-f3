#pragma once

class SPI
{
public:
    
    void init(void);
    bool transferByte(uint8_t* out, uint8_t in);
    bool transfer(uint8_t *out, const uint8_t *in, int len);
    void setDivisor(uint16_t divisor);
    bool readIdentification();
    


    typedef enum {
        SPI_CLOCK_INITIALIZATON = 256,
        SPI_CLOCK_SLOW          = 128, //00.56250 MHz
        SPI_CLOCK_STANDARD      = 4,   //09.00000 MHz
        SPI_CLOCK_FAST          = 2,   //18.00000 MHz
        SPI_CLOCK_ULTRAFAST     = 2,   //18.00000 MHz
    } SPIClockDivider_e;    

private:

    #define SPI_TIMEOUT    1000



};
