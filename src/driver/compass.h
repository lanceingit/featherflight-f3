#pragma once

class Compass
{

public:
	Compass(){}
	~Compass(){}
	virtual bool init(void)=0;
	virtual void read(void)=0;

    float get_mag_x() {return _mag[0];}
    float get_mag_y() {return _mag[1];}
    float get_mag_z() {return _mag[2];}

    void set_mag(float *mag) { _mag[0] = mag[0]; _mag[1] = mag[1]; _mag[2] = mag[2];}

    void set_mag_x(float mag) {_mag[0] = mag;}
    void set_mag_y(float mag) {_mag[1] = mag;}
    void set_mag_z(float mag) {_mag[2] = mag;}


protected:
    float _mag[3];

private:

};
