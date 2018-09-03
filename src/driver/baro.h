#pragma once



class Baro
{

public:
	Baro() {}
	~Baro() {}

	virtual bool init(void)=0;
	virtual void read(void)=0;

    float get_press() {return _pressure;}
    float get_altitude() {return _altitude;}
    float get_temp() {return _temperature;}

protected:
    float _temperature;
    float _pressure;
    float _altitude;

private:

};
