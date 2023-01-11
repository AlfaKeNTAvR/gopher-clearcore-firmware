#include <Arduino.h>
#include "ClearCore.h"

#ifndef MY_POT_H
#define MY_POT_H
class Potentiometer {
public:
	Potentiometer(ClearCorePins pin,float min, float max);
    float getAngle();
    int getAnalogValue();
    int min; 
    int max;
    const int num_samples = 20;
    const float max_angle = 90;


private:
    ClearCorePins pin;
    
};
#endif