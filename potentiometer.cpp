#include "potentiometer.h"
Potentiometer::Potentiometer(ClearCorePins pin,float min, float max){
    this->pin = pin;
    this->min = min;
    this->max = max;
   

}
////////////////////////////////////////////////////////////////////////////
// Calculates the Angle based on analog value of potentiometer
float Potentiometer::getAngle(){
    int analog_value = 0;
    float percent = 0;
    float current_angle = 0;
    float angle_sum = 0;
    float final_angle = 0;
    float diff = (float)(max-min);
    float l_max = max;
    
    for(int i = 0; i < num_samples;i++){
        analog_value = analogRead(pin);
        if(analog_value > l_max){
            l_max = analog_value;
            diff = l_max - min;
        }
        percent =abs(1 - ((analog_value-min)/diff));
        current_angle = max_angle*percent;
        angle_sum += current_angle;
    }
    final_angle = angle_sum/(float)num_samples;
    if (final_angle > 90.5){
        final_angle = -1;
    }

    return final_angle;

}
////////////////////////////////////////////////////////////////////////////
// Returns Analog output of potentiomenter
int Potentiometer::getAnalogValue(){
    int analog_value = analogRead(pin);
   
    return analog_value;

}