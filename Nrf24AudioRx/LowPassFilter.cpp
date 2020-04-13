

#include "LowPassFilter.h"


// Class Constructor
LowPassFilter::LowPassFilter(float _Ts, float _Tc, float _y_n1) 
{
    Ts = _Ts;
    Tc = _Tc;
    alpha = 1-Ts/Tc;
    if(alpha > 1.0) alpha = 1.0;
    if(alpha < 0.0) alpha = 0.0;
    y_n1 = _y_n1;

}

float LowPassFilter::calculate(float _x_0) 
{
    y_0 = alpha * y_n1 + (1-alpha) * _x_0;
    y_n1 = y_0;
    return(y_n1);
}

void LowPassFilter::setTc(float _Tc)
{
    if(_Tc == 0.0) Tc = Ts;
    else Tc = _Tc;
    alpha = 1-Ts/Tc;
}

void LowPassFilter::setYn1(float _y_n1)
{
    y_n1 = _y_n1;
}



