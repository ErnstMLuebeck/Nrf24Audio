

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter 
{
    public:
        LowPassFilter(float _Ts, float _Tc, float _y_n1);
        float calculate(float _x_0);
        void setTc(float _Tc);
        void setYn1(float _y_n1);

    private:
        float y_0, y_n1;  // y[0], y[-1]
        float Ts;   // sample time
        float Tc;   // time constant
        float alpha;  // filter constant
};

#endif


