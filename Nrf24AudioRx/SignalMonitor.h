

#ifndef SIGNALMONITOR_H
#define SIGNALMONITOR_H

#include <Arduino.h>

/* The SignalMonitor is watching a signal and detects changes */
 
class SignalMonitor 
{
    public:
        SignalMonitor(int _x_kn1);
        boolean detectIncrease(int _x_k);
        boolean detectDecrease(int _x_k);
        boolean detectChange(int _x_k);
        
    private:
        int x_kn1;
};

#endif


