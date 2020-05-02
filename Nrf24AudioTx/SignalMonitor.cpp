#include "SignalMonitor.h"


SignalMonitor::SignalMonitor(int _x_kn1)
{
    x_kn1 = _x_kn1;
}


/* returns 1 if signal increased */
boolean SignalMonitor::detectIncrease(int _x_k)
{
    boolean result = 0;
    if(_x_k > x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/* returns 1 if signal decreased */
boolean SignalMonitor::detectDecrease(int _x_k)
{
    boolean result = 0;
    if(_x_k < x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}

/* returns 1 if signal has changed */
boolean SignalMonitor::detectChange(int _x_k)
{
    boolean result = 0;
    if(_x_k != x_kn1) result = 1;

    x_kn1 = _x_k;
    return(result);
}








