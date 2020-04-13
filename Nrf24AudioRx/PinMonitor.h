

#ifndef PINMONITOR_H
#define PINMONITOR_H

#include <Arduino.h>

/* The PinMonitor is watching a physical pin and can return
 * a number of events: rising edge, falling edge, toggle. 
 * The pin is also debounced.
 */
 
class PinMonitor 
{
    public:
        PinMonitor(int _numPin, uint16_t _numDeBnceCycles, boolean _stActv, boolean _pullUpResistor);
        void update();
        boolean risingEdge();
        boolean fallingEdge();
        boolean anyEdge();
        boolean getState();
        unsigned long getHighTime();
        unsigned long getLowTime();
        void setTiHighLim(unsigned long _TiHighLim);
        void setTiLowLim(unsigned long _TiLowLim);
        boolean highLong();
        boolean lowLong();
        void ignoreNxtRisngEdge();
        void ignoreNxtFallngEdge();
        
    private:
        int numPin;
        boolean stActv;         // which state is considered HIGH, 1 or 0
        uint16_t numDeBnceCycles;
        uint16_t cntrDeBnce;    // debounce counter
        boolean y_0, y_n1;    // y[0], y[-1]
        boolean flgRisngEdgePndng;
        boolean flgFallngEdgePndng;
        boolean flgEdgePndng;
        unsigned long TiRisngEdge;      // absolute time of last rising edge
        unsigned long TiFallngEdge;     // absolute time of last falling edge
        unsigned long TiHighLim;   // limit to detect long high time
        unsigned long TiLowLim;  // limit to detect long low time
        boolean flgIgnoreNxtRisngEdge;
        boolean flgIgnoreNxtFallngEdge;
        
};

#endif


