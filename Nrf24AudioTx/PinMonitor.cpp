

#include "PinMonitor.h"


PinMonitor::PinMonitor(int _numPin, uint16_t _numDeBnceCycles, boolean _stActv, boolean _pullUpResistor)
{
    numPin = _numPin;
    numDeBnceCycles = _numDeBnceCycles;
    stActv = _stActv;

    cntrDeBnce = 0;
    y_0 = 0;
    y_n1 = 0;

    flgRisngEdgePndng = 0;
    flgFallngEdgePndng = 0;
    flgEdgePndng = 0;

    TiRisngEdge = 0;
    TiFallngEdge = 0;

    TiHighLim = 0;
    TiLowLim = 0;

    flgIgnoreNxtRisngEdge = 0;
    flgIgnoreNxtFallngEdge = 0;

    pinMode(numPin, INPUT);
    digitalWrite(numPin, _pullUpResistor);

}

// sets the time limit to detect "long" states [ms]
void PinMonitor::setTiHighLim(unsigned long _TiHighLim)
{
    TiHighLim = _TiHighLim;
}

// sets the time limit to detect "long" states [ms]
void PinMonitor::setTiLowLim(unsigned long _TiLowLim)
{
    TiLowLim = _TiLowLim;
}

/* This function needs to be called regularly (e.g. in an interrupt routine)
 * The pin is sampled and a debounce counter is incremented/decremented according
 * to the pin state. If the debounce threshold is reached, the state is set.
 * All kind of edges are detected. There is an inhibit flag to skip the next edge.
 * This is used after a long press, so the button release does not trigger an
 * additional action. 
 */
void PinMonitor::update()
{
    // update counter
    if(digitalRead(numPin) == stActv)
    {   if(cntrDeBnce < numDeBnceCycles) cntrDeBnce++;
    }
    else
    {   if(cntrDeBnce > 0) cntrDeBnce--;
    }

    // detect rising edge
    if((cntrDeBnce >= numDeBnceCycles) && (y_n1 == 0))
    {   
        if(!flgIgnoreNxtRisngEdge) flgRisngEdgePndng = 1;
        else flgIgnoreNxtRisngEdge = 0;
        
        flgEdgePndng = 1;
        y_0 = 1;

        // save time of rising edge
        TiRisngEdge = millis();
        TiFallngEdge = 0;
    }

    // detect falling edge
    if((cntrDeBnce <= 0) && (y_n1 == 1))
    {   
        if(!flgIgnoreNxtFallngEdge) flgFallngEdgePndng = 1;
        else flgIgnoreNxtFallngEdge = 0;
        
        flgEdgePndng = 1;
        y_0 = 0;
     
        // save time of rising edge
        TiFallngEdge = millis();
        TiRisngEdge = 0;
    }
    
    y_n1 = y_0;

}

// returns high if a rising edge occured
boolean PinMonitor::risingEdge()
{
    if(flgRisngEdgePndng)
    {   flgRisngEdgePndng = 0;
        
        if(!flgIgnoreNxtRisngEdge)
        {   return(1);
        }
        else
        {   return(0);
        }
    }
    else return(0);
}

// returns high if a falling edge occured
boolean PinMonitor::fallingEdge()
{
    if(flgFallngEdgePndng)
    {   flgFallngEdgePndng = 0;
        
        if(!flgIgnoreNxtFallngEdge)
        {   return(1);
        }
        else
        {   return(0);
        }
    }
    else return(0);
}

// returns high if any edge occured
boolean PinMonitor::anyEdge()
{
    if(flgEdgePndng)
    {   flgEdgePndng = 0;
        return(1);
    }
    else return(0);
}

// returns the current, debounced pin state
boolean PinMonitor::getState()
{
    return(y_0);
}

// returns the time the pin spent being high [ms]
unsigned long PinMonitor::getHighTime()
{
    if(y_0 == 1) return(millis()-TiRisngEdge);
    else return(0);
}

// returns the time the pin spent being low [ms]
unsigned long PinMonitor::getLowTime()
{
    if(y_0 == 0) return(millis()-TiFallngEdge);
    else return(0);
}

// returns high if the high time is longer than the high threshold
boolean PinMonitor::highLong()
{
    if((TiRisngEdge != 0) && (millis()-TiRisngEdge) >= TiHighLim) 
    {   TiRisngEdge = 0; 
        return(1);
    }
    else return(0);
}

// returns high if the low time is longer than the high threshold
boolean PinMonitor::lowLong()
{
    if((TiFallngEdge != 0) && (millis()-TiFallngEdge) >= TiLowLim) 
    {   TiFallngEdge = 0; 
        return(1);
    }
    else return(0);
}

// sets the inhibit flag to ignore the next rising edge
void PinMonitor::ignoreNxtRisngEdge()
{
    flgIgnoreNxtRisngEdge = 1;
}

// sets the inhibit flag to ignore the next falling edge
void PinMonitor::ignoreNxtFallngEdge()
{
    flgIgnoreNxtFallngEdge = 1;
}
















