
#ifndef AUDIONRF24TX_H
#define AUDIONRF24TX_H

#include "Arduino.h"
#include "AudioStream.h"

class AudioNrf24Tx : public AudioStream
{
public:
	AudioNrf24Tx(void) : AudioStream(1, inputQueueArray) 
    {
	}
	bool available(void) 
    {
		__disable_irq();
		bool flag = new_output;
		if (flag) new_output = false;
		__enable_irq();
		return flag;
	}
    bool isClipping(void) 
    {
		__disable_irq();
		bool flag = FlgClipping;
		if (flag) FlgClipping = false;
		__enable_irq();
		return flag;
	}

	virtual void update(void);
private:
	audio_block_t *inputQueueArray[1];
    volatile bool new_output = false; 
    volatile bool FlgClipping = false;
};

#endif