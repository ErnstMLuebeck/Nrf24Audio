
#ifndef AUDIONRF24RX_H
#define AUDIONRF24RX_H

#include "Arduino.h"
#include "AudioStream.h"

#define NUM_SAMPLES_PER_BLOCK 64
#define NUM_SAMPLES_PER_PACKET 16

#define DEBUG_PIN 16
#define DEBUG2_PIN 17

extern int16_t OutputBuffer[][NUM_SAMPLES_PER_BLOCK];
extern int16_t FadeOut[NUM_SAMPLES_PER_BLOCK];

extern unsigned long TiUpdate;
extern int IdxPingPongRead;
extern bool FlgPingPongViolation;
extern unsigned long TiUpdateBuffer_k;
extern unsigned long TiUpdateBuffer_kn1;
extern unsigned long TdUpdateBuffer;

class AudioNrf24Rx : public AudioStream
{
public:
	AudioNrf24Rx() : AudioStream(0, NULL) {}

	bool available(void) 
    {
		__disable_irq();
		bool flag = new_output;
		if (flag) new_output = false;
		__enable_irq();
		return flag;
	}

	virtual void update(void);
private:
	//audio_block_t *inputQueueArray[1];
    volatile bool new_output = false; 
    bool IdxPingPong_k = 0;
    bool IdxPingPong_kn1 = 0;
};

#endif