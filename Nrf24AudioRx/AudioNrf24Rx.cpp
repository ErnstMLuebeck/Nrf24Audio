#include <Arduino.h>
#include "AudioNrf24Rx.h"
#include "RF24.h"

void AudioNrf24Rx::update(void)
{
    //digitalWrite(DEBUG2_PIN, LOW);

    TiUpdateBuffer_kn1 = TiUpdateBuffer_k;
    TiUpdateBuffer_k = micros();
    TdUpdateBuffer = TiUpdateBuffer_k - TiUpdateBuffer_kn1;

    IdxPingPong_k = IdxPingPongRead;
    
    if(IdxPingPong_k == IdxPingPong_kn1)
    {
        FlgPingPongViolation = 1;
        //digitalWrite(DEBUG_PIN, HIGH);

    }
    else
    {   FlgPingPongViolation = 0;
        //digitalWrite(DEBUG_PIN, LOW);
    }
    
    IdxPingPong_kn1 = IdxPingPongRead;

    TiUpdate = micros();

	audio_block_t *block;

	block = allocate();
	if (block == NULL) return;

	for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {      
        block->data[i] = OutputBuffer[IdxPingPong_k][i];      
        
        /* Clear buffer when played to see if it is empty */
        OutputBuffer[IdxPingPong_k][i] = 0.0f;
    }

	transmit(block, 0);
    release(block);
}
