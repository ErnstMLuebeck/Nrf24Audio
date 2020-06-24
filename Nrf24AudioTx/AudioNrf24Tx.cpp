#include <Arduino.h>
#include "AudioNrf24Tx.h"
#include "RF24.h"

#define NUM_SAMPLES_PER_BLOCK 64

#define DEBUG_PIN 17

extern int16_t InputBuffer[NUM_SAMPLES_PER_BLOCK];

void AudioNrf24Tx::update(void)
{
    //digitalWrite(DEBUG_PIN, HIGH);

    audio_block_t *block;
    bool FlgClippingTemp = 0;
    bool FlgHotTemp = 0;

    block = receiveReadOnly();
	if (block == NULL) return;

	for (int i = 0; i < NUM_SAMPLES_PER_BLOCK; i++)
    {
        /* Open: Limiter/Compressor */

        InputBuffer[i] = block->data[i];

        if(InputBuffer[i] >= 32000) /* int16_t maximum = 32767*/
        {   FlgHotTemp = 0;
            FlgClippingTemp = 1;
        }
    }

	new_output = true;

    if(FlgClippingTemp) FlgClipping = 1;

	release(block);

    //digitalWrite(DEBUG_PIN, LOW);
}