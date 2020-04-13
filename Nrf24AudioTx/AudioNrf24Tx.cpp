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

    block = receiveReadOnly();
	if (block == NULL) return;

	for (int i = 0; i < NUM_SAMPLES_PER_BLOCK; i++)
    {
        /* Open: Limiter/Compressor */

        InputBuffer[i] = block->data[i];
    }

	new_output = true;

	release(block);

    //digitalWrite(DEBUG_PIN, LOW);
}