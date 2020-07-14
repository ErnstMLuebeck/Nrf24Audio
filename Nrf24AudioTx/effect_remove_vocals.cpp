/* "Remove Vocals" Effect for Teensy Audio Library
 * Copyright (c) 2016, Frank Bösing, f.boesing@gmx.de
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "effect_remove_vocals.h"

void AudioEffectRemoveVocals::update(void)
{

	audio_block_t *blocka, *blockb;

	blocka = receiveWritable(0);
	blockb = receiveReadOnly(1);
	if (!blocka) {
		if (blockb) release(blockb);
		return;
	}
	if (!blockb) {
		release(blocka);
		return;
	}

	uint16_t *pa, *pb, *end;
	int16_t l,r;
	pa = (uint16_t*)blocka->data;
	pb = (uint16_t*)blockb->data;
	end = pa + AUDIO_BLOCK_SAMPLES;

	if (!enabled) {
		while (pa < end) {
			l = *pa;
			r = *pb++;
			*pa++ = (l + r) / 2;//convert to mono
		}
	} else {
		while (pa < end) {
			l = *pa;
			r = *pb++;
			*pa++ = (l - r) / 2; //add inverted right to left channel
		}
	}

	transmit(blocka);
	release(blocka);
	release(blockb);
}

