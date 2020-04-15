/* ========================================
 *
 * Copyright © 2017,2019, Dana H. Myers.
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Dana H. Myers.
 *
 * ========================================
*/

#ifndef AXMODEM_H_
#define AXMODEM_H_

#include "kbuf.h"

#define SAMPLE_RATE         (12000)
#define BAUD_RATE           (1200)
#define	SAMPLES_BIT         (SAMPLE_RATE / BAUD_RATE)

/*
 * XXX: the modem is not decimating, the AC101 is configured for
 * 12000 samples/sec but the I2S stream is 4x the same samples at
 * 48000 samples/sec, so this is used to size the DMA buffer
 * Really wish I could get the AC101 to just send 1x samples at
 * 12000s/s but can't seem to.
 */
#define DECIMATION_FACTOR   (4)


/*
 * Size the DSP block for 4 bits/block, this is actually
 * limited by the DMA block len in the ESP32
 */
#define DSP_BLOCK_LEN	(SAMPLES_BIT * 4)
#define DMA_BLOCK_LEN	(DSP_BLOCK_LEN * DECIMATION_FACTOR)

/*
 * modem processor
 */
void modemProcessInput(float *);
void modemInitialize(void);

/*
 *
 */
void transmitFrame(kbuf_t *);
#endif /* AXMODEM_H_ */
