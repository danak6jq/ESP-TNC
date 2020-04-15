/* ========================================
 *
 * Copyright © 2019, Dana H. Myers.
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Dana H. Myers.
 *
 * ========================================
*/

#ifndef KBUF_H_
#define KBUF_H_

/*
 * KISS buffers
 *
 * Contain a raw AX.25 frame
 * kbuf_alloc() - allocate and fill a kbuf
 * kbuf_free() - free a kbuf
 * XXX: kbuf_put_fcs() - calculate the FCS and append it to the kbuf
 * kbuf_get_8() - get a byte from the head of the kbuf
 * kbuf_len() - return the length of data in the kbuf
 */

#include <stdint.h>

#define	KB_MIN_FRAME_SIZE		17	/* 15-byte U/S frame + 2 bytes FCS XXX: */
//#define	KB_MAX_FRAME_SIZE		332	/* largest AX.25 V2.0 frame */
#define	KB_MAX_FRAME_SIZE       1024    /* largest AX.25 V2.0 frame */

/*
 *
 */
typedef struct {
	uint16_t	kb_size;
	uint16_t	kb_o_ndx;
	uint8_t		kb_data[1];
} kbuf_t;

/*
 * Allocate and fill a kbuf; returns NULL on failure
 */
kbuf_t *kbuf_alloc(uint16_t len, uint8_t *data);

/*
 * Free a kbuf
 */
void kbuf_free(kbuf_t *);

/*
 * Get next byte from the frame; returns -1 if none left
 */
int16_t kbuf_get_8(kbuf_t *);

/*
 * Returns transmit FCS for frame
 */
uint16_t kbuf_fcs(kbuf_t *);

/*
 * Returns length of frame
 */
uint16_t kbuf_len(kbuf_t *);

#endif /* KBUF_H_ */
