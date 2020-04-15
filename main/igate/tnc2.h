/* ========================================
 *
 * Copyright © 2019, Dana H. Myers.
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ========================================
*/

#ifndef _TNC2_H_
#define _TNC2_H_

#include <stdint.h>
#include "kbuf.h"

/*
 *
 */
#define	MAX_TNC2_BUF_SIZE	2800
#define	MAX_TNC2_DIGIS		10		// allow some margin
#define	MAX_TNC2_ADDR_LEN	10		// XXNXXX-NN*n

#define	AX_ADDR_LEN			7


typedef struct {
	uint8_t *srcAddr;
	uint8_t *destAddr;
	uint8_t *digis[MAX_TNC2_DIGIS];
	uint8_t *digiPath;
	uint8_t	digiCount;
	uint8_t *msg;
	size_t	msgLen;
} tnc2buf_t;

/*
 * Free a TNC2-form frame buffer
 */
void freeTNC2Buf(tnc2buf_t *);

/*
 * Given an AX.25 frame in a kbuf_t, return a
 * TNC2-format buffer (with 8-bit chars) or NULL
 * if a conversion error.
 *
 * Caller must free TNC2-buffer
 */
tnc2buf_t *ax25ToTNC2(kbuf_t *);

/*
 * Parse a TNC2-format buffer (8-bit chars allowed) into
 * an AX.25 frame in a kbuf_t or NULL if parse fails
 *
 * Caller must free AX.25 frame-buffer
 */
kbuf_t *tnc2ToAX25(tnc2buf_t *);

/*
 * Parse a TNC2-format text-buffer (8-bit chars allowed) into
 * a TNC2-buffer, NULL if failure
 *
 * Caller must free TNC2-buffer
 */
tnc2buf_t *textToTNC2(uint8_t *, size_t);

/*
 * Return the TNC2-format text-buffer from the
 * TNC2-buffer, NULL if failure
 *
 * Caller must free() text-buffer
 */
uint8_t *tnc2ToText(tnc2buf_t *, size_t *);

/*
 * Process a valid TNC2-buf into an AX.25 frame
 * kbuf_t * if successful, NULL if failure
 */
kbuf_t *tnc2ToAx25(tnc2buf_t *);
#endif /* _TNC2_H_ */
