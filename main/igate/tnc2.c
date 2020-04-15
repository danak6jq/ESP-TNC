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

#include "esp_system.h"
#include <stdio.h>
// #include <stdint.h>
#include <string.h>

// #include "igate/igate.h"
#include "kbuf.h"
#include "igate/tnc2.h"

/*
 *
 */
typedef enum {
	AxAddrMore,
	AxAddrLast,
	AxAddrError
} AxAddrVal;

/*
 *
 */
void freeTNC2Buf(tnc2buf_t *tb)
{

	if (!tb) {
		return;
	}

	if (tb->destAddr) {
		free(tb->destAddr);
	}

	if (tb->srcAddr) {
		free(tb->srcAddr);
	}

	for (int i = 0; i < MAX_TNC2_DIGIS; i++) {
		if (tb->digis[i])
			free(tb->digis[i]);
	}

	if (tb->digiPath) {
		free(tb->digiPath);
	}

	if (tb->msg) {
		free(tb->msg);
	}

	free(tb);
}

/*
 * Parse an AX.25 address into TNC2-form
 * Caller must ensure 7 bytes are available
 * malloc()s buffer that caller must free
 * Returns an AxAddrVal
 */
static AxAddrVal
axAddrToTNC2(uint8_t **axp, size_t *axlen, uint8_t **tnc2Buf, bool hBit)
{
	uint8_t addrBuf[MAX_TNC2_ADDR_LEN];	// XXX: needs a define ~11 for 'XXNXXX*-NN'
	int p, ssid;

	if (*axlen < AX_ADDR_LEN) {
		puts("axAddr: frame too short");
		return (AxAddrError);
	}

	p = 0;
	for (int i = 0; i < AX_ADDR_LEN - 1; i++) {
		char c;

		if ((*axp)[i] & 1) {
			// XXX: invalid address termination bit
			puts("axAddr: invalid address termination bit");
		}

		c = (*axp)[i] >> 1;
		if (c == ' ') {
			break;
		}

		/*
		 * If addr has a '*', terminate it there
		 */
		if (c == '*') {
			// XXX: log this as a warning
			break;
		}

		addrBuf[p++]= c;
	}

	ssid = ((*axp)[6] >> 1) & 0xf;
	if (ssid > 0) {
		addrBuf[p++] = '-';
		p += snprintf((char *)addrBuf + p, 3, "%d", ssid);
	} else {
		addrBuf[p] = '\0';
	}

	/* append '*' for heard-bit */
	if (hBit && ((*axp)[6] & 0x80)) {
		addrBuf[p++] = '*';
		addrBuf[p] = '\0';
	}

	*tnc2Buf = malloc(strlen((char *)addrBuf) + 1);
	if (*tnc2Buf == NULL) {
		return (AxAddrError);
	}

	*axp += AX_ADDR_LEN;
	*axlen -= AX_ADDR_LEN;
	strcpy((char *)*tnc2Buf, (char *)addrBuf);
	return ((*axp)[6 - AX_ADDR_LEN] & 0x01 ? AxAddrLast : AxAddrMore);
}

/*
 * Parse digipeater path, saving digis + path
 * XXX: add switch to configure all-space handling
 */
static AxAddrVal
ax25ToTNC2Path(uint8_t **axp, size_t *axlen, tnc2buf_t *tb)
{
	AxAddrVal av;
	int	tnc2Digis = 0;
	size_t pathLen = 0;

	do {
		av = axAddrToTNC2(axp, axlen, tb->digis + tnc2Digis, true);
		if (av != AxAddrError) {
			pathLen += strlen((char *)tb->digis[tnc2Digis]) + 1;
			if (++tnc2Digis >= MAX_TNC2_DIGIS) {
				av = AxAddrError;
			}
		}
	} while (av == AxAddrMore);

	if (av == AxAddrError) {
		tb->digiPath = NULL;
		return (av);
	}

	tb->digiPath = calloc(1, pathLen);
	if (tb->digiPath == NULL) {
		return (AxAddrError);
	}

	for (int i = 0; i < tnc2Digis; i++) {
		strcat((char *) tb->digiPath, (char *) tb->digis[i]);
		if (i < tnc2Digis - 1) {
#if 1
			/* check for a zero-length addr next */
			if (strlen((char *)tb->digis[i + 1]) == 0) {
				break;
			}
#endif
			strcat((char *) tb->digiPath, ",");
		}
	}

	tb->digiCount = tnc2Digis;
	return (av);
}

/*
 * Convert a valid AX.25 frame to an APRS frame in a TNC2 buffer
 */
tnc2buf_t *
ax25ToTNC2(kbuf_t *fp)
{
	tnc2buf_t *tb;
	uint8_t *axp, *term;
	size_t	axLen;
	AxAddrVal av;

	/* allocate TNC2 frame buffer */
	if ( (tb = calloc(1, sizeof (tnc2buf_t))) == NULL) {
		return (tb);
	}

	axLen = fp->kb_size;
	axp = fp->kb_data;

	/* destination address */
	av = axAddrToTNC2(&axp, &axLen, &tb->destAddr, false);
	if (av != AxAddrMore) {
		/* invalid address field */
		goto errorReturn;
	}

	/* source address */
	av = axAddrToTNC2(&axp, &axLen, &tb->srcAddr, false);
	if (av == AxAddrError) {
		/* invalid address field */
		goto errorReturn;
	}

	if (av == AxAddrMore) {
		/* parse the digi path */
		av = ax25ToTNC2Path(&axp, &axLen, tb);
		if (av == AxAddrError) {
			goto errorReturn;
		}
	}

	/* Verify this is an APRS UI + PID 0xf0 frame */
	if (axLen < 2 || axp[0] != 0x03 || axp[1] != 0xf0) {
		// XXX: packet dump to error log
		puts("XXX: not an APRS frame");
		goto errorReturn;
	}

	/* skip control + PID bytes */
	axp += 2;
	axLen -= 2;

	/*
	 * Terminate payload at the first \r\n or \n
	 */
	if ((term = memmem(axp, axLen, "\r\n", 2)) == NULL) {
		term = memchr(axp, '\n', axLen);
	}

	tb->msgLen = term ? term - axp : axLen;
	tb->msg = malloc(tb->msgLen);
	if (tb->msg == NULL) {
		goto errorReturn;
	}

	memcpy(tb->msg, axp, tb->msgLen);

	// XXX: log a warning when payload truncated before end
	return (tb);

errorReturn:
	freeTNC2Buf(tb);
	return (NULL);
}

/*
 * Copy address from/update supplied text pointer into allocated buffer
 */
static int
textAddrToTNC2(uint8_t **textp, uint8_t **addrBuf, const char *delim)
{
	uint8_t *tp0;
	int addrLen;

	tp0 = (uint8_t *) strpbrk((char *) *textp, delim);

	if (((addrLen = tp0 ? tp0 - *textp : -1) < 0) ||
	  (addrLen > 20) ||
	  ((*addrBuf = malloc(addrLen + 1)) == NULL)) {
		return (AxAddrError);
	}

	strlcpy((char *) *addrBuf, (char *) *textp, addrLen + 1);
	*textp += addrLen;

	return (AxAddrMore);
}

/*
 * Parse digipeater path, saving digis + path
 * XXX: add switch to configure all-space handling
 */
static AxAddrVal
textToTNC2Path(uint8_t **textp, tnc2buf_t *tb)
{
	AxAddrVal av;
	int	tnc2Digis = 0;
	size_t pathLen = 0;

	do {
		*textp += 1;
		av = textAddrToTNC2(textp, tb->digis + tnc2Digis, ",:");
		if (av != AxAddrError) {
			pathLen += strlen((char *)tb->digis[tnc2Digis]) + 1;
			if (++tnc2Digis >= MAX_TNC2_DIGIS) {
				av = AxAddrError;
			}
		}
	} while (av != AxAddrError && **textp == ',');

	if (av == AxAddrError) {
		tb->digiPath = NULL;
		return (av);
	}

	tb->digiPath = calloc(1, pathLen);
	if (tb->digiPath == NULL) {
		return (AxAddrError);
	}

	for (int i = 0; i < tnc2Digis; i++) {
		strcat((char *) tb->digiPath, (char *) tb->digis[i]);
		if (i < tnc2Digis - 1) {
#if 1
			/* check for a zero-length addr next */
			if (strlen((char *)tb->digis[i + 1]) == 0) {
				break;
			}
#endif
			strcat((char *) tb->digiPath, ",");
		}
	}

	tb->digiCount = tnc2Digis;
	return (av);
}


/*
 * Convert a valid TNC2 text line to a TNC2 buffer
 */
tnc2buf_t *
textToTNC2(uint8_t *textp, size_t len)
{
	tnc2buf_t *tb;
	uint8_t *tp0 = textp;


	/* allocate TNC2 frame buffer */
	if ((tb = calloc(1, sizeof (tnc2buf_t))) == NULL) {
		return (tb);
	}

	/* src addr first */
	if (textAddrToTNC2(&textp, &tb->srcAddr, ">") == AxAddrError) {
		/* invalid frame */
		goto errorReturn;
	}

	/* skip terminating '>' */
	textp += 1;

	/* dest addr next */
	if (textAddrToTNC2(&textp, &tb->destAddr, ":,") == AxAddrError) {
		/* invalid frame */
		goto errorReturn;
	}

	if (*textp == ',') {
		/* leave the terminating ',' intact */
		if (textToTNC2Path(&textp, tb) == AxAddrError) {
			goto errorReturn;
		}
	}

	/* skip terminating ':' */
	textp += 1;

	/* copy body */
	tb->msgLen = len - (textp - tp0);
	tb->msg = malloc(tb->msgLen);
	if (tb->msg == NULL) {
		goto errorReturn;
	}

	memcpy(tb->msg, textp, tb->msgLen);
	return (tb);

errorReturn:
	freeTNC2Buf(tb);
	return (NULL);
}

/*
 *
 */
uint8_t *
tnc2ToText(tnc2buf_t *tb, size_t *len)
{
	uint8_t *textbuf;
	size_t textlen;
	size_t tlen;

	textlen = strlen((char *)tb->srcAddr) + 1;	// alloc '>'
	textlen += strlen((char *)tb->destAddr);
	for (int i = 0; i < tb->digiCount; i++) {
		textlen += strlen((char *)tb->digis[i]) + 1; // ',' for each digi
	}
	textlen += tb->msgLen + 1;		// allow ':'

	textbuf = malloc(textlen);
	if (textbuf == NULL) {
		*len = 0;
		return (textbuf);
	}

	/* fill the output buffer */
	tlen = snprintf((char *)textbuf, textlen, "%s>%s", tb->srcAddr, tb->destAddr);
	if (tb->digiPath) {
		tlen += snprintf((char *)textbuf + tlen, textlen - tlen, ",%s", tb->digiPath);
	}
	textbuf[tlen++] = ':';
	/* use memcpy() instead of strcpy because there may be \0 bytes in msg */
	memcpy(textbuf + tlen, tb->msg, tb->msgLen);
	tlen += tb->msgLen;
	*len = tlen;

#if 1
	// XXX: test code
	if ((textlen - tlen) != 0) {
		printf("tnc2ToText: %d\n", textlen - tlen);
	}
#endif
	return (textbuf);
}

/*
 *
 */
static void
tnc2AddrToAx25(uint8_t *tnc2Addr, uint8_t *ax25Addr)
{
	int i;

	memset(ax25Addr, ' ' << 1, AX_ADDR_LEN - 1);
	ax25Addr[AX_ADDR_LEN - 1] = 0x60;

	for (i = 0; i < AX_ADDR_LEN - 1; i++) {
		uint8_t b = tnc2Addr[i];

		if (b == '\0' || b == '-' || b == '*') {
			break;
		}
		ax25Addr[i] = b << 1;
	}

	if (tnc2Addr[i] == '-') {
		unsigned int ssid;

		i += 1;
		sscanf((char *) tnc2Addr + i, "%u", &ssid);
		if (ssid > 15) {
			// XXX: error
			ssid = 15;
		}
		ax25Addr[AX_ADDR_LEN - 1] |= ssid << 1;
	}

	if (tnc2Addr[i] == '*' ) {
		/* set the H bit */
		ax25Addr[AX_ADDR_LEN - 1] |= 0x80;
	}
}

/*
 *
 */
kbuf_t *
tnc2ToAx25(tnc2buf_t *tp)
{
	uint8_t *fp;
	kbuf_t *kp;
	size_t axLen, axOffset;

	/* calculate ax.25 frame size */
	axLen = (2 + tp->digiCount) * AX_ADDR_LEN + 2 + tp->msgLen;
	fp = malloc(axLen);
	if (fp == NULL) {
		// XXX: metrics
		return (NULL);
	}

	tnc2AddrToAx25(tp->destAddr, fp + 0);
	/* set destAddr C-bit */
	fp[AX_ADDR_LEN - 1] |= 0x80;
	tnc2AddrToAx25(tp->srcAddr, fp + AX_ADDR_LEN);
	axOffset = 2 * AX_ADDR_LEN;
	for (int i = 0; i < tp->digiCount; i++) {
		tnc2AddrToAx25(tp->digis[i], fp + axOffset);
		axOffset += AX_ADDR_LEN;
	}
	/* terminate addresses */
	fp[axOffset - 1] |= 0x01;
	/* control + PID */
	fp[axOffset++] = 0x03;
	fp[axOffset++] = 0xf0;
	/* copy the body */
	memcpy(fp + axOffset, tp->msg, tp->msgLen);
	kp = kbuf_alloc(axLen, fp);
	free(fp);
	return (kp);
}
