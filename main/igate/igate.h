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

#ifndef _IGATE_H_
#define _IGATE_H_

#include "sys/queue.h"
#include "igate/tnc2.h"
#include "kbuf.h"

#define	HEARD_CACHE_LIFETIME	((15 * 60000UL) / portTICK_PERIOD_MS)


typedef struct heard_cache_entry {
	TickType_t						timeStamp;
	uint8_t							addr[MAX_TNC2_ADDR_LEN];
	LIST_ENTRY(heard_cache_entry)	entries;
} heard_cache_t;

void iGateToIS(kbuf_t *);
void igateStart(void);

#endif /* _IGATE_H_ */
