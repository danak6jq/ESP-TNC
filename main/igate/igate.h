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
