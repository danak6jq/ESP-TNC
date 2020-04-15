/* ========================================
 *
 * Copyright © 2015,2019, Dana H. Myers.
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#include "kbuf.h"
#include "axfcs.h"

/*
 *
 */
kbuf_t *
kbuf_alloc(uint16_t len, uint8_t *data)
{
	kbuf_t *kb;

    /* ESP-IDF malloc() is thread-safe */
    kb = (kbuf_t *) malloc(sizeof (kbuf_t) + (len - 1) * sizeof (uint8_t));

    if (kb != NULL) {
    	kb->kb_o_ndx = 0;
        kb->kb_size = len;
        memcpy(kb->kb_data, data, len);
    }

    return (kb);
}

/*
 * ESP-IDF free() is thread-safe
 */
void
kbuf_free(kbuf_t *kb)
{
	free(kb);
}

int16_t
kbuf_get_8(kbuf_t *kb)
{
    int16_t ret_val = -1;

	if (kb->kb_o_ndx < kb->kb_size) {
		ret_val = kb->kb_data[kb->kb_o_ndx++];
	}

    return (ret_val);
}

uint16_t
kbuf_fcs(kbuf_t *kb)
{
    return (fcs_value(kb->kb_data, kb->kb_size));
}

uint16_t
kbuf_len(kbuf_t *kb)
{
    return (kb->kb_size);
}

/* [] END OF FILE */
