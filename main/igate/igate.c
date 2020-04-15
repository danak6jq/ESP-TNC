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
#include <stdint.h>
#include <string.h>

#include "sys/queue.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/dns.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#include "igate/igate.h"
#include "igate/tnc2.h"
#include "igate/digipeat.h"
#include "axmodem.h"

// void igate_from_aprsis(const char *ax25, int ax25len);

/*
 * iGate Configuration
 */

const uint8_t *myCallsign = (uint8_t *) "K6JQ";

/*
 * Filter prohibited path elements
 */
static char *prohibitedDigiToIS[] = {
		"TCPXX",
		"TCPIP",
		"NOGATE",
		"RFONLY",
		"OPNTRK",
		"OPNTRC"
};

#define	NUM_PROHIBITED_DIGIS_TO_IS	\
			(sizeof (prohibitedDigiToIS) / sizeof (prohibitedDigiToIS[0]))

static char *prohibitedDigiToRF[] = {
		"TCPXX",
		"NOGATE",
		"OPNTRK",
		"OPNTRC"
};

#define	NUM_PROHIBITED_DIGIS_TO_RF	\
			(sizeof (prohibitedDigiToRF) / sizeof (prohibitedDigiToRF[0]))


/*
 *
 */
LIST_HEAD(heardCacheHead, heard_cache_entry) heardCache;
struct heardCacheHead *heardCachep;

static bool
iGateToISPathFilter(tnc2buf_t *tb)
{

	for (int i = 0; i < tb->digiCount; i++) {
		for (int j = 0; j < NUM_PROHIBITED_DIGIS_TO_IS; j++) {
			if (strcmp((char *)tb->digis[i], prohibitedDigiToIS[j]) == 0) {
				return (true);
			}
		}
	}

	return (false);
}

static bool
iGateToRFPathFilter(tnc2buf_t *tb)
{

	for (int i = 0; i < tb->digiCount; i++) {
		for (int j = 0; j < NUM_PROHIBITED_DIGIS_TO_RF; j++) {
			if (strcmp((char *)tb->digis[i], prohibitedDigiToRF[j]) == 0) {
				return (true);
			}
		}
	}

	return (false);
}

/*
 *
 */
static bool
isWide(uint8_t *addr, size_t len)
{

	/* check first for any form of WIDE */
	if (strncmp((char *) addr, "WIDE", 4) != 0) {
		/* not wide */
		return (false);
	}

	/* return true if exactly WIDE */
	if (len == 4) {
		return (true);
	}

	/* return true if WIDEn or WIDEn-N */
	if (len >= 5 && addr[4] >= '0' && addr[4] <= '9') {
		if ((len == 5) || (len > 5 && addr[5] == '-')) {
			return (true);
		}
	}

	printf("allow WIDE: %s %u\n", addr, len);
	/* not a dis-allowed form of WIDE */
	return (false);
}

/*
 * Update heard list from received packet header
 *
 * Passes entire frame so the cache can do what it wants
 * with the entire path, though the cache right now
 * only remembers the directly-heard stations
 *
 * Does not modify the parameter
 *
 */
static void
iGateHeardRf(tnc2buf_t *tb)
{
	heard_cache_t *hp;
	bool found;
	TickType_t timeStamp;
	int heardDigi;
	size_t addrLen;
	uint8_t *addrPtr;

	/*
	 * Get timestamp first up
	 */
	timeStamp = xTaskGetTickCount();

	/*
	 * Determine if a digi was heard
	 */
	heardDigi = -1;
	for (int i = 0; i < tb->digiCount; i++) {
		if (strchr((char *)tb->digis[i], '*') != NULL) {
			heardDigi = i;
		} else {
			break;
		}
	}

	if (heardDigi >= 0) {
		/* use the digi, strip off the trailing '*' */
		addrPtr = tb->digis[heardDigi];
		addrLen = strlen((char *) addrPtr) - 1;
	} else {
		addrPtr = tb->srcAddr;
		addrLen = strlen((char *) addrPtr);
	}

	assert(addrLen <= MAX_TNC2_ADDR_LEN);

	// printf("RF heard: %u %.*s\n", timeStamp, addrLen, addrPtr);

	/* filter out aliases etc. */
	/* RELAY, ECHO, WIDE, WIDEn-N, TRACE, GATE */
	if ((strncmp((char *) addrPtr, "RELAY", addrLen) == 0) ||
	  (strncmp((char *) addrPtr, "ECHO", addrLen) == 0) ||
	  (strncmp((char *) addrPtr, "TRACE", addrLen) == 0) ||
	  (strncmp((char *) addrPtr, "GATE", addrLen) == 0) ||
	  isWide(addrPtr, addrLen) ) {
		// printf("Not HEARD: %s\n", addrPtr);
		return;
		/* don't record */
	}

	/*
	 * Scan the heardCache, timing-out entries also
	 */
	found = false;
	for (hp = heardCache.lh_first; hp != NULL; hp = hp->entries.le_next) {
		/* if entry found, update timeStamp */
		if (strncmp((char *) hp->addr, (char *) addrPtr, addrLen) == 0) {
			hp->timeStamp = timeStamp;
			printf("cache: found %s\n", hp->addr);
			found = true;
			continue;
		}

		/* do expiration processing */
		if (timeStamp - hp->timeStamp > HEARD_CACHE_LIFETIME) {
			printf("cache: removed %s\n", hp->addr);
			LIST_REMOVE(hp, entries);
			free(hp);
		}
	}

	/*
	 * If entry not in the list, add it
	 */
	if (!found) {
		hp = malloc(sizeof (heard_cache_t));
		if (hp == NULL) {
			puts("ERROR: failed to allocate heard_cache_t");
			return;
		}
		hp->timeStamp = timeStamp;
		strncpy((char *) hp->addr, (char *) addrPtr, addrLen);
		hp->addr[addrLen] = '\0';
		printf("cache: added %s\n", hp->addr);
		LIST_INSERT_HEAD(&heardCache, hp, entries);
	}
}

/*
 * Process an APRS 'Message', possible RF-gating it
 */
static void
iGateProcessMessage(tnc2buf_t *tb)
{
	uint8_t *toCall, *tp0;
	int		callLen;

#if 1
	printf("iGateToRF: %s>%s", tb->srcAddr, tb->destAddr);
	if (tb->digiPath) {
		printf(",%s", tb->digiPath);
	}
	printf(":%.*s\n", tb->msgLen, tb->msg);
#endif

	/* validate Message packet */
	if (tb->msgLen < 11 || tb->msg[10] != ':') {
		puts("invalid message");
		/* invalid Message header */
		return;
	}

	/* find destination call */
	tp0 = (uint8_t *) strpbrk((char *) tb->msg + 1, " :");
	if (((callLen = tp0 ? tp0 - tb->msg - 1 : -1) < 0) ||
	  (callLen > 9) || ((toCall = malloc(callLen + 1)) == NULL)) {
		/* XXX: improved logging */
		return;
	}
	strlcpy((char *) toCall, (char *) tb->msg + 1, callLen + 1);

	printf("fromCall: %s toCall: %s\n", tb->srcAddr, toCall);

	/*
	 * Simple Tx IGate
	 * Transmit Messages to BLN*, NWS-*, containing telemetry params
	 * Transmit Messages to toCall heard recently
	 */

	/* XXX: test for parameter messages, bulletins, NWS */
	//strncmp((char *) tb->msg[11], ""
}

/*
 * iGateToRF
 */
static void
iGateToRF(uint8_t *text, size_t textlen)
{
	tnc2buf_t *tb;

	if (textlen == 0) {
		// zero-length line, ignore
		return;
	}

	if (text[0] == '#') {
		// XXX: comment, heartbeat, some other IS thing
		// XXX: improve processing here
		printf("IS: %.*s\n", textlen, text);
		return;
	}

	/*
	 * Parse text into tnc2buf_t
	 */
	if ((tb = textToTNC2(text, textlen)) == NULL) {
		/* invalid text string */
		// XXX: improve logging
		return;
	}

	/*
	 * basic sanity test of frame
	 */
	if (strlen((char *)tb->srcAddr) == 0 ||
	  strlen((char *)tb->destAddr) == 0 ||
	  strlen((char *)tb->digiPath) == 0 ||
	  tb->msgLen == 0) {
		freeTNC2Buf(tb);
		return;
	}

	/*
	 * XXX: filter out frames that we won't transmit here
	 */

	/*
	 * Filter out 'general' queries
	 * XXX: answer them?
	 */
	if ((strcmp((char *)tb->msg, "?ARPS?") == 0) ||
	  (strcmp((char *)tb->msg, "?IGATE") == 0) ||
	  (strcmp((char *)tb->msg, "?WX?") == 0)) {
		printf("iGateToIS: general query: %.*s\n", tb->msgLen, tb->msg);
		freeTNC2Buf(tb);
		return;
	}

	/*
	 * Skip all non-message packets
	 */
	if (tb->msg[0] != ':') {
		freeTNC2Buf(tb);
		return;
	}

#if 0
	/*
	 * Extract 'to' callsign from 'Message' message
	 */
#if 1
	// XXX: replace all of this with iGateProcessMessage() later
	// XXX:
	if (1 || tb->msg[0] == ':') {
		uint8_t *newText;
		size_t newLen;

		newText = tnc2ToText(tb, &newLen);
		if (newText == NULL) {
			puts("!!!!!!!!!!!!!!!!!!!!! tnc2ToText failed!");
		} else {
			// printf("YY: %.*s\n", len, text);
			if (memcmp(text, newText, newLen) != 0) {
				puts("@@@@@@@@@@@@@@@@@@@@@@@ re-write of frame failed");
			}
			// XXX: send re-written frame
			free(newText);
		}
	}
	freeTNC2Buf(tb);
	return;
#endif
#endif

	/*
	 * test for prohibited path elements
	 */
	if (iGateToRFPathFilter(tb)) {
		puts("iGateToRF: prohibited digi");
		freeTNC2Buf(tb);
		return;
	}

	/*
	 * OK to iGate now
	 */
	iGateProcessMessage(tb);
	freeTNC2Buf(tb);
}

/*
 * iGate tasks
 *
 * Down-stream iGate task maintains connection to iGate, the associated socket
 * and collects in-bound messages.
 *
 * Up-stream iGate task sends out-bound messages when the iGate socket is valid,
 * and quietly discards them otherwise
 *
 */

static TaskHandle_t iGateInTaskHandle;
static TaskHandle_t iGateOutTaskHandle;
static QueueHandle_t isOutQueue;

/*
 * iGate host socket
 */
static int iGateSocket = -1;
static SemaphoreHandle_t iGateSocketMutex;
static ip_addr_t iGateAddr;
static const char *iGateName = "noam.aprs2.net";
static enum { DNS_WAIT, DNS_FOUND, DNS_NOTFOUND } dnsStatus;
static bool iGateOK;

/*
 * get/set iGate Host Socket under a mutex
 */
static int
getiGateSocket()
{
	int socket;

	xSemaphoreTake(iGateSocketMutex, portMAX_DELAY);
	socket = iGateSocket;
	xSemaphoreGive(iGateSocketMutex);
	return (socket);
}

static void
setiGateSocket(int socket)
{

	xSemaphoreTake(iGateSocketMutex, portMAX_DELAY);
	iGateSocket = socket;
	xSemaphoreGive(iGateSocketMutex);
}

static void
dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
	// XXX: do I need to copy this?
	if (ipaddr != NULL) {
		iGateAddr = *ipaddr;
		dnsStatus = DNS_FOUND;
	} else {
		dnsStatus = DNS_NOTFOUND;
	}
}

static int
igateReadLine(uint8_t *buf, size_t blen)
{
	uint8_t b;
	int c;

	c = 0;
	while (recv(getiGateSocket(), &b, 1, 0) > 0) {
		if (b == '\r') {
			// ignore
		} else if (b == '\n') {
			*buf++ = '\0';
			return (c);
		} else {
			*buf++ = b;
			c++;
		}
	}

	// socket error?
	return (-1);
}

/*
 * Maintain connection to iGate host, collect and process messages
 */
static void
iGateInTask(void *arg)
{
	err_t err;

	while (true) {
		/*
		 * Initiate DNS resolution of server hostname
		 */
	    // IP_ADDR4( &iGateAddr, 0,0,0,0 );
	    ip_addr_set_zero(&iGateAddr);
	    dnsStatus = DNS_WAIT;
	    err = dns_gethostbyname(iGateName, &iGateAddr, dns_found_cb, NULL );
	    if (err == ERR_VAL) {
	    	/* network isn't up yet; sleep a bit and re-try */
	    	vTaskDelay(100 / portTICK_PERIOD_MS);
	    	continue;
	    } else if (err == ERR_INPROGRESS) {
	    	/* lazy-spin waiting for the callback */
	    	/* XXX: this could timeout, right? */
	    	while (dnsStatus == DNS_WAIT) {
	    		vTaskDelay(20 / portTICK_PERIOD_MS);
	    	}
	    }

	    /*
	     * Validate DNS found the hostname
	     */
	    if (dnsStatus == DNS_NOTFOUND) {
	    	printf("DNS resolution for %s failed\n", iGateName);
	    	vTaskDelay(10000 / portTICK_PERIOD_MS);
	    	continue;
	    }

	    /*
	     * Connect to server
	     */
	    struct sockaddr_in serverAddr;
	    int serverSocket;
	    int cnt;
	    static uint8_t rBuff[520];


	    // XXX: there *must* be a better way to make this assignment
	    serverAddr.sin_addr.s_addr = ip_2_ip4(&iGateAddr)->addr;
	    serverAddr.sin_family = AF_INET;
	    serverAddr.sin_port = htons(14580);

	    serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	    if (serverSocket < 0) {
	    	vTaskDelay(1000 / portTICK_PERIOD_MS);
	    	continue;
	    }

	    err = connect(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
	    if (err < 0) {
	    	// XXX: error
	    	close(serverSocket);
	    	vTaskDelay(1000 / portTICK_PERIOD_MS);
	    	continue;
	    }

	    setiGateSocket(serverSocket);

	    // wait for the login "prompt"
	    cnt = igateReadLine((uint8_t *) rBuff, sizeof (rBuff));
	    if (cnt < 0) {
	    	// XXX: error; close socket, retry
	    	// XXX: log this
		    setiGateSocket(-1);
		    close(serverSocket);
		    iGateOK = false;
			vTaskDelay(10000 / portTICK_PERIOD_MS);
	    	continue;
	    }

	    /*
	     * Login to APRS-IS
	     * XXX: make this configurable
	     */
	    char *login = "user XXXX pass XXXX vers k6jq-test v0.1 filter m/200 b/ECHO\r\n";
	    // XXX: check error return
	    // XXX: odds are the read() will fail anyway in this case
	    send(getiGateSocket(), login, strlen(login), 0);
	    // is it necessary to wait here?
	    // vTaskDelay(7000 / portTICK_PERIOD_MS);

	    iGateOK = true;

	    do {
	    	cnt = igateReadLine((uint8_t *) rBuff, sizeof (rBuff));
	    	if (cnt > 0) {
	    		//printf("IS: %.*s\n", cnt, rBuff);
	    		iGateToRF(rBuff, cnt);
	    	}
	    } while (cnt > 0);

	    // XXX: improved log message
	    setiGateSocket(-1);
	    close(serverSocket);
	    iGateOK = false;

		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

/*
 * Outbound IS frames are handled by this tasks to make sure they are
 * serialized.
 */
static void
iGateOutTask(void *arg)
{

	while (true) {
		tnc2buf_t *tb;

        /* assume valid return */
        (void)xQueueReceive(isOutQueue, &tb, portMAX_DELAY);

    	if (!iGateOK) {
    		freeTNC2Buf(tb);
    		return;
    	}

		/* avoid a temporary buffer here; let LwIP do it */
		send(getiGateSocket(), tb->srcAddr, strlen((char *) tb->srcAddr), 0);
		send(getiGateSocket(), ">", 1, 0);
		send(getiGateSocket(), tb->destAddr, strlen((char *) tb->destAddr), 0);
		if (tb->digiPath) {
			send(getiGateSocket(), ",", 1, 0);
			send(getiGateSocket(), tb->digiPath, strlen((char *) tb->digiPath), 0);
		}
		send(getiGateSocket(), ",qAR,", 5, 0);
		send(getiGateSocket(), myCallsign, strlen((char *) myCallsign), 0);
		send(getiGateSocket(), ":", 1, 0);
		send(getiGateSocket(), tb->msg, tb->msgLen, 0);
		send(getiGateSocket(), "\r\n", 2, 0);

		freeTNC2Buf(tb);
	}
}

/*
 *
 */
void
iGateToIS(kbuf_t *kb)
{
	tnc2buf_t *tb;

	/*
	 * get a TNC2 buffer
	 */
	if ((tb = ax25ToTNC2(kb)) == NULL) {
		/* XXX: metric? */
		return;
	}

	/* Update RF-heard table */
	iGateHeardRf(tb);

	/* digipeat */
	digipeat(tb);

retry:
	/* discard zero-length messages */
	if (tb->msgLen == 0) {
		/* XXX: metric? */
		// puts("iGateToIS: zero-length message");
		freeTNC2Buf(tb);
		return;
	}

	/* XXX: path elements */
	if (iGateToISPathFilter(tb)) {
		// XXX: metric
		// puts("iGateToIS: prohibited path");
		freeTNC2Buf(tb);
		return;
	}

	/* third-party; strip header and retry */
	if (tb->msg[0] == '}') {
		tnc2buf_t *tb2;

		tb2 = textToTNC2(tb->msg + 1, tb->msgLen - 1);
		freeTNC2Buf(tb);
		if (tb2 == NULL) {
			return;
		}
		tb = tb2;
		goto retry;
	}

	/*
	 * Filter out 'general' queries
	 * XXX: possibly answer them?
	 */
	if ((strcmp((char *)tb->msg, "?ARPS?") == 0) ||
	  (strcmp((char *)tb->msg, "?IGATE") == 0) ||
	  (strcmp((char *)tb->msg, "?WX?") == 0)) {
		puts("iGateToIS: general query");
		freeTNC2Buf(tb);
		return;
	}

	/*
	 * Filter out our own callsign
	 */
	if (strcmp((char *)tb->srcAddr, (char *)myCallsign) == 0) {
		// puts("iGateToIS: own callsign");
		freeTNC2Buf(tb);
		return;
	}

	/*
	 * OK to iGate now
	 */
#if 0
	/* avoid a temporary buffer here; let LwIP do it */
	send(getiGateSocket(), tb->srcAddr, strlen((char *) tb->srcAddr), 0);
	send(getiGateSocket(), ">", 1, 0);
	send(getiGateSocket(), tb->destAddr, strlen((char *) tb->destAddr), 0);
	if (tb->digiPath) {
		send(getiGateSocket(), ",", 1, 0);
		send(getiGateSocket(), tb->digiPath, strlen((char *) tb->digiPath), 0);
	}
	send(getiGateSocket(), ",qAR,", 5, 0);
	send(getiGateSocket(), myCallsign, strlen((char *) myCallsign), 0);
	send(getiGateSocket(), ":", 1, 0);
	send(getiGateSocket(), tb->msg, tb->msgLen, 0);
	send(getiGateSocket(), "\r\n", 2, 0);

	freeTNC2Buf(tb);
#endif

	/* send frame to outbound IS queue */
	if (xQueueSend(isOutQueue, &tb, 0) != pdPASS) {
		// XXX: metrics
		freeTNC2Buf(tb);
		puts("iGateToIS: queue full");
	}
}

/*
 * igate start -- make TX-igate allocations and inits
 */
void
igate_start()
{
	BaseType_t err;

	// Always relay all traffic from RF to APRSIS, other
	// direction is handled per transmitter interface...
	// enable_aprsis_rx_dupecheck();

	LIST_INIT(&heardCache);
	iGateSocketMutex = xSemaphoreCreateMutex();
    isOutQueue = xQueueCreate(10, sizeof (tnc2buf_t *));


    /*
     * iGate in-bound task
     */
    err = xTaskCreate(iGateInTask, "iGate-in",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 2,
	  &iGateInTaskHandle);

    /*
     * iGate out-bound task
     */
    err = xTaskCreate(iGateOutTask, "iGate-out",
      ESP_TASK_MAIN_STACK,
      ( void * ) 1,
      ESP_TASK_PRIO_MAX - 2,
	  &iGateOutTaskHandle);
}

/*
 *
 */
void
igateStart()
{
	// XXX:
	igate_start();
}

