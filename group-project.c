/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author:  Romain Jacob
 */

/*---------------------------------------------------------------------------*/
/* general */
#include "contiki.h"
#include "node-id.h"
/* GPIO */
#include "gpio.h"
/* radio */
#include "basic-radio.h"
/* data generator */
#include "data-generator.h"
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE    "DesignProjectApp"
#define LOG_LEVEL LOG_LEVEL_MAIN
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
/* Makefile variables */
#ifndef DATARATE
#error No data rate specified. Example: use '-DDATARATE=10' to configure a rate of 10 packets per second.
#endif /* DATARATE */
uint16_t datarate = DATARATE;

#ifndef SINK_ADDRESS
#error No sink address specified. Example: use '-DSINK_ADDRESS=1' to configure the sink node ID.
#endif /* SINK_ADDRESS */
uint16_t sinkaddress = SINK_ADDRESS;

#ifndef RANDOM_SEED
#error No random seed specified. Example: use '-RANDOM_SEED=123' initialize the random number generator.
#endif /* RANDOM_SEED */
uint16_t randomseed = RANDOM_SEED;
/*---------------------------------------------------------------------------*/

/* Structs for the different packets */
// Sync packet
typedef struct {
	uint8_t						sync_count;
} lpsd_sync_t;
// Normal packet
typedef struct {
	uint16_t					src_id;
	uint8_t						seqn;
	uint16_t					payload;
} lpsd_packet_t;

/*---------------------------------------------------------------------------*/
PROCESS(design_project_process, "Skeleton code - LPSD Design Project");
AUTOSTART_PROCESSES(&design_project_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(design_project_process, ev, data)
{
	/* --- Packets --- */
	//Syncronization Packet
	static lpsd_sync_t			sync_packet;					/* packet buffer */
	static lpsd_sync_t			sync_packet_rcv;				/* received packet buffer */
	//Normal Packet
	static lpsd_packet_t*		packet;							/* packet pointer */
	static lpsd_packet_t*		packet_rcv;						/* received packet buffer */

	static uint8_t				packet_len;						/* packet length, in Bytes */
	static uint16_t				timeout_ms = 30;				/* packet receive timeout, in ms */
	static uint16_t				timeout_sink_ms = 47;			/* packet sink receive timeout, in ms */
	static uint8_t				slot_number = 27;
	static uint32_t				slot_time = CLOCK_SECOND / (slot_number + 1);
	static uint8_t				firstpacket = 1;				/* First packet for the initiator */
	static struct etimer		sync_timer;
	static struct etimer		wait_timer;
	static struct etimer		first_wait_timer;
	static struct etimer		slot_timer;
	static uint8_t				sync = 10;						/* in minimum 3 rounds */
	static uint8_t				last_sync = 0;
	static uint8_t				first_sync = 0;
	static uint8_t				i = 0;
	static clock_time_t			last_time = 0;
	static clock_time_t			first_time = 0;
	static clock_time_t			timestamp;

	static uint8_t				my_slot;						/* used slot ID */
	static uint8_t				slot_mapping[slot_number] = {8,2,3,4,6,7,1,10,11,13,14,15,31,17,18,19,20,22,23,24,25,26,27,28,16,32,33};
	static uint8_t				slots[slot_number] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	PROCESS_BEGIN();

	/* initialize the data generator */
	data_generation_init();

	/* configure GPIO as outputs */
	//PIN_CFG_OUT(RADIO_START_PIN);
	PIN_CFG_OUT(RADIO_RX_PIN);
	PIN_CFG_OUT(RADIO_TX_PIN);
	PIN_CFG_OUT(LED_STATUS);

	/* Setup periodic timers that expire after 10/50/1000 milli-seconds and the slottimer. */
	etimer_set(&first_wait_timer, CLOCK_SECOND / 20);			// 50 milliseconds
	etimer_set(&wait_timer, CLOCK_SECOND / 200);				// 5 milliseconds
	etimer_set(&sync_timer, CLOCK_SECOND);						// 1 second
	etimer_set(&slot_timer, slot_time);							// slot time 35 ms 
	/* set my_slot */
	i = 0;
	while(i < slot_number) {
		if(node_id == slot_mapping[i]) {
			my_slot = i;
			slots[i] = 1;
		}
		++i;
	}

	etimer_restart(&sync_timer);

	while(sync) {
		if(firstpacket && node_id == sinkaddress) {
			/* --- INITIATOR --- */
			/* Wait 50 ms to be sure that all other nodes are ready. */
			etimer_restart(&first_wait_timer);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&first_wait_timer));

			/* prepare first packet */
			firstpacket = 0;
			sync_packet.sync_count = 0;
			packet_len = sizeof(sync_packet);
			--sync;

			/* get Timestamp and send first packet */
			timestamp = clock_time();
			radio_send(((uint8_t*)&sync_packet),packet_len,1);
			LOG_INFO("sync_round: %u\n", sync_packet.sync_count);
		} else {
			/* --- FORWARDER --- */

			LOG_INFO("Listening...");
			while(1) {
				packet_len = radio_rcv(((uint8_t*)&sync_packet_rcv), timeout_ms);
				if(packet_len){
					break;
				}
			}
			/* Restart the timer */
			etimer_restart(&wait_timer);

			LOG_INFO("receive_packet_round: %u\n",sync_packet_rcv.sync_count);

			if(timestamp) {
				if(!first_sync) {
					first_sync = sync_packet.sync_count;
					first_time = timestamp;
				} else {
					last_sync = sync_packet.sync_count;
					last_time = timestamp;
				}
			}

			/* increment counter and resend packet */
			sync_packet = sync_packet_rcv;
			++sync_packet.sync_count;	
			packet_len = sizeof(sync_packet);
			--sync;

			/* Wait for send */
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&wait_timer));

			/* get Timestamp and send packet */
			timestamp = clock_time();
			radio_send(((uint8_t*)&sync_packet),packet_len,1);
			LOG_INFO("send_packet_round: %u\n",sync_packet.sync_count);
		}
	}

	/* calculate t zero and set the sync_timer */
	clock_time_t delta_t = (last_time - first_time) / (last_sync - first_sync);
	clock_time_t t_zero = first_time - (first_sync * delta_t);

	etimer_adjust(&sync_timer, (int16_t) (t_zero - etimer_start_time(&sync_timer)));

	/* ----------------------- HERE WE ARE SYNCED ----------------------- */
	if(sinkaddress == 22) {
		/* --- Scenario 1 --- */
		if(node_id == 3) {
			slots[7] = 1;				// 10
			slots[11] = 1;				// 15
		} else if(node_id == 28) {
			slots[0] = 1;				// 8
			slots[12] = 1;				// 31
		} else if(node_id == 31) {
			slots[25] = 1;				// 32
		} else if(node_id == 33) {
			slots[6] = 1;				// 1
			slots[1] = 1;				// 2
			slots[3] = 1;				// 4
		}
	} else {
		/* --- Scenario 2 --- */

		//TODO
		// - discover Network
		// - set parents
	}

	if(node_id == sinkaddress) {
		while(1) {
			if(is_data_in_queue()) {
				/* --- SINK --- */
				/* Write our own message to serial */
				packet = pop_data();
				LOG_INFO("Pkt:%u,%u,%u\n", packet->src_id,packet->seqn, packet->payload);
			}
			packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_sink_ms);
			if(packet_len) {
				LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv->src_id,packet_rcv->seqn, packet_rcv->payload);
			}
		}
	} else {
		while(1) {
			/* go through all event timers that have to be listen to */
			/* wait for sync timer to expire */
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sync_timer));
			/* reset sync timer and restart all slot timers */
			etimer_restart(&slot_timer);
			etimer_reset(&sync_timer);
			i = 0;
			while(i < slot_number) {
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&slot_timer));
				etimer_reset(&slot_timer);
				if(slots[i]) {
					if(my_slot == i && is_data_in_queue()) {
						packet = pop_data();
						/* --- SOURCE --- */
						radio_send(((uint8_t*)packet),sizeof(lpsd_packet_t),1);
						LOG_INFO("TRM Pkt:%u,%u,%u\n", packet->src_id,packet->seqn, packet->payload);
					} else if(my_slot != i){
						packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
						if(packet_len) {
							LOG_INFO("REC Pkt:%u,%u,%u\n", packet_rcv->src_id,packet_rcv->seqn, packet_rcv->payload);
						}
					}
				}
				++i;
			}

			//TODO
			// -reason to break the while loop
			if(0) break;
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
