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
// Single data value 
typedef struct {
	uint16_t					src_id;
	uint8_t						seqn;
	uint16_t					payload;
	uint8_t						error_correction;
} lpsd_data_struct_t;
// Ten data values collected
typedef struct {
	lpsd_data_struct_t[10]		data_payload;
	uint16_t					error_correction;
} lpsd_data_t;
// Ten data packets collected
typedef struct {
	lpsd_data_t[10]				super_payload;
	uint32_t					error_correction;
} lpsd_super_t;
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
	//Structure Packet
	static lpsd_data_struct_t	structure_packet;				/* packet buffer */
	//Data Packet
	static lpsd_data_t			data_packet;					/* packet buffer */
	//Super Packet
	static lpsd_super_t			super_packet;					/* packet buffer */
	static lpsd_super_t			super_packet_rcv;				/* received packet buffer */
	//Normal Packet
	static lpsd_packet_t*		packet;							/* packet pointer */

	static uint8_t				packet_len;						/* packet length, in Bytes */
	static uint16_t				timeout_ms = 100;				/* packet receive timeout, in ms */
	static uint32_t				slot_time = CLOCK_SECOND / 28;
	static uint8_t				firstpacket = 1;				/* First packet for the initiator */
	static struct etimer		sync_timer;
	static struct etimer		wait_timer;
	static struct etimer		first_wait_timer;
	static struct etimer[27]	slot_timer;
	static uint8_t				sync = 10;						/* in minimum 3 rounds */
	static uint8_t				last_sync = 0;
	static uint8_t				first_sync = 0;
	static uint8_t				test_count = 0;
	static clock_time_t			last_time = 0;
	static clock_time_t			first_time = 0;
	static clock_time_t			timestamp;

	static uint8_t				my_slot;						/* used slot ID */
	static uint8_t[27]			slot_mapping = {1,2,3,4,6,7,8,10,11,13,14,15,16,17,18,19,20,22,23,24,25,26,27,28,31,32,33};
	static uint8_t[27]			slots;

	PROCESS_BEGIN();

	/* initialize the data generator */
	data_generation_init();

	/* configure GPIO as outputs */
	//PIN_CFG_OUT(RADIO_START_PIN);
	PIN_CFG_OUT(RADIO_RX_PIN);
	PIN_CFG_OUT(RADIO_TX_PIN);
	PIN_CFG_OUT(LED_STATUS);

	/* Setup periodic timers that expire after 10/50/1000 milli-seconds. */
	etimer_set(&first_wait_timer, CLOCK_SECOND / 20);			// 50 milliseconds
	etimer_set(&wait_timer, CLOCK_SECOND / 100);				// 10 milliseconds
	etimer_set(&sync_timer, CLOCK_SECOND);						// 1 second

	/* set my_slot and all slot timers */
	uint8_t i = 0;
	while(i < 27) {
		etimer_set(&slot_timer[i], slot_time * (i + 1));
		if(node_id == slot_mapping[i]) {
			my_slot = i + 1;
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
		if(node_id == 1) {
		} else if(node_id == 3) {
			slots[7] = 1;				// 10
			slots[11] = 1;				// 15
		} else if(node_id == 28) {
			slots[6] = 1;				// 8
			slots[24] = 1;				// 31
		} else if(node_id == 31) {
			slots[25] = 1;				// 32
		} else if(node_id == 33) {
			slots[0] = 1;				// 1
			slots[1] = 1;				// 2
			slots[3] = 1;				// 4
		}
	} else {
		/* --- Scenario 2 --- */

		//TODO
		// - discover Network
		// - set parents
	}

	while(1) {
		uint8_t i = 0;
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sync_timer));
		etimer_reset(&sync_timer);
		while(i < 27) {
			etimer_restart(&slot_timer[i]);
			++i;
		}

		/* prepare the super packet to send */
		uint8_t structure_number = 0;
		uint8_t data_number = 0;
		i = 0;
		while(i < 10) {
			&data_packet.data_payload[i] = null;
			&super_packet.super_payload[data_number] = null;
			++i;
		}

		while(is_data_in_queue()){
			packet = pop_data();
			if(node_id == sinkaddress) {
				/* --- SINK --- */
				/* Write our own message to serial */
				LOG_INFO("Pkt:%u,%u,%u\n", packet->src_id,packet->seqn, packet->payload);
			} else {
				/* --- SOURCE --- */
				/* Prepare our packet */
				if(structure_number == 10) {
					super_packet.super_payload[data_number] = data_packet;
					structure_number = 0;
					++data_number;
					i = 0;
					while(i < 10) {
						&data_packet.data_payload[i] = null;
						++i;
					}
				}
				structure_packet.src_id = packet->src_id;
				structure_packet.seqn = packet->seqn;
				structure_packet.payload = packet->payload;
				data_packet.data_payload[structure_number] = structure_packet;
				++structure_number;
			}
		}
		super_packet.super_payload[data_number] = data_packet;

		i = 0;
		++data_number;
		while(i < 10) {
			if(super_packet_rcv.super_payload[i] != null) {
				super_packet.super_payload[data_number] = super_packet_rcv.super_payload[i];
				++data_number;
			}
			++i;
		}

		/* go through all event timers that have to be listen to */
		i = 0;
		while(i < 27) {
			if(slots[i]) {
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&slot_timer[i]));
				if(my_slot == i) {
					radio_send(((uint8_t*)packet),sizeof(lpsd_packet_t),1);
				} else {
					radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
				}
			}
			++i;
		}

		//TODO
		// -reason to break the while loop
		if(0) break;
	}
	PROCESS_END();
}

handle_packets() {

}
/*---------------------------------------------------------------------------*/
