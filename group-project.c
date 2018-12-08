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
#include "memb.h"
#include "queue.h"
/* GPIO */
#include "gpio.h"
/* clock */
#include "clock.h"
/* radio */
#include "basic-radio.h"
/* data generator */
#include "data-generator.h"
#include "rtimer-ext.h"
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


static volatile uint8_t i = 0;
static volatile rtimer_ext_clock_t t_zero = 0;
static volatile uint8_t sync = 10;						/* in minimum 3 rounds */
/*---------------------------------------------------------------------------*/

/* Structs for the different packets */
// Sync packet
typedef struct {
	uint8_t						sync_count;
} lpsd_sync_t;
/* Packet type for queue */
typedef struct {
	uint16_t					src_id;
	uint8_t						seqn;
	uint16_t					payload;
} lpsd_packet_t;
// Super packet
typedef struct {
	uint16_t					src_id[4];
	uint8_t						seqn[20];
	uint16_t					payload[20];
	uint8_t 					size;
} lpsd_superpacket_t;
// Network discovery packet
typedef struct {
	uint16_t					src_id;					// sender of this message
	uint16_t					dst_id;					// target of this sender
	uint8_t 					size;					// tell the number of packets
} lpsd_discovery_t;
// Writing queue
QUEUE(writing_queue);
MEMB(writing_memb, lpsd_packet_queue_t, 200);
/*---------------------------------------------------------------------------*/
/* --- Packets --- */
//Syncronization Packet
static lpsd_sync_t			sync_packet;					/* packet buffer */
static lpsd_sync_t			sync_packet_rcv;				/* received packet buffer */
//Normal Packet
static lpsd_superpacket_t	packet;							/* packet pointer */
static lpsd_packet_t*		pop_packet;						/* packet pointer */
static lpsd_superpacket_t	packet_rcv;						/* received packet buffer */
static uint8_t				packet_len;						/* packet length, in Bytes */
static uint16_t				timeout_ms;						/* packet receive timeout, in ms */
static uint8_t				firstpacket;					/* First packet for the initiator */
static uint8_t				last_sync;
static uint8_t				first_sync;
static rtimer_ext_clock_t	last_time;
static rtimer_ext_clock_t	first_time;
static rtimer_ext_clock_t	timestamp;
static uint8_t				stop;
static uint8_t				seqn;

static uint8_t				my_slot;						/* used slot ID */
static uint8_t				slot_mapping[27];
static uint8_t				slots[27];


/* Functions */
/*void reset_sync_timer(void)
{
	//radio_rcv(((uint8_t*)&i), 2);
	i = 0;
}*/
void reset_slot_timer(void)
{
	if(node_id == sinkaddress) {
		while(is_data_in_queue()) {
			// --- SINK ---
			// Write our own message to serial
			pop_packet = pop_data();
			seqn = pop_packet->seqn;
			LOG_INFO("Pkt:%u,%u,%u\n", pop_packet->src_id,pop_packet->seqn, pop_packet->payload);
		}
		if(my_slot == i) {
			uint8_t break_counter = 0;
			while(*writing_queue != NULL && break_counter < 50) {
				// dequeue the first packet
	  			lpsd_packet_queue_t* pkt = queue_dequeue(writing_queue);
	  			// free the memory block
	  			memb_free(&writing_memb, pkt);

	  			LOG_INFO("Pkt:%u,%u,%u\n", pkt->src_id,pkt->seqn, pkt->payload);
	  			++break_counter;
			}
			if(seqn == 200) ++stop;
		} else {
			packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
			if(packet_len) {
				while(packet_rcv.size > 0) {
					uint8_t counter = 0;
					while(counter < 5) {
						uint8_t read_val = ((packet_rcv.size - 1) * 5) + counter;
						if(packet_rcv.seqn[read_val]) {
							lpsd_packet_queue_t* writing_pkt = memb_alloc(&writing_memb);
							if(writing_pkt == 0) {
								LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id[(packet_rcv.size - 1)],packet_rcv.seqn[read_val], packet_rcv.payload[read_val]);
								++stop;
							} else {
								writing_pkt->src_id   = packet_rcv.src_id[(packet_rcv.size - 1)];
								writing_pkt->seqn     = packet_rcv.seqn[read_val];
								writing_pkt->payload  = packet_rcv.payload[read_val];

								// add packet to the queue
								queue_enqueue(writing_queue, writing_pkt);
							}

							if(stop > 5) {
								stop = 0;
								break;
							}
						}
						++counter;
					}
					--packet_rcv.size;
				}
			}
		}
	} else {
		// reset sync timer and restart all slot timers
		if(slots[i]) {
			if(my_slot == i) {
				uint8_t counter = 0;
				while(is_data_in_queue() && counter < 5) {
					pop_packet = pop_data();

					LOG_INFO("POP Pkt:%u,%u,%u\n", pop_packet->src_id,pop_packet->seqn, pop_packet->payload);

					packet.src_id[0] = pop_packet->src_id;
					packet.seqn[counter] = pop_packet->seqn;
					packet.payload[counter] = pop_packet->payload;
					seqn = pop_packet->seqn;

					++counter;
				}
				while(counter < 5) {
					packet.seqn[counter] = 0;
					packet.payload[counter] = 0;

					++counter;
				}
				if(seqn == 200) ++stop;
				// --- SOURCE ---
				radio_send(((uint8_t*)(&(packet.src_id[0]))),sizeof(lpsd_superpacket_t),1);
				packet.size = 1;
			} else if(my_slot != i){
				packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
				if(packet_len) {
					while(packet_rcv.size > 0) {
						uint8_t counter = 0;
						while(counter < 5) {
							uint8_t read_val = ((packet_rcv.size - 1) * 5) + counter;
							uint8_t write_val = (packet.size * 5) + counter;
							
							packet.src_id[packet.size] = packet_rcv.src_id[(packet_rcv.size - 1)];
							packet.seqn[write_val] = packet_rcv.seqn[read_val];
							packet.payload[write_val] = packet_rcv.payload[read_val];
							++counter;
						}
						if(seqn == 200) ++stop;
						--packet_rcv.size;
						++packet.size;
					}
				}
			}
		}
		//TODO
		// -reason to break the while loop
	}
	++i;
	if(i == 27) i = 0;
}
void schedule_sync_timer(void)
{
	//clock_delay((uint16_t) 11.0424028 * t_zero);
	if(!t_zero) t_zero = 1130;
	rtimer_ext_stop(RTIMER_EXT_LF_0);
	rtimer_ext_reset();
	//rtimer_ext_schedule(RTIMER_EXT_LF_1, t_zero, RTIMER_EXT_SECOND_LF, (rtimer_ext_callback_t) &reset_sync_timer);
	rtimer_ext_schedule(RTIMER_EXT_LF_2, t_zero, (RTIMER_EXT_SECOND_LF/27), (rtimer_ext_callback_t) &reset_slot_timer);
	rtimer_ext_clock_t exp_time;
	rtimer_ext_next_expiration(RTIMER_EXT_LF_1, &exp_time);

	LOG_INFO("T_ZERO: %u\n",(uint16_t) t_zero);

	if(sync) {
		LOG_INFO("Not synced --> try with average t_zero %u.", (uint16_t) t_zero);
		sync = 0;
	}

	/* initialize the data generator */
	data_generation_init();
}

/*---------------------------------------------------------------------------*/
PROCESS(design_project_process, "Flocklab Multi-Hop by Alex and Dario");
AUTOSTART_PROCESSES(&design_project_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(design_project_process, ev, data)
{
	PROCESS_BEGIN();

	packet.size = 1;
	timeout_ms = 25;
	firstpacket = 1;
	last_sync = 0;
	first_sync = 0;
	last_time = 0;
	first_time = 0;
	stop = 0;
	seqn = 0;

	slot_mapping[0] = 8;
	slot_mapping[1] = 2;
	slot_mapping[2] = 3;
	slot_mapping[3] = 4;
	slot_mapping[4] = 6;
	slot_mapping[5] = 7;
	slot_mapping[6] = 1;
	slot_mapping[7] = 10;
	slot_mapping[8] = 11;
	slot_mapping[9] = 13;
	slot_mapping[10] = 14;
	slot_mapping[11] = 15;
	slot_mapping[12] = 31;
	slot_mapping[13] = 17;
	slot_mapping[14] = 18;
	slot_mapping[15] = 19;
	slot_mapping[16] = 20;
	slot_mapping[17] = 22;
	slot_mapping[18] = 23;
	slot_mapping[19] = 24;
	slot_mapping[20] = 25;
	slot_mapping[21] = 26;
	slot_mapping[22] = 27;
	slot_mapping[23] = 28;
	slot_mapping[24] = 16;
	slot_mapping[25] = 32;
	slot_mapping[26] = 33;

	slots[1] = 0;
	slots[2] = 0;
	slots[3] = 0;
	slots[4] = 0;
	slots[5] = 0;
	slots[6] = 0;
	slots[7] = 0;
	slots[8] = 0;
	slots[9] = 0;
	slots[10] = 0;
	slots[11] = 0;
	slots[12] = 0;
	slots[13] = 0;
	slots[14] = 0;
	slots[15] = 0;
	slots[16] = 0;
	slots[17] = 0;
	slots[18] = 0;
	slots[19] = 0;
	slots[20] = 0;
	slots[21] = 0;
	slots[22] = 0;
	slots[23] = 0;
	slots[24] = 0;
	slots[25] = 0;
	slots[26] = 0;

	/* initialize the writing queue */
  	memb_init(&writing_memb);
  	queue_init(writing_queue);

	/* configure GPIO as outputs */
	//PIN_CFG_OUT(RADIO_START_PIN);
	PIN_CFG_OUT(RADIO_RX_PIN);
	PIN_CFG_OUT(RADIO_TX_PIN);
	PIN_CFG_OUT(LED_STATUS);

	rtimer_ext_schedule(RTIMER_EXT_LF_0, 0, RTIMER_EXT_SECOND_LF, (rtimer_ext_callback_t) &schedule_sync_timer);

	/* set my_slot */
	i = 0;
	while(i < 27) {
		if(node_id == slot_mapping[i]) {
			my_slot = i;
			slots[i] = 1;
		}
		++i;
	}
	while(sync) {
		if(firstpacket && node_id == sinkaddress) {
			/* --- INITIATOR --- */
			// wait 50 ms
			clock_delay(17668);

			/* prepare first packet */
			firstpacket = 0;
			sync_packet.sync_count = 0;
			packet_len = sizeof(sync_packet);
			--sync;

			/* get Timestamp and send first packet */
			//timestamp = rtimer_ext_now_lf();
			radio_send(((uint8_t*)&sync_packet),packet_len,1);
			LOG_INFO("sync_round: %u\n", sync_packet.sync_count);
		} else {
			/* --- FORWARDER --- */

			//LOG_INFO("Listening...");
			while(1) {
				packet_len = radio_rcv(((uint8_t*)&sync_packet_rcv), timeout_ms);
				if(packet_len){
					break;
				}
			}

			// wait 5 ms
			clock_delay(1767);

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

			/* get Timestamp and send packet */
			timestamp = rtimer_ext_now_lf();
			radio_send(((uint8_t*)&sync_packet),packet_len,1);
			//LOG_INFO("send_packet_round: %u\n",sync_packet.sync_count);
		}
	}
	/* calculate t zero and set the sync_timer */
	rtimer_ext_clock_t delta_t = (last_time - first_time) / (uint64_t) (last_sync - first_sync);
	t_zero = first_time - ((uint64_t) first_sync * delta_t);

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

	while(1) {
		if(stop >= 5) break;
	}

	if(node_id == sinkaddress) {
		while(*writing_queue != NULL) {
			/* dequeue the first packet */
  			lpsd_packet_queue_t* pkt = queue_dequeue(writing_queue);
  			/* free the memory block */
  			memb_free(&writing_memb, pkt);

  			LOG_INFO("Pkt:%u,%u,%u\n", pkt->src_id,pkt->seqn, pkt->payload);
		}
	} else {
		LOG_INFO("no new packets --> going to LPM4.");
		LPM4;
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
