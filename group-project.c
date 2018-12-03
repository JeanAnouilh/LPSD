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
//static uint8_t packcounter = 0;
static volatile uint8_t i = 0;
static volatile uint8_t j = 1;
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
//static lpsd_packet_t instancepacket;
//instancepacket.src_id = 0;
//instancepacket.seqn = 0;
//instancepacket.payload = 0;
typedef struct {
	lpsd_packet_t				single_packet[4];
	uint8_t 					size;
} lpsd_superpacket_t;
	

/*---------------------------------------------------------------------------*/

/* Functions */
void reset_sync_timer(void)
{
	i = 0;
}
void reset_slot_timer(void)
{
	j = 1;
}

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
	static lpsd_superpacket_t*		packet;							/* packet pointer */
	static lpsd_packet_t*			poppacket;						/* packet pointer */
	static lpsd_superpacket_t		packet_rcv;						/* received packet buffer */
	//static lpsd_superpacket_t		received_packets;				
	packet->size=0;
	packet_rcv.size=0;
	static lpsd_packet_t instancepacket;
	packet->single_packet[0]= instancepacket;
	packet->single_packet[1]= instancepacket;
	packet->single_packet[2]= instancepacket;
	packet->single_packet[3]= instancepacket;
	packet_rcv.single_packet[0]= instancepacket;
	packet_rcv.single_packet[1]= instancepacket;
	packet_rcv.single_packet[2]= instancepacket;
	packet_rcv.single_packet[3]= instancepacket;
	static uint8_t				packet_len;						/* packet length, in Bytes */
	static uint16_t				timeout_ms = 25;				/* packet receive timeout, in ms */
	static uint32_t				slot_time = CLOCK_SECOND / 28;
	static uint8_t				firstpacket = 1;				/* First packet for the initiator */
	static uint8_t				sync = 10;						/* in minimum 3 rounds */
	static uint8_t				last_sync = 0;
	static uint8_t				first_sync = 0;
	static rtimer_ext_clock_t	last_time = 0;
	static rtimer_ext_clock_t	first_time = 0;
	static rtimer_ext_clock_t	timestamp;

	static uint8_t				send_counter = 0;
	static uint8_t				rec_counter = 0;

	static uint8_t				my_slot;						/* used slot ID */
	static uint8_t				slot_mapping[27] = {8,2,3,4,6,7,1,10,11,13,14,15,31,17,18,19,20,22,23,24,25,26,27,28,16,32,33};
	static uint8_t				slots[27] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	PROCESS_BEGIN();

	/* initialize the data generator */
	data_generation_init();

	/* configure GPIO as outputs */
	//PIN_CFG_OUT(RADIO_START_PIN);
	PIN_CFG_OUT(RADIO_RX_PIN);
	PIN_CFG_OUT(RADIO_TX_PIN);
	PIN_CFG_OUT(LED_STATUS);

	/* set my_slot */
	i = 0;
	while(i < 27) {
		if(node_id == slot_mapping[i]) {
			my_slot = i;
			slots[i] = 1;
		}
		++i;
	}
	rtimer_ext_schedule(RTIMER_EXT_LF_1, 0, RTIMER_EXT_SECOND_LF, (rtimer_ext_callback_t) &reset_sync_timer);
	while(sync) {
		if(firstpacket && node_id == sinkaddress) {
			/* --- INITIATOR --- */
			uint8_t waiter = 100;
			while(waiter) {
				LOG_INFO("waiting\n");
				--waiter;
			}

			/* prepare first packet */
			firstpacket = 0;
			sync_packet.sync_count = 0;
			packet_len = sizeof(sync_packet);
			--sync;

			/* get Timestamp and send first packet */
			timestamp = rtimer_ext_now_lf();
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
			/* Restart the timer */
			LOG_INFO("waiting\n");
			LOG_INFO("waiting\n");
			LOG_INFO("waiting\n");
			LED_ON(LED_STATUS);

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
			LOG_INFO("waiting\n");
			LOG_INFO("waiting\n");
			LOG_INFO("waiting\n");
			LED_OFF(LED_STATUS);

			/* get Timestamp and send packet */
			timestamp = rtimer_ext_now_lf();
			radio_send(((uint8_t*)&sync_packet),packet_len,1);
			//LOG_INFO("send_packet_round: %u\n",sync_packet.sync_count);
		}
	}
	/* calculate t zero and set the sync_timer */
	rtimer_ext_clock_t delta_t = (last_time - first_time) / (uint64_t) (last_sync - first_sync);
	rtimer_ext_clock_t t_zero = first_time - ((uint64_t) first_sync * delta_t);
	rtimer_ext_clock_t next_exp;
	rtimer_ext_next_expiration(RTIMER_EXT_LF_1, &next_exp);
	LOG_INFO("START: %u",(uint16_t) (t_zero + next_exp));

	rtimer_ext_schedule(RTIMER_EXT_LF_1, t_zero+RTIMER_EXT_SECOND_LF, RTIMER_EXT_SECOND_LF, (rtimer_ext_callback_t) &reset_sync_timer);

	LOG_INFO("WE ARE SYNCED");

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
		//LOG_INFO("callback_counter: %u",counter);
		if(node_id == sinkaddress) {
			/* reset sync timer and restart all slot timers */
			if(i == 0) {
				rtimer_ext_schedule(RTIMER_EXT_LF_2, 0, (RTIMER_EXT_SECOND_LF/27), (rtimer_ext_callback_t) &reset_slot_timer);
				LED_TOGGLE(LED_STATUS);
				while(i < 27) {
					while(1) {
						if(j) {
							if(is_data_in_queue()) {
								/* --- SINK --- */
								/* Write our own message to serial */
								poppacket = pop_data();
								LOG_INFO("Pkt:%u,%u,%u\n", poppacket->src_id,poppacket->seqn, poppacket->payload);
							}
							packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
							if(packet_len) {
								while(packet_rcv.size > 0) {
									LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.single_packet[(4-packet_rcv.size)].src_id,packet_rcv.single_packet[(4-packet_rcv.size)].seqn, packet_rcv.single_packet[(4-packet_rcv.size)].payload);
									--packet_rcv.size;
								}
							}
							j = 0;
							break;
						}
					}
					++i;
				}
			}
		} else {
			//LOG_INFO("callback:%u, i:%u\n",counter,i);
			/* reset sync timer and restart all slot timers */
			packet->size = 0;
			if(i == 0) {
				rtimer_ext_schedule(RTIMER_EXT_LF_2, 0, (RTIMER_EXT_SECOND_LF/27), (rtimer_ext_callback_t) &reset_slot_timer);
				while(i < 27) {
					while(1) {
						if(j) {
							if(slots[i]) {
								if(my_slot == i && is_data_in_queue()) {
									poppacket = pop_data();
									packet->single_packet[send_counter] = *poppacket;
									++send_counter;
									packet->size = send_counter;
									/* --- SOURCE --- */
									radio_send(((uint8_t*)packet),sizeof(lpsd_superpacket_t),1);
									send_counter = 0;
									LOG_INFO("TRM Pkt Size :%u\n", packet->size);
								} else if(my_slot != i){
									packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
									if(packet_len) {
										rec_counter = packet_rcv.size;
										while(rec_counter > 0){
											LOG_INFO("REC Pkt:%u,%u,%u\n", packet_rcv.single_packet[(4-rec_counter)].src_id,packet_rcv.single_packet[(4-rec_counter)].seqn, packet_rcv.single_packet[(4-rec_counter)].payload);
											packet->single_packet[send_counter] = packet_rcv.single_packet[(4-rec_counter)];
											--rec_counter;
											++send_counter;
										}
										packet->size = send_counter;
									}
								}
							}
							j = 0;
							break;
						}
					}
					++i;
				}
			}

			//TODO
			// -reason to break the while loop
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
