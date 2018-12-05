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
static volatile uint8_t j = 1;
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

} lpsd_poppacket_t;
// Normal packet
typedef struct {
	uint8_t						packet_size;
	uint8_t						size1;
	uint16_t					src_id1;
	uint8_t						seqn11;
	uint8_t						seqn12;
	uint8_t						seqn13;
	uint8_t						seqn14;
	uint8_t						seqn15;
	uint16_t					payload11;
	uint16_t					payload12;
	uint16_t					payload13;
	uint16_t					payload14;
	uint16_t					payload15;
	uint16_t					src_id2;
	uint8_t						seqn21;
	uint8_t						seqn22;
	uint8_t						seqn23;
	uint8_t						seqn24;
	uint8_t						seqn25;
	uint16_t					payload21;
	uint16_t					payload22;
	uint16_t					payload23;
	uint16_t					payload24;
	uint16_t					payload25;
	uint16_t					src_id3;
	uint8_t						seqn31;
	uint8_t						seqn32;
	uint8_t						seqn33;
	uint8_t						seqn34;
	uint8_t						seqn35;
	uint16_t					payload31;
	uint16_t					payload32;
	uint16_t					payload33;
	uint16_t					payload34;
	uint16_t					payload35;
	uint16_t					src_id4;
	uint8_t						seqn41;
	uint8_t						seqn42;
	uint8_t						seqn43;
	uint8_t						seqn44;
	uint8_t						seqn45;
	uint16_t					payload41;
	uint16_t					payload42;
	uint16_t					payload43;
	uint16_t					payload44;
	uint16_t					payload45;
	
} lpsd_packet_t;

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
	static lpsd_packet_t*		packet;							/* packet pointer */
	static lpsd_poppacket_t*	poppacket;						/* poppacket pointer */
	static lpsd_packet_t		packet_rcv;						/* received packet buffer */
	//static lpsd_superpacket_t		received_packets;				
	//packet->size=0;
	//packet_rcv.size=0;
	static uint8_t				packet_len;						/* packet length, in Bytes */
	static uint16_t				timeout_ms = 25;				/* packet receive timeout, in ms */
	static uint8_t				firstpacket = 1;				/* First packet for the initiator */
	static uint8_t				sync = 10;						/* in minimum 3 rounds */
	static uint8_t				last_sync = 0;
	static uint8_t				first_sync = 0;
	static rtimer_ext_clock_t	last_time = 0;
	static rtimer_ext_clock_t	first_time = 0;
	static rtimer_ext_clock_t	timestamp;

	static uint8_t				load_counter = 5;

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
			/*uint8_t waiter = 100;
			while(waiter) {
				LOG_INFO("waiting\n");
				--waiter;
			}*/
			/* wait 10 ms before going further */
			clock_delay_usec(10000);
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
			/* Wait 5 ms */
			clock_delay_usec(5000);
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

			/* Wait 5 ms for send */
			clock_delay_usec(5000);
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
	//t_zero = 0;

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
		if(node_id == sinkaddress) {
			/* reset sync timer and restart all slot timers */
			if(i == 0) {
				rtimer_ext_schedule(RTIMER_EXT_LF_2, 0, (RTIMER_EXT_SECOND_LF/28), (rtimer_ext_callback_t) &reset_slot_timer);
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
									if(packet_rcv.size == 1)
									{
										if(packet_rcv.seqn11 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id1,packet_rcv.seqn11, packet_rcv.payload11);
										}
										if(packet_rcv.seqn12 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id1,packet_rcv.seqn12, packet_rcv.payload12);
										}
										if(packet_rcv.seqn13 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id1,packet_rcv.seqn13, packet_rcv.payload13);
										}
										if(packet_rcv.seqn14 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id1,packet_rcv.seqn14, packet_rcv.payload14);
										}
										if(packet_rcv.seqn15 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id1,packet_rcv.seqn15, packet_rcv.payload15);
										}
										--packet_rcv.size;
									}
									if(packet_rcv.size == 2)
									{
										if(packet_rcv.seqn21 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id2,packet_rcv.seqn21, packet_rcv.payload21);
										}
										if(packet_rcv.seqn22 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id2,packet_rcv.seqn22, packet_rcv.payload22);
										}
										if(packet_rcv.seqn23 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id2,packet_rcv.seqn23, packet_rcv.payload23);
										}
										if(packet_rcv.seqn24 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id2,packet_rcv.seqn24, packet_rcv.payload24);
										}
										if(packet_rcv.seqn25 != 0)
										{
											LOG_INFO("Pkt:%u,%u,%u\n", packet_rcv.src_id2,packet_rcv.seqn25, packet_rcv.payload25);
										}
										--packet_rcv.size;
									}
									/* insert here same code for packet 3 and 4 ( skipped for debugging reasons)*/





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
				rtimer_ext_schedule(RTIMER_EXT_LF_2, 0, (RTIMER_EXT_SECOND_LF/28), (rtimer_ext_callback_t) &reset_slot_timer);
				//LED_TOGGLE(LED_STATUS);
				while(i < 27) {
					while(1) {
						if(j) {
							if(slots[i]) {
								if(my_slot == i && is_data_in_queue()) {
									/* load the big packet with our own packets*/
									while(is_data_in_queue() && load_counter){
										poppacket = pop_data();
										if(packet->size==0){
											if(load_counter == 5){
												packet->src_id1= poppacket->src_id;
												packet->seqn11 = poppacket->seqn;
												packet->payload11 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 4){
												packet->seqn12 = poppacket->seqn;
												packet->payload12 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 3){
												packet->seqn13 = poppacket->seqn;
												packet->payload13 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 2){
												packet->seqn14 = poppacket->seqn;
												packet->payload14 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 1){
												packet->seqn15 = poppacket->seqn;
												packet->payload15 = poppacket->payload;
												--load_counter;
											}
											++packet->size;
										}
										else if(packet->size == 1){
											if(load_counter == 5){
												packet->src_id2= poppacket->src_id;
												packet->seqn21 = poppacket->seqn;
												packet->payload21 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 4){
												packet->seqn22 = poppacket->seqn;
												packet->payload22 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 3){
												packet->seqn23 = poppacket->seqn;
												packet->payload23 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 2){
												packet->seqn24 = poppacket->seqn;
												packet->payload24 = poppacket->payload;
												--load_counter;
											}
											else if(load_counter == 1){
												packet->seqn25 = poppacket->seqn;
												packet->payload25 = poppacket->payload;
												--load_counter;
											}
											++packet->size;
										}
										else if(packet->size == 2){
											/*inserted here for debugging reasons insert same code as above */
											LOG_INFO("Packet contains already 2 singlepackets\n",);
											--load_counter;
										}
										else if(packet->size == 3){
											/*inserted for debugging reasons */
											LOG_INFO("Packet contains already 3 singlepackets\n",);
											--load_counter;
										}

										



									}
									LOG_INFO("Loading of packet successful Load_counter:%u \n", load_counter);
									/* --- SOURCE --- */
									send_counter = radio_send(((uint8_t*)packet),sizeof(lpsd_packet_t),1);
									/*after sending prepare for a new packet */
									packet->size = 0;
									load_counter = 5;
									LOG_INFO("TRM Send successful :%u time: %u \n", send_counter,t);
								} else if(my_slot != i){
									packet_len = radio_rcv(((uint8_t*)&packet_rcv), timeout_ms);
									LOG_INFO("REC Pkt with size :%u\n", packet_rcv.size);
									if(packet_len) {
										while(packet_rcv.size > 0)
										/*if own packet is empty replace it with the received packet*/
										if(packet->size == 0){
											*packet = packet_rcv;
											packet_rcv.size = 0;
										}
										/*else fill up the own packet */
										else if(packet->size ==1){
											if(packet_rcv.size == 3){
												packet->nodeid2 = packet_rcv.nodeid3;
												packet->seqn21 = packet_rcv.seqn31;
												packet->payload21 = packet_rcv.payload31;
												packet->seqn22 = packet_rcv.seqn32;
												packet->payload22 = packet_rcv.payload32;
												packet->seqn23 = packet_rcv.seqn33;
												packet->payload23 = packet_rcv.payload33;
												packet->seqn24 = packet_rcv.seqn34;
												packet->payload24 = packet_rcv.payload34;
												packet->seqn25 = packet_rcv.seqn35;
												packet->payload25 = packet_rcv.payload35;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 3 to Pkt 2");
											}
											else if(packet_rcv.size == 2){
												packet->nodeid2 = packet_rcv.nodeid2;
												packet->seqn21 = packet_rcv.seqn21;
												packet->payload21 = packet_rcv.payload21;
												packet->seqn22 = packet_rcv.seqn22;
												packet->payload22 = packet_rcv.payload22;
												packet->seqn23 = packet_rcv.seqn23;
												packet->payload23 = packet_rcv.payload23;
												packet->seqn24 = packet_rcv.seqn24;
												packet->payload24 = packet_rcv.payload24;
												packet->seqn25 = packet_rcv.seqn25;
												packet->payload25 = packet_rcv.payload25;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 2 to Pkt 2");
											}
											else if(packet_rcv.size == 1){
												packet->nodeid2 = packet_rcv.nodeid1;
												packet->seqn21 = packet_rcv.seqn11;
												packet->payload21 = packet_rcv.payload11;
												packet->seqn22 = packet_rcv.seqn12;
												packet->payload22 = packet_rcv.payload12;
												packet->seqn23 = packet_rcv.seqn13;
												packet->payload23 = packet_rcv.payload13;
												packet->seqn24 = packet_rcv.seqn14;
												packet->payload24 = packet_rcv.payload14;
												packet->seqn25 = packet_rcv.seqn15;
												packet->payload25 = packet_rcv.payload15;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 1 to Pkt 2");
											}
											++packet->size;
										}
										else if(packet->size == 2){
											if(packet_rcv.size == 2){
												packet->nodeid3 = packet_rcv.nodeid2;
												packet->seqn31 = packet_rcv.seqn21;
												packet->payload31 = packet_rcv.payload21;
												packet->seqn32 = packet_rcv.seqn22;
												packet->payload32 = packet_rcv.payload22;
												packet->seqn33 = packet_rcv.seqn23;
												packet->payload33 = packet_rcv.payload23;
												packet->seqn34 = packet_rcv.seqn24;
												packet->payload34 = packet_rcv.payload24;
												packet->seqn35 = packet_rcv.seqn25;
												packet->payload35 = packet_rcv.payload25;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 2 to Pkt 3");
											}
											else if(packet_rcv.size == 1){
												packet->nodeid3 = packet_rcv.nodeid1;
												packet->seqn31 = packet_rcv.seqn11;
												packet->payload31 = packet_rcv.payload11;
												packet->seqn32 = packet_rcv.seqn12;
												packet->payload32 = packet_rcv.payload12;
												packet->seqn33 = packet_rcv.seqn13;
												packet->payload33 = packet_rcv.payload13;
												packet->seqn34 = packet_rcv.seqn14;
												packet->payload34 = packet_rcv.payload14;
												packet->seqn35 = packet_rcv.seqn15;
												packet->payload35 = packet_rcv.payload15;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 1 to Pkt 3");
											}
											++packet->size;
										}
										else if(packet->size == 3){
										
											if(packet_rcv.size == 1){
												packet->nodeid4 = packet_rcv.nodeid1;
												packet->seqn41 = packet_rcv.seqn11;
												packet->payload41 = packet_rcv.payload11;
												packet->seqn42 = packet_rcv.seqn12;
												packet->payload42 = packet_rcv.payload12;
												packet->seqn43 = packet_rcv.seqn13;
												packet->payload43 = packet_rcv.payload13;
												packet->seqn44 = packet_rcv.seqn14;
												packet->payload44 = packet_rcv.payload14;
												packet->seqn45 = packet_rcv.seqn15;
												packet->payload45 = packet_rcv.payload15;
												packet_rcv.size--;
												LOG_INFO("copied Pkt 1 to Pkt 4");
											}
											++packet->size;
										}
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
