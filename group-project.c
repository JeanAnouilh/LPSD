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
/**
 * @brief     Struct to store packet information
 */
typedef struct {
  uint8_t         hop_count;
  uint8_t         round_count;
  uint16_t        data;
  uint16_t        payload;
  uint16_t        payload2;
  uint16_t        payload3;
  uint16_t        payload4;
  uint16_t        payload5;
  uint16_t        payload6;
  uint16_t        payload7;
  uint16_t        payload8;
  uint16_t        payload9;
  uint16_t        payload10;
  uint16_t        payload11;
  uint16_t        payload12;
  uint16_t        payload13;
} lpsd_payload_t;
typedef struct {
  uint16_t        src_id;
  uint8_t         seqn;
  lpsd_payload_t  payload;
} lpsd_packet_t;
typedef struct {
  uint8_t         synccount;
} lpsd_syncpacket_t;
/*---------------------------------------------------------------------------*/
PROCESS(design_project_process, "Skeleton code - LPSD Design Project");
AUTOSTART_PROCESSES(&design_project_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(design_project_process, ev, data)
{
  //static lpsd_packet_t  packet;     /* packet buffer */
  //static lpsd_packet_t* packet2;    /* packet pointer */
  //static lpsd_packet_t  packet_rcv; /* received packet buffer */

  //Syncronization Packet
  static lpsd_syncpacket_t  sync_packet;            /* packet buffer */
  static lpsd_syncpacket_t  sync_packet_rcv;        /* received packet buffer */

  static uint8_t        packet_len; /* packet length, in Bytes */
  static uint16_t       timeout_ms; /* packet receive timeout, in ms */
  static uint8_t	firstpacket = 1; // First packet for the initiator
  static struct etimer  sync_timer;
  static struct etimer  wait_timer;
  static struct etimer  first_wait_timer;
  //static struct etimer  slot_timer;
  //static struct etimer  periodtimer;
  static uint8_t	sync = 10;  /*mindestens 3 Runden*/
  //static uint8_t	synccount;
  //static uint8_t	syncdist;
  static uint8_t	last_sync = 0;
  static uint8_t	first_sync = 0;
  static uint8_t	test_count = 0;
  static clock_time_t	last_time = 0;
  static clock_time_t	first_time = 0;
  static clock_time_t	timestamp;


  //static std::vector<uint8_t> my_parents;  					/* parent slot IDs */
  //static uint8_t        			my_dst;     			/* packet destination ID */
  //static uint8_t				my_slot;			/* used slot ID */

  PROCESS_BEGIN();

  /* initialize the data generator */
  data_generation_init();

  /* configure GPIO as outputs */
  //PIN_CFG_OUT(RADIO_START_PIN);
  PIN_CFG_OUT(RADIO_RX_PIN);
  PIN_CFG_OUT(RADIO_TX_PIN);
  PIN_CFG_OUT(LED_STATUS);

  /* Setup periodic timers that expire after 10/50/1000 milli-seconds. */
  etimer_set(&first_wait_timer, CLOCK_SECOND / 20);
  etimer_set(&wait_timer, CLOCK_SECOND / 100);
  etimer_set(&sync_timer, CLOCK_SECOND);
  timeout_ms = 100;


	if(sinkaddress == 22) {
		/*if(node_id == 1) {
			my_dst = 33;
			my_slot = 14;
		} else if(node_id == 2) {
			my_dst = 33;
			my_slot = 19;
		} else if(node_id == 3) {
		//	my_parents.push_back(10);
		//	my_parents.push_back(15);
			my_dst = 22;
			my_slot = 9;
		} else if(node_id == 4) {
			my_dst = 33;
			my_slot = 21;
		} else if(node_id == 6) {
			my_dst = 22;
			my_slot = 22;
		} else if(node_id == 8) {
			my_dst = 28;
			my_slot = 17;
		} else if(node_id == 10) {
			my_dst = 3;
			my_slot = 20;
		} else if(node_id == 15) {
			my_dst = 3;
			my_slot = 23;
		} else if(node_id == 16) {
			my_dst = 22;
			my_slot = 16;
		} else if(node_id == 18) {
			my_dst = 22;
			my_slot = 18;
		} else if(node_id == 22) {
			my_slot = 0;
		} else if(node_id == 28) {
		//	my_parents.push_back(8);
		//	my_parents.push_back(31);
			my_dst = 22;
			my_slot = 1;
		} else if(node_id == 31) {
		//	my_parents.push_back(32);
			my_dst = 28;
			my_slot = 12;
		} else if(node_id == 32) {
			my_dst = 31;
			my_slot = 15;
		} else if(node_id == 33) {
		//	my_parents.push_back(1);
		//	my_parents.push_back(2);
		//	my_parents.push_back(4);
			my_dst = 22;
			my_slot = 5;
		}*/
		etimer_restart(&sync_timer);

		while(sync) {
			if(firstpacket && node_id == sinkaddress) {
				/* --- INITIATOR --- */
				/* Wait 50 ms to be sure that all other nodes are ready. */
				etimer_restart(&first_wait_timer);
  				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&first_wait_timer));

  				/* prepare first packet */
				firstpacket = 0;
				sync_packet.synccount = 0;
				packet_len = sizeof(sync_packet);
				--sync;

				/* get Timestamp and send first packet */
				timestamp = clock_time();
				radio_send(((uint8_t*)&sync_packet),packet_len,1);
				LOG_INFO("sync_round: %u\n", sync_packet.synccount);
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

				LOG_INFO("receive_packet_round: %u\n",sync_packet_rcv.synccount);

				if(timestamp) {
					if(!first_sync) {
						first_sync = sync_packet.synccount;
						first_time = timestamp;
					} else {
						last_sync = sync_packet.synccount;
						last_time = timestamp;
					}
				}

				/* increment counter and resend packet */
				sync_packet = sync_packet_rcv;
				++sync_packet.synccount;	
				packet_len = sizeof(sync_packet);
				--sync;

				/* Wait for send */
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&wait_timer));

				/* get Timestamp and send packet */
				timestamp = clock_time();
				radio_send(((uint8_t*)&sync_packet),packet_len,1);
				LOG_INFO("send_packet_round: %u\n",sync_packet.synccount);
			}
		}

		clock_time_t delta_t = (last_time - first_time) / (last_sync - first_sync);
		clock_time_t t_zero = first_time - (first_sync * delta_t);

		etimer_adjust(&sync_timer, (int16_t) (t_zero - etimer_start_time(&sync_timer)));



		while(1) {
  			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sync_timer));
			LOG_INFO("Round: %u\n",test_count);
			++test_count;
			etimer_reset(&sync_timer);
		}
	}
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
