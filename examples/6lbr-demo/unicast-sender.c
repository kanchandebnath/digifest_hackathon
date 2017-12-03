/*Copyright (c) 2011, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"
#include "sys/node-id.h"
#include "simple-udp.h"
#include "dev/eeprom_cc2538.h"

#include "smart_street_light.h"
#include "uip-ds6-nbr.h"
#include "nbr-table.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define UDP_PORT 1177
#define DEBUG 1

static struct simple_udp_connection unicast_connection;
extern street_light_states street_light_status;
extern data_pack_unicast_t data_pack_unicast;
/*---------------------------------------------------------------------------*/
PROCESS(unicast_sender_process, "Unicast sender process");
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{

  data_pack_unicast.all_flags=*(uint32_t*)data;
  street_light_status.presence_detect=data_pack_unicast.presence_detect;
  street_light_status.presence_detect_timer_start=1;
  eeprom_street_light_status=street_light_status.all_flags;
  eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS,eeprom_street_light_status,5); 
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_sender_process, ev, data)
{
  

  PROCESS_BEGIN();
  simple_udp_register(&unicast_connection, UDP_PORT, NULL, UDP_PORT, receiver);
  while(1) {

	  PROCESS_WAIT_EVENT_UNTIL(ev==presence_detect_event);

	  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
	  while (nbr != NULL)
	  {

		//  uip_debug_ipaddr_print(&nbr->ipaddr);
		  simple_udp_sendto(&unicast_connection, data, sizeof(data)+1, &nbr->ipaddr);
		  nbr = nbr_table_next(ds6_neighbors, nbr);
		  clock_delay_usec(50);
	  }
      }
  PROCESS_END();
}
/*-----`------------------------------------:----------------------------------*/

