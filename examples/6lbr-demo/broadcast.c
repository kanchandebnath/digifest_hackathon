/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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

#include "simple-udp.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smart_street_light.h"
#include "dev/eeprom_cc2538.h"
#define DEBUG 1

#define UDP_PORT 1155
#define SEND_INTERVAL           (5*CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection broadcast_connection;
extern street_light_states street_light_status;
extern street_light_custom_cmd custom_command;
extern data_pack_t data_pack;
/*---------------------------------------------------------------------------*/
PROCESS(broadcast_process, "UDP broadcast process");
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
  data_pack.all_flags=*(uint32_t*)data;  

  eeprom_read_data(EEPROM_ZONE_ID, &eeprom_zone_id, 2); 
  eeprom_read_data_big(EEPROM_CUSTOM_COMMAND,&eeprom_custom_command,4);
   
if((data_pack.zone_id==0) || (data_pack.zone_id==eeprom_zone_id))
{
 
  if(data_pack.operating_mode==MODE_AUTO)
   {
   eeprom_auto_default_brightness_value=data_pack.brightness_value;
   custom_command.brightness_value=data_pack.brightness_value;
   eeprom_write_data(EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE,eeprom_auto_default_brightness_value,3);

   }
  else if(data_pack.operating_mode==MODE_MANUAL)
  {

   eeprom_manual_custom_brightness_value=data_pack.brightness_value;
   custom_command.brightness_value=data_pack.brightness_value;
   eeprom_write_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE,eeprom_manual_custom_brightness_value,3);
   custom_command.pwr_on_command=data_pack.pwr_on_command;

  }

   custom_command.reset_node=data_pack.reset_node;
   custom_command.operating_mode=data_pack.operating_mode;
   custom_command.all_flags=data_pack.all_flags;
   eeprom_custom_command=custom_command.all_flags;
   eeprom_write_data_big(EEPROM_CUSTOM_COMMAND,eeprom_custom_command,5);

  }

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
//  static struct etimer;
  uip_ipaddr_t addr;

  PROCESS_BEGIN();

  simple_udp_register(&broadcast_connection, UDP_PORT, NULL, UDP_PORT, receiver);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev==default_dutycycle_event);
    uip_create_linklocal_allnodes_mcast(&addr);
    simple_udp_sendto(&broadcast_connection, data, sizeof(data)+1, &addr);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
