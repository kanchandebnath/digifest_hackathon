/*
 * Copyright (c) 2016, Zolertia <http://www.zolertia.com>
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
/**
 * \addtogroup zoul-examples
 * @{
 *
 * \defgroup zoul-ac-dimmer-test Krida Electronics AC light dimmer test example
 *
 * Demonstrates the use of an AC dimmer with zero-crossing, connected to the
 * ADC1 and ADC2 pins (PA5 and PA4 respectively), powered over the D+5.1 pin
 *
 * @{
 *
 * \file
 *         A quick program to test an AC dimmer
 * \author
 *         Antonio Lignan <alinan@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "dev/eeprom_cc2538.h"
#include "dev/ac-dimmer.h"
#include "lib/sensors.h"
#include "smart_street_light.h"
/*---------------------------------------------------------------------------*/
PROCESS(ac_dimmer_process, "AC light dimmer");
/*---------------------------------------------------------------------------*/
static uint8_t dimming;
static struct etimer et;
extern uint8_t ssl_brightness_value;
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(ac_dimmer_process, ev, data)
{
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(ac_dimmer);

 dimming =5;

  etimer_set(&et, CLOCK_SECOND/2);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  while(1) {


    if(ssl_brightness_value<10)
      dimming=ssl_brightness_value;    
	  
    if(ssl_brightness_value > dimming && ssl_brightness_value>=10)
  {
    dimming += 1;//DIMMER_DEFAULT_MIN_DIMM_VALUE;
    if(dimming > DIMMER_DEFAULT_MAX_DIMM_VALUE) {
      dimming = ssl_brightness_value;
    }

   }
   else if( dimming > ssl_brightness_value )
   {
           dimming -= 1;//DIMMER_DEFAULT_MIN_DIMM_VALUE;
		   if(dimming < 10)
		     dimming = ssl_brightness_value;
   }
   else
   {
   }
   // printf("Dimming Vlaue %u\n", dimming);
    ac_dimmer.value(dimming);
    etimer_set(&et, CLOCK_SECOND/200);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}





/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

