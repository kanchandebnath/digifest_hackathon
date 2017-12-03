/*
 * Copyright (c) 2015, Zolertia <http://www.zolertia.com>
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
 * \file
 *         A  program for reading Texas Instruments Temperature and humidity sensor HDC-1080
 * \author
 *         Antonio Lignan <alinan@zolertia.com>
 */
#include <stdio.h>
#include "contiki.h"
#include "dev/hdc1080.h"
#include "dev/eeprom_cc2538.h"
#include "lib/random.h"
process_event_t temp_humidity_event;
PROCESS_NAME(http_get_process);
PROCESS(temp_humidity_measurement_process, "HDC1080_Sensor");
#define SEND_INTERVAL  180*CLOCK_SECOND
#define SEND_TIME (random_rand() %(SEND_INTERVAL))


static struct etimer et;
double temperature, humidity;


PROCESS_THREAD(temp_humidity_measurement_process, ev, data)
{

  PROCESS_BEGIN();
  temp_humidity_event=process_alloc_event();
  etimer_set(&et, CLOCK_SECOND*3);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));


  while(1) {
    uint8_t battery_status;
    etimer_set(&et, CLOCK_SECOND*5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    hdc1080_measure(HDC1080_T_RES_11, HDC1080_RH_RES_11, HDC1080_HEATER_ON, &battery_status, &temperature, &humidity);
    temperature=temperature*100;
    humidity=humidity*100;   
//    printf("Temperature and Humidity %u, %u\n", (uint16_t)temperature, (uint16_t)humidity);
    etimer_set(&et, SEND_TIME);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  //  printf("Post Temperature and Humidity Data\n");
    process_post(&http_get_process, temp_humidity_event, NULL);
       }
  PROCESS_END();
}
