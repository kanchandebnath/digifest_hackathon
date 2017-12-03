/*
 * Copyright (c) 2015, CETIC.
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
 */

/**
 * \file
 *         6LBR-Demo Project Configuration
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#ifndef SIXLN_CONF_CONTIKI_H
#define SIXLN_CONF_CONTIKI_H

#undef LWM2M_HAS_POWER_INFO
#define LWM2M_HAS_POWER_INFO "contiki-power-info.h"

#define USER_CONF_NVM_DATA 0


#ifndef CUSTOM_COAP_RESOURCES

/* Disable .well-known/core filtering to save code */
#define COAP_LINK_FORMAT_FILTERING 0

/* Disable CoAP Binding */
#define COAP_BINDING_CONF_ENABLED 0 //0

/* Disable RD Client */
#define RD_CLIENT_CONF_ENABLED 1  // 0

#if UIP_CONF_IPV6_RPL

/* Disable observe */
#define COAP_CORE_OBSERVE 1

/* Disable coap push */
#define COAP_PUSH_CONF_ENABLED  1 //0

/* Disable core interface binding table */
#define REST_CONF_RES_BINDING_TABLE 0

/* Disable core interface binding nvm */
#define CORE_ITF_CONF_BINDING_TABLE_NVM 0


/* Disable core interface linked batch nvm */
#define CORE_ITF_CONF_LINKED_BATCH_NVM 0

#endif /* UIP_CONF_IPV6_RPL */

#if UIP_CONF_IPV6_RPL

#define REST_CONF_RES_BATTERY 1
#define REST_CONF_RES_BATTERY_PERIODIC 1
#define REST_CONF_RES_BUTTON_LEFT 1
#define REST_CONF_RES_BUTTON_RIGHT 1
#define REST_CONF_RES_BUTTON_UP 1
#define REST_CONF_RES_BUTTON_DOWN 1
#define REST_CONF_RES_BUTTON_SELECT 1
#define REST_CONF_RES_BUTTON_LEFT_EVENT 1
#define REST_CONF_RES_BUTTON_RIGHT_EVENT 1
#define REST_CONF_RES_BUTTON_UP_EVENT 1
#define REST_CONF_RES_BUTTON_DOWN_EVENT 1
#define REST_CONF_RES_BUTTON_SELECT_EVENT 1
#define REST_CONF_RES_DEVICE_BATCH 1
#define REST_CONF_RES_DEVICE_MANUFACTURER 1
#define REST_CONF_RES_DEVICE_MODEL 1
#define REST_CONF_RES_DEVICE_MODEL_HW 1
#define REST_CONF_RES_DEVICE_MODEL_SW 1
#define REST_CONF_RES_DEVICE_SERIAL 1
#define REST_CONF_RES_DEVICE_NAME 1
#define REST_CONF_RES_DEVICE_STATUS 1
#define REST_CONF_RES_DEVICE_STATUS_PERIODIC 1
#define REST_CONF_RES_NODE_ID 1





#define REST_CONF_RES_DEVICE_TIME 1
#define REST_CONF_RES_DEVICE_UPTIME 1
#define REST_CONF_RES_SENSORS_BATCH 1
#define REST_CONF_RES_LED_RED 0
#define REST_CONF_RES_LED_GREEN 0
#define REST_CONF_RES_LED_BLUE 0
#define REST_CONF_RES_TEMP 1
#define REST_CONF_RES_TEMP_PERIODIC 1
#define REST_CONF_RES_RADIO_LQI 1
#define REST_CONF_RES_RADIO_LQI_PERIODIC 1
#define REST_CONF_RES_RADIO_RSSI 1
#define REST_CONF_RES_RADIO_RSSI_PERIODIC 1


#define REST_CONF_RES_AMBIENT_TEMPERATURE_SENSOR 1
#define REST_CONF_RES_AMBIENT_TEMPERATURE_SENSOR_PERIODIC 1
#define REST_CONF_RES_AMBIENT_TEMPERATURE_SENSOR_PERIOD 30
#define REST_CONF_RES_HUMIDITY_SENSOR 1
#define REST_CONF_RES_HUMIDITY_SENSOR_PERIODIC 1
#define REST_CONF_RES_HUMIDITY_SENSOR_PERIOD 30


#define REST_CONF_RES_CONFIG_STACK_PHY 0
#define REST_CONF_RES_CONFIG_STACK_MAC 0
#define REST_CONF_RES_CONFIG_STACK_NET 0
#define REST_CONF_RES_CONFIG_STACK_RTG 0




/* Disable core interface linked batch table */
#define REST_CONF_RES_LINKED_BATCH_TABLE 0

#define LWM2M_CONF_DEVICE_OBJECT 1 //0

#else /* UIP_CONF_IPV6_RPL */

/* Disable config stack */
#define REST_CONF_RES_CONFIG_STACK_PHY 0
#define REST_CONF_RES_CONFIG_STACK_MAC 0
#define REST_CONF_RES_CONFIG_STACK_NET 0
#define REST_CONF_RES_CONFIG_STACK_RTG 0

/* disable resources batch */
#define REST_CONF_RES_DEVICE_BATCH 1  //0

#define REST_CONF_RES_SENSORS_BATCH 1 //0

/* Disable core interface linked batch table */

#endif /* UIP_CONF_IPV6_RPL */

#endif /* CUSTOM_COAP_RESOURCES */

/* NVM is too limited on Sky */
#define REST_CONF_MAX_DEVICE_NAME_LENGTH 16
#define COAP_PUSH_CONF_MAX_URI_SIZE 16
#define CORE_ITF_CONF_USER_BINDING_NB 1
#endif
