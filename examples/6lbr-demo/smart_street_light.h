

/*
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-test-pwm Test the CC2538 PWM driver
 *
 * Demonstrates the use of the CC2538 PWM driver
 *
 * @{
 *
 * \file
 *         A quick program for testing the CC2538 PWM driver
 * \author
 *         Javier Sanchez <jsanchez@zolertia.com>
 *         Antonio Lignan <alinan@zolertia.com>
 */

#ifndef SMART_STREET_LIGHT_H_
#define SMART_STREET_LIGHT_H__

#include "contiki.h"
#include "cpu.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "dev/sys-ctrl.h"
#include "pwm.h"

#include "lpm.h"
#include "dev/ioc.h"
#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t timer;
  uint8_t ab;
  uint8_t port;
  uint8_t pin;
  uint8_t duty;
  uint8_t off_state;
  uint32_t freq;
} pwm_config_t;
/*---------------------------------------------------------------------------*/

#define NORMAL                  1
#define ABNORMAL                0
#define MODE_AUTO               1
#define MODE_MANUAL             0

#define OFF                     0
#define ON                      1

#define PRESENCE_DETECTED       1
#define PRESENCE_NOT_DETECTED   0 

#define ALS_DATA_DAWN_MAX       10
#define AMBIENT_LIGHT_CONDITION_OK      15

#define MINIMUM_ON_CURRENT          1000
#define MAXIMUM_ON_CURRENT          2000

#define CURRENT_SENSOR_RES_MUL 0.0264
#define CURRENT_SENSOR_RES_OFFSET 13.51

#define STATUS_SUCCESS               0
#define STATUS_FAILED              (-1)
#define STATUS_OK                       1





typedef union
{
 uint32_t   all_flags;      // Allows us to refer to the flags 'en masse
 struct
 {
  uint32_t brightness_value     : 7,
           operating_mode       : 1,
           pwr_on_command       : 1,
           zone_id		: 5,  
           last_pwr_state       : 1,   // last power State-1=ON and 0=OFF
           last_pwr_dwn_mode    : 1,   // Last Power Down Mode- 1=Systemetic Shutdown 0=Abnormal Power off- like Power Cut 
           overload_fail        : 1,   // Light Failed due to 1=Over Load  0= Light OK,Over_load 
           underload_fail       : 1,   // Light failed 1= Failed  0=Light OK, Used to Indicate that Bulb might be fused 
           light_is_on          : 1,   // Current State of the Light 1- ON ,  0-OFF 
           presence_detect      : 1,   // this falg is used to deternime when light shoots to 100%
           application_id       : 4,
           beacon_status        : 1,
           temp_humidity        : 1,
           presence_detect_timer_start  : 1,
           pole_tilted          : 1,
           siren_status         : 1,
           dummy_bit_30         : 1,
           dummy_bit_31         : 1,
           node_reset           : 1;
};
}street_light_states;


typedef union
{
 uint32_t   all_flags;      // Allows us to refer to the flags 'en masse
 struct
 {
  uint32_t brightness_value     : 7,   // last power State-1=ON and 0=OFF
           operating_mode       : 1,   // Street Light Operating Mode- 1=Auto 0= Manual Mode  
           pwr_on_command       : 1,   // 1-Power_ON from Manual Mode, 0= Manual Mode Power Off
           broadcast_data       : 1,   // Bit free for future assignment 
           zone_id              : 5,   // Bit free for future assignment 
           reset_node           : 1,   // Bit free for future assignment 
           beacon_on            : 1,
           siren_on             : 1,
           node_id              : 12,
           dummy_2_bits         : 2;
            

 };
} street_light_custom_cmd;


typedef struct 
{
uint16_t node_id;
uint16_t pan_id;
uint8_t zone_id;
uint8_t dummy_data;
char device_name[24];
}node_edit;


typedef union
{
 uint32_t   all_flags;      // Allows us to refer to the flags 'en masse
 struct
 {
  uint32_t brightness_value     : 7,   // last power State-1=ON and 0=OFF
           operating_mode       : 1,   // Street Light Operating Mode- 1=Auto 0= Manual Mode  
           pwr_on_command       : 1,   // 1-Power_ON from Manual Mode, 0= Manual Mode Power Off
           broadcast_data       : 1,   // Bit free for future assignment 
           zone_id              : 5,   // Bit free for future assignment 
           reset_node           : 1,   // Bit free for future assignment 
           beacon_on            : 1,   // Bit free for future assignment 
           siren_on             : 1,   // Bit free for future assignment 
           node_id              : 12,
           dummy_9_bits         : 2;   // Bit free for future assignment 

 };
} data_pack_t;



typedef union
{
 uint32_t   all_flags;      // Allows us to refer to the flags 'en masse
 struct
 {
  uint32_t zone_id              : 5,   // last power State-1=ON and 0=OFF
           sensor_value         : 7,   // Street Light Operating Mode- 1=Auto 0= Manual Mode  
           pwr_on_command       : 1,   // 1-Power_ON from Manual Mode, 0= Manual Mode Power Off
           broadcast_data       : 1,   // Bit free for future assignment 
           presence_detect      : 1,
           reset_node           : 1,   // Bit free for future assignment 
           dummy_bits16         : 16;


 };
} data_pack_unicast_t;



uint8_t ssl_brightness_value;
extern process_event_t  presence_detect_event;
extern process_event_t  default_dutycycle_event;
uint8_t light_power_on(uint8_t duty_cycle);
uint8_t light_power_off(void);
uint8_t incr_brightness(uint8_t duty_cycle);
uint8_t decr_brightness(uint8_t duty_cycle);
uint16_t read_ambient_light(uint8_t als_ch);
uint8_t read_presence_sensor(void);
uint8_t read_light_fixure_status(uint8_t adc_ch);
uint8_t adjust_brightness(uint8_t present_duty_cycle, uint8_t target_duty_cycle);
uint8_t  wrapper_pwm_enable(uint8_t);
uint8_t  wrapper_pwm_disable(void);
uint8_t  wrapper_pwm_start(void);
uint8_t  wrapper_pwm_stop(void);
uint8_t  beacon_pwm_enable(uint8_t);
uint8_t  beacon_pwm_disable(void);
uint8_t  beacon_pwm_start(void);
uint8_t  beacon_pwm_stop(void);
uint8_t  beacon_light_on(uint16_t frequency, uint8_t dutycycle);
uint8_t  beacon_light_off(void);
uint8_t  wrapper_pwm_toggle_direction(void);
void eeprom_reset_condition(void);


#define STREET_LIGHT_CURRENT_SENS 0
#define MAX_PWM 4


#define TIMER_1A_PWM    0
#define TIMER_1B_PWM	1
#define TIMER_2A_PWM	2
#define TIMER_2B_PWM	3

static const pwm_config_t pwm_num[MAX_PWM] = {
  {
    .timer = PWM_TIMER_1,
    .ab = PWM_TIMER_A,
    .port = GPIO_C_NUM,
    .pin = 6,
    .duty = 100,
    .freq = 16000,
    .off_state = PWM_OFF_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_1,
    .ab = PWM_TIMER_B,
    .port = GPIO_C_NUM,
    .pin = 7,
    .duty = 50,
    .freq = 30,
    .off_state = PWM_ON_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_2,
    .ab = PWM_TIMER_A,
    .port = GPIO_C_NUM,
    .pin = 1,
    .duty = 50,
    .freq = 160000,
    .off_state = PWM_OFF_WHEN_STOP,
  }, {
    .timer = PWM_TIMER_2,
    .ab = PWM_TIMER_B,
    .port = GPIO_C_NUM,
    .pin = 7,
    .duty = 80,
    .freq = 80000,
    .off_state = PWM_ON_WHEN_STOP,
  }
};

#endif
