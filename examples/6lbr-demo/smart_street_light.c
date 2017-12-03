/*
Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
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
 * \addtogroup cc2538-platforms
 * @{
 *
 * \defgroup cc2538-examples cc2538 Example Projects
 * @{
 *
 * \defgroup cc2538-demo cc2538dk Demo Project
 *
 *   Example project demonstrating the cc2538dk functionality
 *
 *   This assumes that you are using a SmartRF06EB with a cc2538 EM
 *
 * - Boot sequence: LEDs flashing, LED2 followed by LED3 then LED4
 * - etimer/clock : Every LOOP_INTERVAL clock ticks the LED defined as
 *                  LEDS_PERIODIC will turn on
 * - rtimer       : Exactly LEDS_OFF_HYSTERISIS rtimer ticks later,
 *                  LEDS_PERIODIC will turn back off
 * - Buttons      :
 *                - BTN_DOWN turns on LEDS_REBOOT and causes a watchdog reboot
 *                - BTN_UP to soft reset (SYS_CTRL_PWRDBG::FORCE_WARM_RESET)
 *                - BTN_LEFT and BTN_RIGHT flash the LED defined as LEDS_BUTTON
 * - ADC sensors  : On-chip VDD / 3 and temperature, and ambient light sensor
 *                  values are printed over UART periodically.
 * - UART         : Every LOOP_INTERVAL the EM will print something over the
 *                  UART. Receiving an entire line of text over UART (ending
 *                  in \\r) will cause LEDS_SERIAL_IN to toggle
 * - Radio comms  : BTN_SELECT sends a rime broadcast. Reception of a rime
 *                  packet will toggle LEDs defined as LEDS_RF_RX
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc2538dk platform
 */
 
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "uip-debug.h"
#include "sys/timer.h"
#include "dev/leds.h"
#include "dev/pwm.h"
#include "dev/uart.h"
#include "dev/eeprom_cc2538.h"
#include "dev/cc2538-sensors.h"
#include "dev/button-sensor.h"
#include "dev/als-sensor.h"
#include "dev/current-sensor.h"
#include "dev/watchdog.h"
#include "dev/sys-ctrl.h"
#include "smart_street_light.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "uip-ds6-nbr.h"
#include "nbr-table.h"
#include "coap-common.h"
#include "core-interface.h"
#include "device-resource.h"
#include "device-info-resource.h"

//====================================

#define DEBUG 1
uint8_t brightness_value;
extern resource_t resource_street_light_status;
static struct etimer presence_detected_timer;
static struct etimer et;
static uint8_t pwm_en[MAX_PWM];
static uint32_t broadcast_data=0;
static uint32_t unicast_data=0;
volatile uint16_t als_lux_value; 
volatile uint16_t current_drawn; 


street_light_states street_light_status;
street_light_custom_cmd custom_command;
node_edit custom_node_edit;
data_pack_t data_pack;
data_pack_unicast_t data_pack_unicast;
process_event_t  presence_detect_event;
process_event_t  default_dutycycle_event;
process_event_t  status_change_event;
PROCESS_NAME(unicast_sender_process);
PROCESS_NAME(broadcast_process);
PROCESS_NAME(http_get_process);


void eeprom_reset_condition(void)
{
street_light_status.all_flags=0x2100092;
eeprom_street_light_status=street_light_status.all_flags;
eeprom_checksum=0x8777;
eeprom_pan_id=0xABCD;
eeprom_zone_id=0;
eeprom_channel=0x1A;
eeprom_custom_command=0x0092;
eeprom_node_id=0;
eeprom_auto_default_brightness_value=40;
eeprom_manual_custom_brightness_value=40;
eeprom_brightness_value=40;
char eeprom_device_name[]="Device-1";
char eeprom_node_latitude[]="000.000";
char eeprom_node_longitude[]="000.000";
char eeprom_mac_addr[]="aa:bb:cc:dd:ee:ff:gg:hh";
eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
eeprom_write_data(EEPROM_CHECKSUM, eeprom_checksum, 3);
eeprom_write_data(EEPROM_PAN_ID, eeprom_pan_id, 3);
eeprom_write_data(EEPROM_ZONE_ID, eeprom_zone_id, 3);
eeprom_write_data(EEPROM_CHANNEL, eeprom_channel, 3);
eeprom_write_data(EEPROM_NODE_ID, eeprom_node_id, 3);
eeprom_write_data(EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE, eeprom_auto_default_brightness_value, 3);
eeprom_write_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE, eeprom_manual_custom_brightness_value, 3);
eeprom_write_data(EEPROM_BRIGHTNESS_VALUE, eeprom_brightness_value, 3);
eeprom_write_data_big(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 5);
eeprom_write_string(EEPROM_DEVICE_NAME, eeprom_device_name, NAME_LENGTH);
eeprom_write_string(EEPROM_LATITUDE, eeprom_node_latitude, VALUE_LENGTH);
eeprom_write_string(EEPROM_LONGITUDE, eeprom_node_longitude, VALUE_LENGTH);
eeprom_write_string(EEPROM_MAC_ADDR, eeprom_mac_addr, MAC_ADDR_LENGTH);

}


void initial_eeprom_condition(void)
{

eeprom_read_data_big(EEPROM_STREET_LIGHT_STATUS, &eeprom_street_light_status, 4);
eeprom_read_data(EEPROM_CHECKSUM, &eeprom_checksum,2);
eeprom_read_data(EEPROM_PAN_ID, &eeprom_pan_id, 2);
eeprom_read_data(EEPROM_ZONE_ID, &eeprom_zone_id, 2);
eeprom_read_data(EEPROM_CHANNEL, &eeprom_channel, 2);
eeprom_read_data(EEPROM_NODE_ID, &eeprom_node_id, 2);
eeprom_read_data(EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE, &eeprom_auto_default_brightness_value, 2);
eeprom_read_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE, &eeprom_manual_custom_brightness_value, 2);
eeprom_read_data_big(EEPROM_CUSTOM_COMMAND, &eeprom_custom_command, 4); 
eeprom_read_string(EEPROM_DEVICE_NAME, eeprom_device_name, NAME_LENGTH);
eeprom_read_string(EEPROM_LATITUDE, eeprom_node_latitude, VALUE_LENGTH);
eeprom_read_string(EEPROM_LONGITUDE, eeprom_node_longitude, VALUE_LENGTH);



}


#define ADC_TO_LUX_LOOKUP_TABLE_ROWS 47
/** Lookup table used if LIGHT_SENSOR_USE_LOOKUP_TABLE is defined in lieu of calculating the value with math. 
First value is ADC reading, second is lux/10 */
const int16_t adcToLuxLookupTable[ADC_TO_LUX_LOOKUP_TABLE_ROWS][2] = { 
    {5,1},     {10,2},     {15,3},    {20,4},    {25,5},    {30,6},    {35,7},     {40,8},     {45,9},
    {50,10},    {55,20},    {60,30},   {65,40},   {70,50},   {535,60},   {548,70},    {559,80},    {569,90},
    {578,100},   {635,200},   {669,300},  {693,400},  {712,500},  {727,600},  {740,700},   {751,800},   {761,900},
    {770,1000},  {828,2000},  {862,3000}, {886,4000}, {905,5000}, {920,6000}, {933,7000},  {944,8000},  {954,9000},
    {963,10000}, {970,11000}, {978,12000},{985,13000},{991,14000},{997,15000},{1002,16000},{1007,17000},{1012,18000},{1016,19000},{1020,20000}}; 

	
int16_t  convertAdcToLux(int16_t adc)
{
    unsigned char iterator = 0;
    while (adc > adcToLuxLookupTable[iterator][0])
        iterator++;
    if (iterator == ADC_TO_LUX_LOOKUP_TABLE_ROWS)
        return 0xFFFF;  //ADC value not found in table; error. 

    return (adcToLuxLookupTable[iterator][1]);

}

/*
void get_neighbours(void)
{
     int num= uip_ds6_nbr_num();
     //printf("Numer of Neighbours is %d\n", num);

      uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);

      while(nbr !=NULL)
	{
      	PRINT6ADDR(&nbr->ipaddr);
      	nbr = nbr_table_next(ds6_neighbors, nbr);
	}

}
*/

uint16_t read_ambient_light(uint8_t als_ch)
{
als_lux_value=convertAdcToLux(als_sensor.value(1));
printf("Converted ADC Vaue is converted %u\n", als_lux_value);	
return als_lux_value;
}


uint8_t  wrapper_pwm_enable(uint8_t duty_cycle)
{
	if(pwm_enable(pwm_num[TIMER_1A_PWM].freq, duty_cycle,
                  pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab) == PWM_SUCCESS) {
      pwm_en[TIMER_1A_PWM] = 1;
      PRINTF("%s (%u) configuration OK\n", gpt_name(pwm_num[TIMER_1A_PWM].timer),
             pwm_num[TIMER_1A_PWM].ab);
    		return STATUS_FAILED;
	} 
	return STATUS_SUCCESS;
}


uint8_t  beacon_pwm_enable(uint8_t duty_cycle)
{
	if(pwm_enable(pwm_num[TIMER_1B_PWM].freq, duty_cycle,
                  pwm_num[TIMER_1B_PWM].timer, pwm_num[TIMER_1B_PWM].ab) == PWM_SUCCESS) {
      pwm_en[TIMER_1B_PWM] = 1;
      PRINTF("%s (%u) configuration OK\n", gpt_name(pwm_num[TIMER_1B_PWM].timer),
             pwm_num[TIMER_1B_PWM].ab);
    		return STATUS_FAILED;
	} 
	return STATUS_SUCCESS;
}



uint8_t  wrapper_pwm_disable(void)
{
	 if((pwm_en[TIMER_1A_PWM]) &&
         (pwm_disable(pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab,
                   pwm_num[TIMER_1A_PWM].port, pwm_num[TIMER_1A_PWM].pin) != PWM_SUCCESS))
        	{
		pwm_en[TIMER_1A_PWM] = 0;
		PRINTF("%s (%u) failed to stop\n", gpt_name(pwm_num[TIMER_1A_PWM].timer),
		pwm_num[TIMER_1A_PWM].ab);
		return STATUS_FAILED;
		}
	       return STATUS_SUCCESS;
	
}


uint8_t  beacon_pwm_disable(void)
{
	 if((pwm_en[TIMER_1B_PWM]) &&
         (pwm_disable(pwm_num[TIMER_1B_PWM].timer, pwm_num[TIMER_1B_PWM].ab,
                   pwm_num[TIMER_1B_PWM].port, pwm_num[TIMER_1B_PWM].pin) != PWM_SUCCESS))
        	{
		pwm_en[TIMER_1B_PWM] = 0;
		PRINTF("%s (%u) failed to stop\n", gpt_name(pwm_num[TIMER_1B.PWM].timer),
		pwm_num[TIMER_1B_PWM].ab);
		return STATUS_FAILED;
		}
	       return STATUS_SUCCESS;
	
}


uint8_t  wrapper_pwm_start(void)

{
	if((pwm_en[TIMER_1A_PWM]) &&
         (pwm_start(pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab,
                    pwm_num[TIMER_1A_PWM].port, pwm_num[TIMER_1A_PWM].pin) != PWM_SUCCESS)) {
        pwm_en[TIMER_1A_PWM] = 0;
        PRINTF("%s (%u) failed to start \n", gpt_name(pwm_num[TIMER_1A_PWM].timer),
               pwm_num[TIMER_1A_PWM].ab);

			   return STATUS_FAILED;
      } 
	return STATUS_SUCCESS;
}


uint8_t  beacon_pwm_start(void)

{
	if((pwm_en[TIMER_1B_PWM]) &&
         (pwm_start(pwm_num[TIMER_1B_PWM].timer, pwm_num[TIMER_1B_PWM].ab,
                    pwm_num[TIMER_1B_PWM].port, pwm_num[TIMER_1B_PWM].pin) != PWM_SUCCESS)) {
        pwm_en[TIMER_1B_PWM] = 0;
        PRINTF("%s (%u) failed to start \n", gpt_name(pwm_num[TIMER_1B_PWM].timer),
               pwm_num[TIMER_1B_PWM].ab);

			   return STATUS_FAILED;
      } 
	return STATUS_SUCCESS;
}





uint8_t wrapper_pwm_stop(void)

{
	if((pwm_en[TIMER_1A_PWM]) &&
         (pwm_stop(pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab,
                   pwm_num[TIMER_1A_PWM].port, pwm_num[TIMER_1A_PWM].pin,
                   pwm_num[TIMER_1A_PWM].off_state) != PWM_SUCCESS)) {
        pwm_en[TIMER_1A_PWM] = 0;
        PRINTF("%s (%u) failed to stop\n", gpt_name(pwm_num[TIMER_1A_PWM].timer),
               pwm_num[TIMER_1A_PWM].ab);
		return STATUS_FAILED;
      }
		return STATUS_SUCCESS;
}



uint8_t beacon_pwm_stop(void)
{
	if((pwm_en[TIMER_1B_PWM]) &&
         (pwm_stop(pwm_num[TIMER_1B_PWM].timer, pwm_num[TIMER_1B_PWM].ab,
                   pwm_num[TIMER_1B_PWM].port, pwm_num[TIMER_1B_PWM].pin,
                   pwm_num[TIMER_1B_PWM].off_state) != PWM_SUCCESS)) {
        pwm_en[TIMER_1B_PWM] = 0;
        PRINTF("%s (%u) failed to stop\n", gpt_name(pwm_num[TIMER_1B_PWM].timer),
               pwm_num[TIMER_1B_PWM].ab);
		return STATUS_FAILED;
      }
		return STATUS_SUCCESS;
}

uint8_t  wrapper_pwm_toggle_direction(void)
{
      if((pwm_en[TIMER_1A_PWM]) &&
         (pwm_toggle_direction(pwm_num[TIMER_1A_PWM].timer,
                               pwm_num[TIMER_1A_PWM].ab) != PWM_SUCCESS)) {
        PRINTF("%s (%u) invert failed \n", gpt_name(pwm_num[TIMER_1A_PWM].timer),
               pwm_num[TIMER_1A_PWM].ab);
   		return STATUS_FAILED;

      }
	  	return STATUS_SUCCESS;
}


uint8_t light_power_off(void)
{      
	if(street_light_status.light_is_on) 
	{
	 adjust_brightness(brightness_value, 0);
	 street_light_status.light_is_on=OFF;
	 street_light_status.last_pwr_state=OFF;
	 street_light_status.last_pwr_dwn_mode=NORMAL;
	 street_light_status.presence_detect=PRESENCE_NOT_DETECTED;
	 street_light_status.presence_detect_timer_start=PRESENCE_NOT_DETECTED;
	 eeprom_street_light_status=street_light_status.all_flags;
         eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
         leds_off(LEDS_GREEN);
	 wrapper_pwm_stop();
	 wrapper_pwm_disable();
	 return STATUS_SUCCESS;
	}
	
	else 
	return STATUS_FAILED;
}


uint8_t beacon_light_on(uint16_t frequency, uint8_t dutycycle)
{     

        printf("Beacon  ON   Called\n"); 
 
	if(!street_light_status.beacon_status) 
	{
	 beacon_pwm_enable(30);
	 beacon_pwm_start();
         set_duty_cycle(frequency, dutycycle, pwm_num[TIMER_1B_PWM].timer, pwm_num[TIMER_1B_PWM].ab);	
	 street_light_status.beacon_status=ON;
	 eeprom_street_light_status=street_light_status.all_flags;
         eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
	 return STATUS_SUCCESS;
	}
	
	else 
	return STATUS_FAILED;
}



uint8_t beacon_light_off(void)
{      
	if(street_light_status.light_is_on) 
	{
	 beacon_pwm_stop();
	 beacon_pwm_disable();
	 street_light_status.beacon_status=OFF;
	 eeprom_street_light_status=street_light_status.all_flags;
         eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
	 return STATUS_SUCCESS;
	}
	
	else 
	return STATUS_FAILED;
}



uint8_t adjust_brightness(uint8_t present_duty_cycle, uint8_t target_duty_cycle)
{
  uint8_t duty_cycle_delta;

	if(target_duty_cycle>present_duty_cycle)
	{
	duty_cycle_delta= target_duty_cycle-present_duty_cycle;
	incr_brightness(duty_cycle_delta);	  
	}
  
	else if ( target_duty_cycle<present_duty_cycle)
	{
	duty_cycle_delta= present_duty_cycle-target_duty_cycle;
	decr_brightness(duty_cycle_delta);	
	}

       else if(present_duty_cycle==target_duty_cycle)
	{
        set_duty_cycle(pwm_num[TIMER_1A_PWM].freq, brightness_value, pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab);	
	}

        return 0;
}

uint8_t decr_brightness(uint8_t duty_cycle)
{
uint8_t  decrement;	
	 
  for(decrement=0; decrement<duty_cycle; ++decrement )
   {
   --brightness_value;
   
   set_duty_cycle(pwm_num[TIMER_1A_PWM].freq, brightness_value, pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab);	
   clock_delay_usec(5);
   }
   
   street_light_status.brightness_value=brightness_value;
   eeprom_street_light_status=street_light_status.all_flags;
   eeprom_write_data(EEPROM_BRIGHTNESS_VALUE,eeprom_brightness_value,3);
   eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
   return brightness_value;
}


uint8_t incr_brightness(uint8_t duty_cycle)
{
   uint8_t increment;

   for(increment=0; increment<duty_cycle; ++increment) 
   {
   ++brightness_value;
   set_duty_cycle(pwm_num[TIMER_1A_PWM].freq, brightness_value, pwm_num[TIMER_1A_PWM].timer, pwm_num[TIMER_1A_PWM].ab);	
   clock_delay_usec(5);
   }
  
   street_light_status.brightness_value=brightness_value;  
   eeprom_street_light_status=street_light_status.all_flags;
   eeprom_write_data(EEPROM_BRIGHTNESS_VALUE,eeprom_brightness_value,3);
   eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
  
   return brightness_value;

 }


#if 0
uint16_t  read_load_current(uint8_t adc_ch)
{
       
        float average_current=0;
        uint16_t raw_average_current=0;
        uint16_t i;
        uint16_t number_of_sample=5;
        
        raw_average_current= current_sensor.value(1);
         
        for( i=0; i<number_of_sample;i++)
         {
         average_current = (average_current+CURRENT_SENSOR_RES_MUL*current_sensor.value(1) - CURRENT_SENSOR_RES_OFFSET);
         raw_average_current= raw_average_current+ current_sensor.value(1);
         }

	
	if((average_current<MINIMUM_ON_CURRENT) && (street_light_status.light_is_on==ON))
	{
		street_light_status.overload_fail = 1;
 		eeprom_street_light_status = street_light_status.all_flags;
                eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
		return raw_average_current;
	}
	 
       else if((average_current>MAXIMUM_ON_CURRENT) && (street_light_status.light_is_on==ON))
	{
		street_light_status.overload_fail = 1;
		eeprom_street_light_status=street_light_status.all_flags;
                eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
		return raw_average_current;
	}	 
 return raw_average_current;
}
#endif 

/*---------------------------------------------------------------------------*/
PROCESS(smart_street_light, "smart_street_light");




PROCESS_THREAD(smart_street_light, ev, data)
{

  PROCESS_BEGIN();

  memset(pwm_en, 0, MAX_PWM);
 
   static uint8_t count=0;
   static bool mode_change=false;
  
   printf("Condiitons before While loop Start ================================================\n");
   initial_eeprom_condition();
   street_light_status.all_flags=eeprom_street_light_status;
   street_light_status.presence_detect=0;
   street_light_status.light_is_on=0;
   street_light_status.beacon_status=0;
   street_light_status.siren_status=0;
   street_light_status.presence_detect_timer_start=0;
   street_light_status.brightness_value=0;
   street_light_status.node_reset=1;
   custom_command.operating_mode=street_light_status.operating_mode;
   custom_command.beacon_on=street_light_status.beacon_status;
   custom_command.siren_on=street_light_status.siren_status;
   custom_command.pwr_on_command=street_light_status.pwr_on_command;
   presence_detect_event=process_alloc_event();    
   default_dutycycle_event=process_alloc_event();    
   status_change_event=process_alloc_event();    
   etimer_set(&et, CLOCK_SECOND);


 while(1) {

    PROCESS_YIELD();
    
    bool status_change_flag=false;

    if(ev == PROCESS_EVENT_TIMER) 
   {

   // get_neighbours(); 
    
    if(custom_command.broadcast_data==1)
      {
      printf("Broadcasting Custom Command -1\n");
      data_pack.all_flags=0;
      broadcast_data=0;
      data_pack.brightness_value=custom_command.brightness_value;
      data_pack.reset_node=custom_command.reset_node;
      data_pack.zone_id=custom_command.zone_id;
      data_pack.pwr_on_command=custom_command.pwr_on_command;
      data_pack.operating_mode=custom_command.operating_mode;
      data_pack.beacon_on=custom_command.beacon_on;
      data_pack.siren_on=custom_command.siren_on;
      broadcast_data=data_pack.all_flags;
      count++;
      process_post(&broadcast_process, default_dutycycle_event, &broadcast_data);
      if(count>=3)
      {
      count=0;
      custom_command.broadcast_data=0;
      }
      clock_delay_usec(50);
      }
	 	 
      if(custom_command.operating_mode==MODE_AUTO)
	  { 
            printf("In Operating Mode Auto --- 1 \n");
             if(mode_change==false)
               {
               status_change_flag=true;
               mode_change=true;
               } 
              street_light_status.operating_mode=MODE_AUTO;
              eeprom_read_data(EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE,&eeprom_auto_default_brightness_value,2);
              als_lux_value=read_ambient_light(1);
               

               if(street_light_status.presence_detect_timer_start==1)
                {
                etimer_set(&presence_detected_timer, CLOCK_SECOND*60);   
                street_light_status.presence_detect_timer_start=0;        
                }				
				
		if(etimer_expired(&presence_detected_timer))
		{ 
                  street_light_status.presence_detect=PRESENCE_NOT_DETECTED;
                  etimer_stop(&presence_detected_timer);
		}
				
	  	if(als_lux_value <ALS_DATA_DAWN_MAX)
           	{
               	leds_on(LEDS_GREEN);
               	if(etimer_expired(&presence_detected_timer))//&&(!(street_light_status.presence_detect))))
		{
                        if(street_light_status.light_is_on!=ON) 
		 	{
		 	wrapper_pwm_enable(brightness_value);
		 	wrapper_pwm_start();
                        street_light_status.light_is_on=ON;
		 	adjust_brightness(brightness_value, eeprom_auto_default_brightness_value);
                 	eeprom_street_light_status=street_light_status.all_flags;
                 	status_change_flag=true;
          	 	}
			 
		 	else if ((street_light_status.light_is_on==ON)&&(brightness_value !=eeprom_auto_default_brightness_value))
		 	{
		 	wrapper_pwm_enable(brightness_value);
		 	wrapper_pwm_start();
		 	adjust_brightness(brightness_value, eeprom_auto_default_brightness_value);
		 	street_light_status.brightness_value=brightness_value;
                 	eeprom_street_light_status=street_light_status.all_flags;
                 	status_change_flag=true;
		 	}
			 			 
		}
			
      		else if((street_light_status.presence_detect)&&(!(etimer_expired(&presence_detected_timer))))
		{
			if(street_light_status.light_is_on==ON) 
			{
                	eeprom_read_data(EEPROM_BRIGHTNESS_VALUE,&eeprom_brightness_value,2);
			wrapper_pwm_enable(brightness_value);
			wrapper_pwm_start();
			adjust_brightness(brightness_value, PWM_DUTY_MAX-1);
			street_light_status.brightness_value=brightness_value;
                	eeprom_street_light_status=street_light_status.all_flags;
			}
		}
      	}

        else if((street_light_status.light_is_on==ON) && (als_lux_value>=AMBIENT_LIGHT_CONDITION_OK))			
	{  
	   light_power_off();
           status_change_flag=true;
	}

           
	   eeprom_street_light_status=street_light_status.all_flags;
           eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
		 		  
      }
		 
      else if (custom_command.operating_mode==MODE_MANUAL)
         
	 {

           if(mode_change==true)
             {
             status_change_flag=true;
             mode_change=false;
             }
 
	  street_light_status.operating_mode=MODE_MANUAL;

	  if(custom_command.pwr_on_command==ON)	
	  {
          leds_on(LEDS_GREEN);
          eeprom_read_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE,&eeprom_manual_custom_brightness_value,2);
          eeprom_read_data(EEPROM_BRIGHTNESS_VALUE,&eeprom_brightness_value,2);

	  if((street_light_status.light_is_on==ON) && (eeprom_manual_custom_brightness_value !=brightness_value))
	  {
	  wrapper_pwm_enable(brightness_value);
	  wrapper_pwm_start();
	  adjust_brightness(brightness_value, eeprom_manual_custom_brightness_value);
          street_light_status.brightness_value=brightness_value;
          status_change_flag=true;
	  }
			  				  			  			  	  
          else if (street_light_status.light_is_on==OFF)
	  {
	  wrapper_pwm_enable(0);
	  wrapper_pwm_start();
	  adjust_brightness(brightness_value, eeprom_manual_custom_brightness_value);
	  street_light_status.light_is_on=ON;
          street_light_status.brightness_value=brightness_value;
          status_change_flag=true;
	  }
         
	  street_light_status.pwr_on_command=custom_command.pwr_on_command;
	  eeprom_street_light_status=street_light_status.all_flags;
          eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
         }
			 
	else if ((custom_command.pwr_on_command==OFF)&&(street_light_status.light_is_on==ON))
	  {
           light_power_off();
	   street_light_status.pwr_on_command=custom_command.pwr_on_command;
           eeprom_street_light_status=street_light_status.all_flags;
           eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
           status_change_flag=true;
          }

	else if (custom_command.beacon_on==ON)
	  {
	   beacon_light_on(30, 50);
           status_change_flag=true;
          }

	else if (custom_command.beacon_on==OFF)
	  {
	   beacon_light_on(30, 50);
           status_change_flag=true;
          }

	else if (custom_command.siren_on==ON)
	  {
	   siren_on(30, 50);
           status_change_flag=true;
          }

	else if (custom_command.siren_on==OFF)
	  {
	   siren_off(30, 50);
           status_change_flag=true;
          }


	}
		 
   if(custom_command.reset_node==1)
     {
    custom_command.reset_node=0;
    eeprom_custom_command=custom_command.all_flags;
    eeprom_write_data_big(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 5);   
    sys_ctrl_reset();
    }	
   etimer_set(&et, CLOCK_SECOND);
  }
 
	
else if(ev == sensors_event)
	{
     
	 if((data == &presence_sensor)||(data==&presence_sensor1))
		{
                
                unicast_data=0;
		data_pack_unicast.all_flags=0;
                data_pack_unicast.presence_detect=1;
                unicast_data=data_pack_unicast.all_flags;
	   	etimer_reset(&presence_detected_timer);
	   	street_light_status.presence_detect= PRESENCE_DETECTED; 
	   	street_light_status.presence_detect_timer_start= 1; 
		eeprom_street_light_status=street_light_status.all_flags;
		process_post(&unicast_sender_process, presence_detect_event, &unicast_data);
                eeprom_write_data_big(EEPROM_STREET_LIGHT_STATUS, eeprom_street_light_status, 5);
                status_change_flag=true;
		}  
        }

  if(status_change_flag)
  {
      
      process_post(&http_get_process, status_change_event, NULL);
      resource_street_light_status.trigger();
      coap_push_update_binding(&resource_street_light_status, eeprom_street_light_status);
      status_change_flag=false;

  }

  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */


