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
 *         Sky target resources initialisation
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#include "contiki.h"

#include "coap-common.h"
#include "core-interface.h"

#if WITH_IPSO_APP_FW
#include "ipso-app-fw.h"
#include "sensors-batch-resource.h"
#endif

#if WITH_LWM2M
#include "lwm2m.h"
#include "lwm2m-device-object.h"
#include "ipso-so.h"
#endif

#if !IGNORE_CETIC_CONTIKI_PLATFORM
#include "contiki-resources.h"

/*KDSL  Commented out the  Humidity  resource*/
/*
#include "sht-temp-resource.h"
#include "sht-humidity-resource.h"
*/

#include "battery-resource.h"
#include "radio-lqi-resource.h"
#include "radio-rssi-resource.h"
#include "button-resource.h"
#include "dev/eeprom_cc2538.h"
#include "led-red-resource.h"
#include "led-green-resource.h"
#include "led-blue-resource.h"
//#include "button-left-resource.h"
//#include "button-right-resource.h"
//#include "button-up-resource.h"
//#include "button-down-resource.h"
//#include "button-select-resource.h"
#include "cc2538_temp_sensor-resource.h"
#include "vdd3_sensor-resource.h"
#include "smart_street_light.h"
#include "humidity_sensor-resource.h"
#include "ambient_temperature_sensor-resource.h"

extern street_light_states street_light_status;
extern street_light_custom_cmd custom_command;



#if PLATFORM_HAS_BATTERY
#include "dev/battery-sensor.h"

#define SENSOR_INIT_BATTERY() SENSORS_ACTIVATE(battery_sensor);

#if REST_RES_BATTERY_RAW
#define REST_REST_BATTERY_VALUE battery_sensor.value(0)
#else
#define REST_REST_BATTERY_VALUE ((battery_sensor.value(0) * 5 * 1000L) / 4096)
#endif
#endif


#if PLATFORM_HAS_ON_CHIP_TEMP
#include "dev/cc2538-temp-sensor.h"

#define SENSOR_INIT_CC2538_TEMP_SENSOR() SENSORS_ACTIVATE(cc2538_temp_sensor);

#if REST_RES_CC2538_TEMP_SENSOR_RAW
#define REST_REST_CC2538_TEMP_SENSOR_VALUE cc2538_temp_sensor.value(0)
#else
#define REST_REST_CC2538_TEMP_SENSOR_VALUE cc2538_temp_sensor.value(1) 
#endif
#endif


#if PLATFORM_HAS_ON_CHIP_VOLTAGE
#include "dev/vdd3-sensor.h"

#define SENSOR_INIT_VDD3_SENSOR() SENSORS_ACTIVATE(vdd3_sensor);

#if REST_RES_VDD3_SENSOR_RAW
#define REST_REST_VDD3_SENSOR_VALUE vdd3_sensor.value(0)
#else
#define REST_REST_VDD3_SENSOR_VALUE vdd3_sensor.value(1)
#endif
#endif



#if PLATFORM_HAS_RADIO
#include "dev/radio-sensor.h"

#if UDPCLIENT && UDP_CLIENT_STORE_RADIO_INFO
extern int udp_client_lqi;
extern int udp_client_rssi;
#endif

#define SENSOR_INIT_RADIO_LQI() SENSORS_ACTIVATE(radio_sensor);
#define SENSOR_INIT_RADIO_RSSI() SENSORS_ACTIVATE(radio_sensor);

#if UDPCLIENT && UDP_CLIENT_STORE_RADIO_INFO
#define RADIO_LQI_VALUE_SOURCE udp_client_lqi
#define RADIO_RSSI_VALUE_SOURCE udp_client_rssi
#else
#define RADIO_LQI_VALUE_SOURCE radio_sensor.value(RADIO_SENSOR_LAST_PACKET)
#define RADIO_RSSI_VALUE_SOURCE radio_sensor.value(RADIO_SENSOR_LAST_VALUE)
#endif

#if REST_RES_RADIO_LQI_RAW
#define REST_REST_RADIO_LQI_VALUE RADIO_LQI_VALUE_SOURCE
#else
#if defined CONTIKI_TARGET_CC2538DK || defined CONTIKI_TARGET_ZOUL
#define REST_REST_RADIO_LQI_VALUE ((RADIO_LQI_VALUE_SOURCE*100)/127)
#else
#define REST_REST_RADIO_LQI_VALUE ((RADIO_LQI_VALUE_SOURCE - 50) * 100 / (110-50))
#endif
#endif

#define REST_REST_RADIO_RSSI_VALUE RADIO_RSSI_VALUE_SOURCE

#endif


#if PLATFORM_HAS_TEMPERATURE_HUMIDITY_SENSOR
#include "dev/hdc1080.h"
#include "dev/eeprom_cc2538.h"


extern double temperature;
extern double humidity;
#define SENSOR_INIT_AMBIENT_TEMPERATURE_SENSOR() temperature=0;
#define SENSOR_INIT_HUMIDITY_SENSOR() humidity=0;

#define AMBIENT_TEMPERATURE_SENSOR_VALUE_SOURCE temperature
#define HUMIDITY_SENSOR_VALUE_SOURCE humidity

#define REST_REST_AMBIENT_TEMPERATURE_SENSOR_VALUE AMBIENT_TEMPERATURE_SENSOR_VALUE_SOURCE
#define REST_REST_HUMIDITY_SENSOR_VALUE HUMIDITY_SENSOR_VALUE_SOURCE


#endif


#if PLATFORM_HAS_BUTTON
#include "dev/button-sensor.h"
#if 0
#define SENSOR_INIT_BUTTON_LEFT() SENSORS_ACTIVATE(button_left_sensor);

#define REST_REST_BUTTON_LEFT_VALUE 0 
#define REST_REST_BUTTON_LEFT_ACTUATOR reset_node

static int reset_node(uint8_t value) {
        if(value ==1)
{      
      eeprom_read_data(EEPROM_CUSTOM_COMMAND, &eeprom_custom_command, 2);
      custom_command.reset_node=1;
      eeprom_custom_command=custom_command.all_flags;
      eeprom_write_data(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 3);

 
}
   return 1;


}



#define SENSOR_INIT_BUTTON_RIGHT() SENSORS_ACTIVATE(button_right_sensor);
#define REST_REST_BUTTON_RIGHT_VALUE custom_command.operating_mode
#define REST_REST_BUTTON_RIGHT_ACTUATOR operating_mode

static int operating_mode(uint16_t value) {

        if((value==0)||(value==1))
        {
        eeprom_read_data(EEPROM_CUSTOM_COMMAND, &eeprom_custom_command, 2);
        custom_command.operating_mode=value;
        }

       eeprom_custom_command=custom_command.all_flags;       
       eeprom_write_data(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 3);
  
   return 1;
}


#define SENSOR_INIT_BUTTON_UP() SENSORS_ACTIVATE(button_up_sensor);
#define REST_REST_BUTTON_UP_VALUE eeprom_manual_custom_brightness_value
#define REST_REST_BUTTON_UP_ACTUATOR  change_brightness

static int change_brightness(uint8_t value) {
            if(value>=100);
              value=99;

            if(value >=0 || value <=99)
	{ 
            eeprom_read_data(EEPROM_CUSTOM_COMMAND, &eeprom_custom_command, 2);
            custom_command.brightness_value=value;
            eeprom_custom_command=custom_command.all_flags;
	    eeprom_manual_custom_brightness_value=value;
	}
       eeprom_write_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE, eeprom_manual_custom_brightness_value, 3);
       eeprom_write_data(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 3);

   return 1;
}


#define SENSOR_INIT_BUTTON_DOWN() SENSORS_ACTIVATE(button_up_sensor);
#define REST_REST_BUTTON_DOWN_VALUE street_light_status.pwr_on_command
#define REST_REST_BUTTON_DOWN_ACTUATOR  power_on_off

static int power_on_off(uint8_t value) {
   

        if(value ==0)
	{  
            eeprom_read_data(EEPROM_CUSTOM_COMMAND, &eeprom_custom_command, 2);
            custom_command.pwr_on_command= 0;

	}

        if(value ==1)
	{  
            custom_command.pwr_on_command= 1;

	}

       eeprom_custom_command=custom_command.all_flags;
       eeprom_write_data(EEPROM_CUSTOM_COMMAND, eeprom_custom_command, 3);
return 1;
}

#endif 

#define REST_CONF_RES_RESOURCEID_EVENT_HANDLER(ev, data) \
    if (ev == sensors_event && data == &button_sensor) { \
      resource_button.trigger(); \
      coap_push_update_binding(&resource_button, button_sensor.value(1)); \
    }

#endif

#if PLATFORM_HAS_LEDS
#include "dev/leds.h"

#if REST_RES_LED_RED
inline int led_r_value(void) {
  return ((leds_get() & LEDS_RED) != 0);
}

static int led_r_set(uint32_t value, uint32_t len) {
  if (value) {
    leds_on(LEDS_RED);
  } else {
    leds_off(LEDS_RED);
  }
  return 1;
}

#define SENSOR_INIT_LED_RED()
#define REST_REST_LED_RED_VALUE led_r_value()
#define REST_REST_LED_RED_ACTUATOR led_r_set
#endif

#if REST_RES_LED_GREEN
inline int led_g_value(void) {
  return ((leds_get() & LEDS_GREEN) != 0);
}

static int led_g_set(uint32_t value, uint32_t len) {
  if (value) {
    leds_on(LEDS_GREEN);
  } else {
    leds_off(LEDS_GREEN);
  }
  return 1;
}

#define SENSOR_INIT_LED_GREEN()
#define REST_REST_LED_GREEN_VALUE led_g_value()
#define REST_REST_LED_GREEN_ACTUATOR led_g_set
#endif

#if REST_RES_LED_BLUE
inline int led_b_value(void) {
  return ((leds_get() & LEDS_BLUE) != 0);
}

static int led_b_set(uint32_t value, uint32_t len) {
  if (value) {
    leds_on(LEDS_BLUE);
  } else {
    leds_off(LEDS_BLUE);
  }
  return 1;
}

#define SENSOR_INIT_LED_BLUE()
#define REST_REST_LED_BLUE_VALUE led_b_value()
#define REST_REST_LED_BLUE_ACTUATOR led_b_set
#endif

#endif

//REST_RES_SHT_TEMP_DECLARE();
//REST_RES_SHT_HUMIDITY_DECLARE();
REST_RES_BATTERY_DECLARE();
REST_RES_CC2538_TEMP_SENSOR_DECLARE();
REST_RES_VDD3_SENSOR_DECLARE();
REST_RES_AMBIENT_TEMPERATURE_SENSOR_DECLARE();
REST_RES_HUMIDITY_SENSOR_DECLARE();
REST_RES_RADIO_LQI_DECLARE();
REST_RES_RADIO_RSSI_DECLARE();
//REST_RES_BUTTON_LEFT_DECLARE();
//REST_RES_BUTTON_RIGHT_DECLARE();
//REST_RES_BUTTON_DOWN_DECLARE();
//REST_RES_BUTTON_UP_DECLARE();
//REST_RES_BUTTON_SELECT_DECLARE();
REST_RES_LED_RED_DECLARE();
REST_RES_LED_GREEN_DECLARE();
REST_RES_LED_BLUE_DECLARE();
//REST_RES_DEVICE_STATUS_DECLARE(); Sumit 

REST_RES_BATTERY_DEFINE();
REST_RES_CC2538_TEMP_SENSOR_DEFINE();
REST_RES_VDD3_SENSOR_DEFINE();
REST_RES_RADIO_LQI_DEFINE();
REST_RES_RADIO_RSSI_DEFINE();
REST_RES_AMBIENT_TEMPERATURE_SENSOR_DEFINE();
REST_RES_HUMIDITY_SENSOR_DEFINE();
//REST_RES_BUTTON_LEFT_DEFINE();
//REST_RES_BUTTON_RIGHT_DEFINE(IF_ACTUATOR, GPIO_BUTTON_RT, STATE_SENSOR_RES_ID);
//REST_RES_BUTTON_UP_DEFINE(IF_ACTUATOR, GPIO_BUTTON_RT, STATE_SENSOR_RES_ID);
//REST_RES_BUTTON_DOWN_DEFINE(IF_ACTUATOR, GPIO_BUTTON_RT, STATE_SENSOR_RES_ID);
//REST_RES_BUTTON_SELECT_DEFINE(IF_ACTUATOR, GPIO_BUTTON_RT, STATE_SENSOR_RES_ID);
REST_RES_LED_RED_DEFINE();
REST_RES_LED_GREEN_DEFINE();
REST_RES_LED_BLUE_DEFINE();





/*KDSL*/

void
contiki_platform_resources_init(void)
{

  REST_RES_RADIO_LQI_INIT();
  REST_RES_RADIO_RSSI_INIT();
  REST_RES_AMBIENT_TEMPERATURE_SENSOR_INIT();
  REST_RES_HUMIDITY_SENSOR_INIT();
  //REST_RES_BUTTON_LEFT_INIT();
//  REST_RES_BUTTON_RIGHT_INIT();
 // REST_RES_BUTTON_UP_INIT();
  //REST_RES_BUTTON_DOWN_INIT();
 // REST_RES_BUTTON_SELECT_INIT();
  REST_RES_LED_RED_INIT();
  REST_RES_LED_GREEN_INIT();
REST_RES_VDD3_SENSOR_INIT();
REST_RES_CC2538_TEMP_SENSOR_INIT();

  REST_RES_LED_BLUE_INIT();
#if WITH_LWM2M
  REST_RES_BATTERY_INIT_WITH_PATH(LWM2M_DEVICE_PATH(LWM2M_DEVICE_POWER_VOLTAGE_RESOURCE_ID));
#endif
#if WITH_IPSO_APP_FW
  REST_RES_BATTERY_INIT_WITH_PATH(DEVICE_POWER_SUPPLY_ENUM_LINE DEVICE_POWER_SUPPLY_VOLTAGE_RES);
#endif
}
#endif /* !IGNORE_CETIC_CONTIKI_PLATFORM */
