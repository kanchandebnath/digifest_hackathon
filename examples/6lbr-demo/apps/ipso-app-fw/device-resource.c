/*
 * Copyright (c) 2014, CETIC.
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
 *         Simple CoAP Library
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */
#include "coap-common.h"
#include "core-interface.h"
#include "device-resource.h"
#include "device-info-resource.h"
#include "smart_street_light.h"
#include "dev/eeprom_cc2538.h"

#if WITH_CETIC_6LN_NVM
#include "nvm-config.h"
#endif

extern street_light_states street_light_status;
extern street_light_custom_cmd custom_command;
extern node_edit custom_node_edit;

//#define REST_RES_DEVICE_STATUS_DECLARE() 

extern resource_t resource_street_light_status;

/*---------------------------------------------------------------------------*/

#if REST_RES_DEVICE_BATCH
#define REST_RES_DEVICE_BATCH_RESOURCE BATCH_RESOURCE
#else
#define REST_RES_DEVICE_BATCH_RESOURCE(...)
#endif

#if REST_RES_DEVICE_MANUFACTURER
#define REST_RES_DEVICE_MANUFACTURER_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_MANUFACTURER_REF &resource_device_manufacturer,
#else
#define REST_RES_DEVICE_MANUFACTURER_RESOURCE(...)
#define REST_RES_DEVICE_MANUFACTURER_REF
#endif

#if REST_RES_DEVICE_MODEL
#define REST_RES_DEVICE_MODEL_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_MODEL_REF &resource_device_model,
#else
#define REST_RES_DEVICE_MODEL_RESOURCE(...)
#define REST_RES_DEVICE_MODEL_REF
#endif

#if REST_RES_DEVICE_MODEL_HW
#define REST_RES_DEVICE_MODEL_HW_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_MODEL_HW_REF &resource_device_model_hw,
#else
#define REST_RES_DEVICE_MODEL_HW_RESOURCE(...)
#define REST_RES_DEVICE_MODEL_HW_REF
#endif

#if REST_RES_DEVICE_MODEL_SW
#define REST_RES_DEVICE_MODEL_SW_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_MODEL_SW_REF &resource_device_model_sw,
#else
#define REST_RES_DEVICE_MODEL_SW_RESOURCE(...)
#define REST_RES_DEVICE_MODEL_SW_REF
#endif

#if REST_RES_DEVICE_SERIAL
#define REST_RES_DEVICE_SERIAL_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_SERIAL_REF &resource_device_serial,
#else
#define REST_RES_DEVICE_SERIAL_RESOURCE(...)
#define REST_RES_DEVICE_SERIAL_REF
#endif


#if REST_RES_DEVICE_EUID
#define REST_RES_DEVICE_EUID_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_EUID_REF &resource_device_euid,
#else
#define REST_RES_DEVICE_EUID_RESOURCE(...)
#define REST_RES_DEVICE_EUID_REF
#endif


#if REST_RES_DEVICE_NAME
#define REST_RES_DEVICE_NAME_RESOURCE REST_ACTUATOR
#define REST_RES_DEVICE_NAME_REF &resource_device_name,
#else
#define REST_RES_DEVICE_NAME_RESOURCE(...)
#define REST_RES_DEVICE_NAME_REF
#endif


#if REST_RES_DEVICE_STATUS
//#define REST_RES_DEVICE_STATUS_RESOURCE REST_PERIODIC_RESOURCE 
#define REST_RES_DEVICE_STATUS_RESOURCE REST_EVENT_RESOURCE 
#define REST_RES_DEVICE_STATUS_REF &resource_street_light_status,
#else
#define REST_RES_DEVICE_STATUS_RESOURCE(...)
#define REST_RES_DEVICE_STATUS_REF
#endif


#if REST_RES_ZONE_ID
#define REST_RES_ZONE_ID_RESOURCE REST_ACTUATOR
#define REST_RES_ZONE_ID_REF &resource_ZONE_ID,
#else
#define REST_RES_ZONE_ID_RESOURCE(...)
#define REST_RES_ZONE_ID_REF
#endif


#if REST_RES_PAN_ID
#define REST_RES_PAN_ID_RESOURCE REST_ACTUATOR
#define REST_RES_PAN_ID_REF &resource_PAN_ID,
#else
#define REST_RES_PAN_ID_RESOURCE(...)
#define REST_RES_PAN_ID_REF
#endif

#if REST_RES_NODE_ID
#define REST_RES_NODE_ID_RESOURCE REST_ACTUATOR
#define REST_RES_NODE_ID_REF &resource_NODE_ID,
#else
#define REST_RES_NODE_ID_RESOURCE(...)
#define REST_RES_NODE_ID_REF
#endif

#if REST_RES_NODE_LATITUDE
#define REST_RES_NODE_LATITUDE_RESOURCE REST_ACTUATOR
#define REST_RES_NODE_LATITUDE_REF &resource_node_latitude,
#else
#define REST_RES_NODE_LATITUDE_RESOURCE(...)
#define REST_RES_NODE_LATITUDE_REF
#endif

#if REST_RES_NODE_LONGITUDE
#define REST_RES_NODE_LONGITUDE_RESOURCE REST_ACTUATOR
#define REST_RES_NODE_LONGITUDE_REF &resource_node_longitude,
#else
#define REST_RES_NODE_LONGITUDE_RESOURCE(...)
#define REST_RES_NODE_LONGITUDE_REF
#endif



#if REST_RES_NODE_CUSTOM_COMMAND
#define REST_RES_NODE_CUSTOM_COMMAND_RESOURCE REST_ACTUATOR
#define REST_RES_NODE_CUSTOM_COMMAND_REF &resource_custom_command,
#else
#define REST_RES_NODE_CUSTOM_COMMAND_RESOURCE(...)
#define REST_RES_NODE_CUSTOM_COMMAND_REF
#endif


#if REST_RES_NODE_CUSTOM_NODE_EDIT
#define REST_RES_NODE_CUSTOM_NODE_EDIT_RESOURCE REST_ACTUATOR
#define REST_RES_NODE_CUSTOM_NODE_EDIT_REF &resource_custom_node_edit,
#else
#define REST_RES_NODE_CUSTOM_NODE_EDIT_RESOURCE(...)
#define REST_RES_NODE_CUSTOM_NODE_EDIT_REF
#endif





#if REST_RES_DEVICE_TIME
#define REST_RES_DEVICE_TIME_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_TIME_REF &resource_device_time,
#else
#define REST_RES_DEVICE_TIME_RESOURCE(...)
#define REST_RES_DEVICE_TIME_REF
#endif

#if REST_RES_DEVICE_UPTIME
#define REST_RES_DEVICE_UPTIME_RESOURCE REST_RESOURCE
#define REST_RES_DEVICE_UPTIME_REF &resource_device_uptime,
#else
#define REST_RES_DEVICE_UPTIME_RESOURCE(...)
#define REST_RES_DEVICE_UPTIME_REF
#endif

#ifdef RES_CONF_DEVICE_NAME_VALUE
#define RES_DEVICE_NAME_VALUE RES_CONF_DEVICE_NAME_VALUE
#else
#define RES_DEVICE_NAME_VALUE eeprom_device_name
#endif


inline int device_name_set(uint8_t const* name, int len) {
  if (len < REST_MAX_DEVICE_NAME_LENGTH) {
    strncpy(RES_DEVICE_NAME_VALUE, (const char*)name,23);
    strcpy(eeprom_device_name,RES_DEVICE_NAME_VALUE);
    printf("Device name eceived is %s\n",name);
    eeprom_device_name[23]='\0';
    eeprom_write_string(EEPROM_DEVICE_NAME, eeprom_device_name, REST_MAX_DEVICE_NAME_LENGTH);
    return 1;
  } else {
    return 0;
  }
}

#ifdef RES_CONF_NODE_ID_VALUE
#define RES_NODE_ID_VALUE RES_CONF_NODE_ID_VALUE
#else
#define RES_NODE_ID_VALUE eeprom_node_id 
#endif
inline int device_node_id_set(uint16_t value) {
  if (value != 0) {
     RES_NODE_ID_VALUE = value&0x0fff;
     eeprom_node_id=RES_NODE_ID_VALUE;
     eeprom_write_data(EEPROM_NODE_ID, eeprom_node_id, 3);
    return 1;
  } else {
    return 0;
  }
}



#ifdef RES_CONF_NODE_LATITUDE_VALUE
#define RES_NODE_LATITUDE_VALUE RES_CONF_NODE_LATITUDE_VALUE
#else
#define RES_NODE_LATITUDE_VALUE eeprom_node_latitude
#endif


inline int device_node_latitude_set(char *value, uint8_t len) {
  if (len<=8) {
    strncpy(RES_NODE_LATITUDE_VALUE, value,7);
    strcpy(eeprom_node_latitude, RES_NODE_LATITUDE_VALUE);
    eeprom_node_latitude[7]='\0';
    eeprom_write_string(EEPROM_LATITUDE, eeprom_node_latitude, 8);
    return 1;
  } else {
    return 0;
  }
}




#ifdef RES_CONF_NODE_LONGITUDE_VALUE
#define RES_NODE_LONGITUDE_VALUE RES_CONF_NODE_LONGITUDE_VALUE
#else
#define RES_NODE_LONGITUDE_VALUE eeprom_node_longitude
#endif


inline int device_node_longitude_set(char *value, uint8_t len) {
  if (len<=8) {
    strncpy(RES_NODE_LONGITUDE_VALUE, value,7);
    strcpy(eeprom_node_longitude, RES_NODE_LONGITUDE_VALUE);
    eeprom_node_latitude[7]='\0';
    eeprom_write_string(EEPROM_LONGITUDE, eeprom_node_longitude, 8);
    return 1;
  } else {
    return 0;
  }
}


#ifdef RES_CONF_NODE_CUSTOM_COMMAND_VALUE
#define RES_NODE_CUSTOM_COMMAND_VALUE RES_CONF_NODE_CUSTOM_COMMAND_VALUE
#else
#define RES_NODE_CUSTOM_COMMAND_VALUE eeprom_custom_command
#endif



#ifdef RES_CONF_NODE_CUSTOM_NODE_EDIT_VALUE
#define RES_NODE_CUSTOM_NODE_EDIT_VALUE RES_CONF_NODE_CUSTOM_NODE_EDIT_VALUE
#else
#define RES_NODE_CUSTOM_NODE_EDIT_VALUE eeprom_custom_node_edit
#endif

inline int device_custom_node_edit_set(char const* value, uint8_t len)

{
     
     printf("Length of String received %d\n", len);  //  RES_NODE_CUSTOM_NODE_EDIT_VALUE = *(uint32_t*)value;
     printf("Custom command Node Edit Node_ID-1 Value =%d\n",custom_node_edit.node_id);
     printf("Custom command Node Edit PAN_ID-1 Value =%d\n",custom_node_edit.pan_id);
     printf("Custom Command Node EDit Zone ID-1 Value =%d\n",custom_node_edit.zone_id);
     printf("Custom command Node Edit Device Name-1 Value =%s\n",custom_node_edit.device_name);

     node_edit *temp_custom_node_edit = (node_edit*)value;
     printf("Size of Node mmmmm  EDIT %d\n", sizeof(node_edit));
     memset(&custom_node_edit,0,sizeof(node_edit));
     printf("Custom command Node Edit Device Name-1 ========Value =%s\n",value);
     printf("Custom command Node Edit Device Name-1 ========Value =%s\n",(char*)temp_custom_node_edit);
   //  eeprom_pan_id = temp_custom_node_edit->pan_id;
     custom_node_edit.node_id = temp_custom_node_edit->node_id;
     custom_node_edit.pan_id =  temp_custom_node_edit->pan_id;
     custom_node_edit.zone_id = temp_custom_node_edit->zone_id;
     strncpy(custom_node_edit.device_name, temp_custom_node_edit->device_name,23);
     strncpy(eeprom_device_name, custom_node_edit.device_name,23);
     custom_node_edit.device_name[23]='\0';
     eeprom_device_name[23]='\0';

     #if 1
    // printf("Custom Command Node EDit Zone ID Value =%d\n",temp_custom_node_edit->zone_id);
    // printf("Custom command Node Edit Node_ID Value =%d\n",temp_custom_node_edit->node_id);
    // printf("Custom command Node Edit PAN_ID Value =%d\n",temp_custom_node_edit->pan_id);
    //printf("Custom command Node Edit Device Name Value =%s\n",eeprom_device_name);
     printf("Custom command Node Edit Node_ID-1 Value =%d\n",custom_node_edit.node_id);
     printf("Custom command Node Edit PAN_ID-1 Value =%d\n",custom_node_edit.pan_id);
     printf("Custom Command Node EDit Zone ID-1 Value =%d\n",custom_node_edit.zone_id);
     printf("Custom command Node Edit Device Name-1 Value =%s\n",custom_node_edit.device_name);
     #endif 

     eeprom_write_data(EEPROM_PAN_ID, eeprom_pan_id,3);
     eeprom_write_data(EEPROM_NODE_ID, eeprom_node_id,3);
     eeprom_write_data(EEPROM_ZONE_ID, eeprom_zone_id,3);
     eeprom_write_string(EEPROM_DEVICE_NAME, eeprom_device_name, REST_MAX_DEVICE_NAME_LENGTH);

     return 1; 
}






inline int device_custom_command_set(uint32_t value) 
{
     RES_NODE_CUSTOM_COMMAND_VALUE=value;
     custom_command.all_flags=value;
     if(custom_command.brightness_value>=100)
       custom_command.brightness_value=99;
     eeprom_custom_command=custom_command.all_flags;
    #if 1
     printf("Custom Command Brigtness Value=%d\n",custom_command.brightness_value);
     printf("Custom command Operating Mode=%d\n",custom_command.operating_mode);
     printf("Custom command Pwr_on_Command=%d\n",custom_command.pwr_on_command);
     printf("Custom command Broadcast_data=%d\n",custom_command.broadcast_data);
     printf("Custom command Zone_ID=%d\n",custom_command.zone_id);
     printf("Custom command Reset_node=%d\n",custom_command.reset_node);
     printf("Custom command Siren On Status=%d\n",custom_command.siren_on);
     printf("Custom command Beacon_On=%d\n",custom_command.beacon_on);
     #endif 
     eeprom_write_data_big(EEPROM_CUSTOM_COMMAND, eeprom_custom_command,5);

      
     if(custom_command.operating_mode==MODE_MANUAL)
     {
     eeprom_manual_custom_brightness_value=custom_command.brightness_value;
     eeprom_write_data(EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE, eeprom_manual_custom_brightness_value, 3);
     }

     if(custom_command.operating_mode==MODE_AUTO)
     {
     eeprom_auto_default_brightness_value=custom_command.brightness_value;
     eeprom_write_data(EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE, eeprom_auto_default_brightness_value,3);
     }
    return 1;
 }



//Adding  zone-id , zone name and  device status
//===============================================


#ifdef RES_CONF_ZONE_ID_VALUE
#define RES_ZONE_ID_VALUE RES_CONF_ZONE_ID_VALUE
#else
#define RES_ZONE_ID_VALUE eeprom_zone_id
#endif


inline int device_zone_id_set(uint8_t value) {
  if (value<=31) {
    RES_ZONE_ID_VALUE=value&0x001F;
    eeprom_zone_id=RES_ZONE_ID_VALUE;
    eeprom_write_data(EEPROM_ZONE_ID, eeprom_zone_id, 3);
    return 1;
  } else {
    return 0;
  }
}



#ifdef RES_CONF_PAN_ID_VALUE
#define RES_PAN_ID_VALUE RES_CONF_PAN_ID_VALUE
#else
#define RES_PAN_ID_VALUE eeprom_pan_id
#endif


inline int device_pan_id_set(uint16_t value) {
  if (value<=0xFFFF) {
    RES_PAN_ID_VALUE=value;
    eeprom_pan_id=RES_PAN_ID_VALUE;
    eeprom_write_data(EEPROM_ZONE_ID, eeprom_pan_id, 3);
    return 1;
  } else {
    return 0;
  }
}




#ifdef RES_CONF_DEVICE_STATUS_VALUE
#define RES_DEVICE_STATUS_VALUE RES_CONF_DEVICE_STATUS_VALUE
#else
#define RES_DEVICE_STATUS_VALUE eeprom_street_light_status
#endif


inline int device_status_value_set(uint32_t value) {
return 0;
}


REST_RES_DEVICE_MANUFACTURER_RESOURCE(device_manufacturer,
    ,
    IF_RO_PARAMETER,
    DEVICE_MANUFACTURER_RT,
    COAP_RESOURCE_TYPE_STRING, "mfg", RES_DEVICE_MANUFACTURER_VALUE)

REST_RES_DEVICE_MODEL_RESOURCE(device_model,
    ,
    IF_RO_PARAMETER,
    DEVICE_MODEL_RT,
    COAP_RESOURCE_TYPE_STRING, "mdl", RES_DEVICE_MODEL_VALUE)

REST_RES_DEVICE_MODEL_HW_RESOURCE(device_model_hw,
    ,
    IF_RO_PARAMETER,
    DEVICE_MODEL_HW_RT,
    COAP_RESOURCE_TYPE_STRING, "Hardware", RES_DEVICE_MODEL_HW_VALUE)

REST_RES_DEVICE_MODEL_SW_RESOURCE(device_model_sw,
    ,
    IF_RO_PARAMETER,
    DEVICE_MODEL_SW_RT,
    COAP_RESOURCE_TYPE_STRING, "Software", RES_DEVICE_MODEL_SW_VALUE)

REST_RES_DEVICE_SERIAL_RESOURCE(device_serial,
    ,
    IF_RO_PARAMETER,
    DEVICE_SERIAL_RT,
    COAP_RESOURCE_TYPE_STRING, "Serial_No", RES_DEVICE_SERIAL_VALUE)


REST_RES_DEVICE_EUID_RESOURCE(device_euid,
    ,
    IF_RO_PARAMETER,
    DEVICE_EUID_RT,
    COAP_RESOURCE_TYPE_STRING, "Device_UID", RES_DEVICE_EUID_VALUE)

REST_RES_DEVICE_NAME_RESOURCE(device_name,

    ,
    IF_RO_PARAMETER,
    DEVICE_NAME_RT,
    COAP_RESOURCE_TYPE_STRING, "Device_Name", RES_DEVICE_NAME_VALUE, device_name_set) 

REST_RES_NODE_ID_RESOURCE(NODE_ID,
    ,
    IF_RO_PARAMETER,
    NODE_ID_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "Node_ID", RES_NODE_ID_VALUE,device_node_id_set)


REST_RES_ZONE_ID_RESOURCE(ZONE_ID,
    ,
    IF_RO_PARAMETER,
    ZONE_ID_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "Zone_ID", RES_ZONE_ID_VALUE,device_zone_id_set)


REST_RES_PAN_ID_RESOURCE(PAN_ID,
    ,
    IF_RO_PARAMETER,
    PAN_ID_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "PAN_ID", RES_PAN_ID_VALUE,device_pan_id_set)

REST_RES_DEVICE_STATUS_RESOURCE(street_light_status,
    /*30,*/
    ,
    IF_RO_PARAMETER,
    DEVICE_STATUS_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "Device_status", RES_DEVICE_STATUS_VALUE)

REST_RES_NODE_LATITUDE_RESOURCE(node_latitude,
    ,
    IF_RO_PARAMETER,
    NODE_LATITUDE_RT,
    COAP_RESOURCE_TYPE_STRING, "Node_Latitude", RES_NODE_LATITUDE_VALUE, device_node_latitude_set)


REST_RES_NODE_LONGITUDE_RESOURCE(node_longitude,
    ,
    IF_RO_PARAMETER,
    NODE_LONGITUDE_RT,
    COAP_RESOURCE_TYPE_STRING, "Node_Longitude", RES_NODE_LONGITUDE_VALUE, device_node_longitude_set)


REST_RES_NODE_CUSTOM_COMMAND_RESOURCE(custom_command,
    ,
    IF_RO_PARAMETER,
    NODE_CUSTOM_COMMAND_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "Custom_Command", RES_NODE_CUSTOM_COMMAND_VALUE, device_custom_command_set)


REST_RES_NODE_CUSTOM_NODE_EDIT_RESOURCE(custom_node_edit,
    ,
    IF_RO_PARAMETER,
    NODE_CUSTOM_NODE_EDIT_RT,
    COAP_RESOURCE_TYPE_STRING, "Custom_Node_Edit", RES_NODE_CUSTOM_NODE_EDIT_VALUE, device_custom_node_edit_set)




REST_RES_DEVICE_TIME_RESOURCE(device_time,
    ,
    IF_RO_PARAMETER,
    DEVICE_TIME_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "time", RES_DEVICE_TIME_VALUE)

REST_RES_DEVICE_UPTIME_RESOURCE(device_uptime,
    ,
    IF_RO_PARAMETER,
    DEVICE_UPTIME_RT,
    COAP_RESOURCE_TYPE_UNSIGNED_INT, "uptime", RES_DEVICE_UPTIME_VALUE)

/*---------------------------------------------------------------------------*/
REST_RES_DEVICE_BATCH_RESOURCE(device, 0, IF_BATCH, DEVICE_RT,
    REST_RES_DEVICE_MANUFACTURER_REF
    REST_RES_DEVICE_MODEL_REF
    REST_RES_DEVICE_MODEL_HW_REF
    REST_RES_DEVICE_MODEL_SW_REF
    REST_RES_DEVICE_SERIAL_REF
    REST_RES_DEVICE_NAME_REF
    REST_RES_DEVICE_STATUS_REF
    REST_RES_ZONE_ID_REF
    REST_RES_NODE_ID_REF
    REST_RES_NODE_LATITUDE_REF
    REST_RES_NODE_LONGITUDE_REF
    REST_RES_NODE_CUSTOM_COMMAND_REF
    REST_RES_NODE_CUSTOM_NODE_EDIT_REF
    REST_RES_DEVICE_TIME_REF
    REST_RES_DEVICE_UPTIME_REF
    )
/*---------------------------------------------------------------------------*/
