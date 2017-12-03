#ifndef __EEPROM_H__
#define __EEPROM_H__

#define NAME_LENGTH 24
#define MAC_ADDR_LENGTH 24
#define VALUE_LENGTH 8

//12 Bytes 
uint16_t  eeprom_checksum; //1
uint16_t   eeprom_zone_id; //1
uint16_t   eeprom_channel;   //1
uint16_t   eeprom_auto_default_brightness_value; //1
uint16_t   eeprom_manual_custom_brightness_value; //1
uint16_t   eeprom_brightness_value; //1 

//12 Bytes 
uint16_t eeprom_pan_id; //2
uint16_t eeprom_node_id; //2
uint32_t eeprom_custom_command; //2
uint32_t eeprom_custom_node_edit; //2
uint32_t eeprom_street_light_status; //2


//112 bytes 
char     eeprom_device_temperature[VALUE_LENGTH]; //8
char     eeprom_battery_voltage[VALUE_LENGTH]; //8
char     eeprom_humidity[VALUE_LENGTH]; //8
char     eeprom_ambient_temperature[VALUE_LENGTH]; //8
char     eeprom_node_latitude[VALUE_LENGTH]; //8
char     eeprom_node_longitude[VALUE_LENGTH]; //8


//174 bytes 
char eeprom_mac_addr[MAC_ADDR_LENGTH]; //24 
char eeprom_mac_addr_str[MAC_ADDR_LENGTH]; //24 
char eeprom_device_name[NAME_LENGTH]; //24 


#define EEPROM_START_ADDR    0x0000
#define EEPROM_END_ADDR    0x01FF

//Page -0
#define  EEPROM_CHECKSUM                   EEPROM_START_ADDR   		//0x00
#define  EEPROM_ZONE_ID                    EEPROM_CHECKSUM+2	  		//0x02
#define  EEPROM_CHANNEL                    EEPROM_ZONE_ID+2       		//0x04
#define  EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE     EEPROM_CHANNEL+2       		//0x06
#define  EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE	   EEPROM_AUTO_DEFAULT_BRIGHTNESS_VALUE+2      //0X08
#define  EEPROM_BRIGHTNESS_VALUE           EEPROM_MANUAL_CUSTOM_BRIGHTNESS_VALUE+2  	//0X0A


#define  EEPROM_PAN_ID    		   EEPROM_BRIGHTNESS_VALUE+2  		//0X1C
#define  EEPROM_NODE_ID                    EEPROM_PAN_ID+2   		//0X1E
#define  EEPROM_CUSTOM_COMMAND             EEPROM_NODE_ID+2 //0X24
#define  EEPROM_CUSTOM_NODE_EDIT           EEPROM_CUSTOM_COMMAND+4 //0X24
#define  EEPROM_STREET_LIGHT_STATUS        EEPROM_CUSTOM_NODE_EDIT+4  	//0X22



#define  EEPROM_DEVICE_TEMP                EEPROM_STREET_LIGHT_STATUS+4     	//0X38~0X3F
#define  EEPROM_BATTERY_VOLTAGE            EEPROM_DEVICE_TEMP+8     	//0X38~0X3F
#define  EEPROM_HUMIDITY                   EEPROM_BATTERY_VOLTAGE+8     //0X40~0X47
#define  EEPROM_AMBIENT_TEMP               EEPROM_HUMIDITY+8     	//0X48~0X4f
#define  EEPROM_LATITUDE                   EEPROM_AMBIENT_TEMP+8     	//0X50~0X57
#define  EEPROM_LONGITUDE                  EEPROM_LATITUDE+8     	//0X58~0X5F
#define  EEPROM_MAC_ADDR                   EEPROM_LONGITUDE+8     	//0XA0~0XB7   //162 
#define  EEPROM_MAC_ADDR_STR               EEPROM_MAC_ADDR+24     	//0XA0~0XB7   //162 
#define  EEPROM_DEVICE_NAME                EEPROM_MAC_ADDR_STR+24




#ifdef NODE_NAME
#define PARAMS_NODE_NAME SERVER_NAME
#else
#define PARAMS_NODE_NAME "node-1"
#endif


#ifdef APPLICATION_NAME
#define PARAMS_APPLICATION_NAME APPLICATION_NAME
#else
#define PARAMS_APPLICATION_NAME "Intellligent Street Light"
#endif


#ifdef DEVICE_NAME
#define PARAMS_DEVICE_NAME DEVICE_NAME
#else
#define PARAMS_DEVICE_NAME "device-1"
#endif


#ifdef SITE_NAME
#define PARAMS_SITE_NAME SITE_NAME
#else
#define PARAMS_SITE_NAME "site-xyz"
#endif


#ifdef ZONE_NAME
#define PARAMS_ZONE_NAME ZONE_NAME
#else
#define PARAMS_ZONE_NAME "zone-1"
#endif





#ifdef NODE_ID
#define PARAMS_NODE_ID NODE_ID
#else
#define PARAMS_NODE_ID 111
#endif



#ifdef CHANNEL_802_15_4
#define PARAMS_CHANNEL CHANNEL_802_15_4
#else
#define PARAMS_CHANNEL 26
#endif


#ifdef IEEE802154_PANID
#define PARAMS_PANID IEEE802154_PANID
#else
#define PARAMS_PANID 0xABCD
#endif


#ifdef IEEE802154_PANADDR
#define PARAMS_PANADDR IEEE802154_PANADDR
#else
#define PARAMS_PANADDR 0
#endif


#ifdef EUI64_ADDRESS
#define PARAMS_EUI64ADDR EUI64_ADDRESS
#else
#define PARAMS_EUI64ADDR { 0x00, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x01 }
#endif

//===========I2C  EEPROM Routines=======================

/**
 * \brief      Perform all operations to send a byte to a slave
 * \param slave_addr The adress of the slave to which data are sent
 * \param data The data to send to the slave
 * \return     Return the value of i2c_master_error() after the I2C operation
 */
uint8_t eeprom_write_string(uint16_t sub_addr, char *string, uint8_t len);
uint8_t eeprom_write_data(uint16_t sub_addr, uint16_t data, uint8_t len);
uint8_t eeprom_write_data_big(uint16_t sub_addr, uint32_t data, uint8_t len);

/**
 * \brief      Perform all operations to receive a byte from a slave
 * \param slave_addr The address of the slave from which data are received
 * \param data A pointer to store the received data
 * \return     Return the value of i2c_master_error() after the I2C operation
 */
uint8_t eeprom_read_data(uint16_t sub_addr, uint16_t *data, uint8_t len);
uint8_t eeprom_read_data_big(uint16_t sub_addr, uint32_t *data, uint8_t len);
uint8_t eeprom_read_string(uint16_t sub_addr, char *string, uint8_t len);

#endif
