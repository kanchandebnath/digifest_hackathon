#include <stdint.h>
#include <string.h>
#include "eeprom_cc2538.h"
#include "malloc.h"
#include "i2c.h"
#include "contiki.h"
#include "dev/nvic.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "sys/timer.h"

#define EEPROM_WP_PORT_BASE    GPIO_PORT_TO_BASE(EEPROM_WP_PORT)
#define EEPROM_WP_PIN_MASK     GPIO_PIN_MASK(EEPROM_WP_PIN)
#define EEPROM_SLAVE_ADDR  0x50
/* EEPROM SLAVE  ADDRESS
   Bit 7~Bit4 : Fix 1010
   Bit 3~2 ( depends upon the condition of PIN  EEPROM Pin 2 and 3 of EEPROM, in our case is not connected,  but internally Pulled down
   Bit 1 is : Page Address  0= Page 0  and 1 Means Page 1
   Bit 0 is the R/W Bit = Bit 1 0 Means Write Address , 1 Means Read
 */ 

typedef enum
{
  OK       = 0x00,
  ERROR    = 0x01,
  BUSY     = 0x02,
  TIMEOUT  = 0x03
} EEPROM_StatusTypeDef;


// This  function drives the pin PD4 or PD5 to low, thereby  enabling EEPROM  write 

void  enable_eeprom_write(void)
{
GPIO_SOFTWARE_CONTROL(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
GPIO_SET_OUTPUT(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
GPIO_CLR_PIN(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
ioc_set_over(EEPROM_WP_PORT, EEPROM_WP_PIN, IOC_OVERRIDE_DIS);
GPIO_CLR_PIN(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);

}

// This function drives the EEPROM Pin HIGH(PD4/PD5) ,  so once the  PIN is set to high,  we can not write  to it 

void  disable_eeprom_write(void)
{
GPIO_SOFTWARE_CONTROL(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
GPIO_SET_OUTPUT(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
GPIO_CLR_PIN(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);
ioc_set_over(EEPROM_WP_PORT, EEPROM_WP_PIN, IOC_OVERRIDE_DIS);
GPIO_SET_PIN(EEPROM_WP_PORT_BASE, EEPROM_WP_PIN_MASK);

}

/*
 * eeprom_write() - Write register
 * @reg: Register address
 * @val: 8-bit register value from the 
 * Returns HAL status or HAL_ERROR for invalid arguments.
 */

uint8_t eeprom_write_string(uint16_t sub_addr, char *string, uint8_t len)
{
      uint8_t buf[len+1];
      uint8_t error=ERROR;
    
      if (sub_addr > EEPROM_END_ADDR)
       return error;
     uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
     uint8_t page_addr= (uint8_t)(sub_addr>>8);
   
     eeprom_slave_addr |= page_addr;
     memcpy(buf,&sub_addr, sizeof(uint8_t));     
     memcpy(buf+1,string, sizeof(uint8_t)*len);     

     enable_eeprom_write();
     clock_delay_usec(10);
     error = i2c_burst_send(eeprom_slave_addr,buf,len);
 
     if (error != OK)
     {
     printf("I2C  EEPROM Error-1\n");
     return error;
     }

  return OK;
}



uint8_t eeprom_read_string( uint16_t sub_addr, char *string, uint8_t len)
{
      uint8_t error=ERROR;
        if (sub_addr > EEPROM_END_ADDR)
                return error; 
        
        uint8_t buf[len];
        uint8_t page_addr= (uint8_t)(sub_addr>>8);
        uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
        eeprom_slave_addr |= page_addr;
        
        memcpy(buf,&sub_addr, sizeof(uint8_t));     
        /* Send the read followed by address */
        enable_eeprom_write();
        clock_delay_usec(10);
        error = i2c_single_send(eeprom_slave_addr,buf[0]);
        if (error != OK)
        {
  
        printf("I2C EEPROM Error-5\n");
         return error;

        }
        clock_delay_usec(10);
        /* Receive a 2-byte result */
        error = i2c_burst_receive(eeprom_slave_addr, buf, len );
        if (error != OK)
          {
          printf("I2C EEPROM ERROR-6\n");
                return error;
          }
        memcpy(string, buf, sizeof(uint8_t)*len);
        return OK;  /* Success */


}


/*
 * eeprom_write() - Write register
 * @reg: Register address
 * @val: 8-bit register value from the 
 * Returns HAL status or HAL_ERROR for invalid arguments.
 */

uint8_t eeprom_write_data(uint16_t sub_addr, uint16_t data, uint8_t len)
{
	uint8_t buf[len];
      uint8_t error=ERROR;
	if (sub_addr > EEPROM_END_ADDR)   // dummy write to adr 0 ... trigger measurement
        return error;
 
	uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
 	uint8_t page_addr= (uint8_t)(sub_addr>>8);
	eeprom_slave_addr |= page_addr;
 	buf[0] = (uint8_t)sub_addr;
        buf[1] = (uint8_t)((data >> 8) & 0xff);  // msb
        buf[2] = (uint8_t)(data & 0xff); 
       /* Send the command and data */
        enable_eeprom_write();
        clock_delay_usec(10);
        error = i2c_burst_send(eeprom_slave_addr,buf,len);
        if (error != OK)
        {
        printf("I2C EEPROM Error- 2\n");
                return error;
        }
        else
         {
          disable_eeprom_write();
          return OK;  /* Success */
         }
}



uint8_t eeprom_write_data_big(uint16_t sub_addr, uint32_t data, uint8_t len)
{
	uint8_t buf[len];
	uint8_t error=ERROR;
	if (sub_addr > EEPROM_END_ADDR)   // dummy write to adr 0 ... trigger measurement
        return error;
 
	uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
 	uint8_t page_addr= (uint8_t)(sub_addr>>8);
	eeprom_slave_addr |= page_addr;
 	buf[0] = (uint8_t)sub_addr;
        buf[1] = (uint8_t)((data >> 24) & 0xff);  // msb
        buf[2] = (uint8_t)((data >> 16) & 0xff);  // msb
        buf[3] = (uint8_t)((data >> 8) & 0xff);  // msb
        buf[4] = (uint8_t)(data & 0xff); 

       /* Send the command and data */
        enable_eeprom_write();
        clock_delay_usec(10);
        error = i2c_burst_send(eeprom_slave_addr,buf,len);
        if (error != OK)
        {
        printf("I2C EEPROM Error- 2\n");
                return error;
        }
        else
         {
          disable_eeprom_write();
          return OK;  /* Success */
        }
}

uint8_t eeprom_read_data( uint16_t sub_addr, uint16_t *data, uint8_t len)
{
        uint8_t error=ERROR;
        if (sub_addr > EEPROM_END_ADDR)
                return error; 
        
        uint8_t buf[len];
        uint8_t page_addr= (uint8_t)(sub_addr>>8);
        uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
        eeprom_slave_addr |= page_addr;
        
        buf[0] = (uint8_t)sub_addr;
        /* Read register */
        /* Send the read followed by address */
     	enable_eeprom_write();
    	clock_delay_usec(10);
        error = i2c_single_send(eeprom_slave_addr,buf[0]);
        if (error != OK)
        {
        printf("I2C EEPROM Error -3\n");
                return error;
        }
        clock_delay_usec(10);

        /* Receive a 2-byte result */
        error = i2c_burst_receive(eeprom_slave_addr, buf, len );
        if (error != OK)
          {
          printf("I2C EEPROM Error -4\n");
                return error;
          }
        /* Result */
        *data = buf[0]*256+buf[1];

        return OK;  /* Success */

}




uint8_t eeprom_read_data_big( uint16_t sub_addr, uint32_t *data, uint8_t len)
{
        uint8_t error=ERROR;
        if (sub_addr > EEPROM_END_ADDR)
                return error; 
        
        uint8_t buf[len];
        uint8_t page_addr= (uint8_t)(sub_addr>>8);
        uint8_t eeprom_slave_addr=EEPROM_SLAVE_ADDR;
        eeprom_slave_addr |= page_addr;
        
        buf[0] = (uint8_t)sub_addr;
        /* Read register */
        /* Send the read followed by address */
     	enable_eeprom_write();
    	clock_delay_usec(10);
        error = i2c_single_send(eeprom_slave_addr,buf[0]);
        if (error != OK)
        {
        printf("I2C EEPROM Error -3\n");
                return error;
        }
        clock_delay_usec(10);

        /* Receive a 2-byte result */
        error = i2c_burst_receive(eeprom_slave_addr, buf, len );
        if (error != OK)
          {
          printf("I2C EEPROM Error -4\n");
                return error;
          }
        /* Result */
        *data = buf[0]*16777216+buf[1]*65536+buf[2]*256+buf[3];
         
        return OK;

}

