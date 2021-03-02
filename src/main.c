#include "gd32vf103.h"
#include "gd32vf103_eclic.h"
#include "gd32vf103_gpio.h"
#include "gd32vf103_i2c.h"
#include "gd32vf103_rcu.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>

#include "drv_usb_core.h"
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include "systick.h"
#include "usbd_core.h"
#include "usbd_conf.h"
//#include "mpu6050.h" //had a lot of useful register info

extern uint8_t packet_sent, packet_receive;
extern uint32_t receive_length;
extern uint8_t usb_data_buffer[CDC_ACM_DATA_PACKET_SIZE];
void debug(const char *format, ...);

//This is written using I2C0 only - however the code can be recycled by
//adding a vairable named i2c_num to the functions or redefining it here.
#define i2c_num I2C0
#define addr_MPU6050 0x68 //address is 0b110100X
#define addr_GD32 0x72 //own address, not really needed here
#define SCL GPIO_PIN_6
#define SDA GPIO_PIN_7 

usb_core_driver USB_OTG_dev = {
	.dev = {
	.desc = {
		.dev_desc	= (uint8_t *)&device_descriptor,
		.config_desc	= (uint8_t *)&configuration_descriptor,
		.strings	= usbd_strings,
	}
	}
};

typedef struct _full {
	int16_t aX;
	int16_t aY;
	int16_t aZ;
	uint16_t T;
	int16_t gX;
	int16_t gY;
	int16_t gZ;
} reading_t;

// Wait for data, then read
uint8_t read()
{
	while(!i2c_flag_get(i2c_num, I2C_FLAG_RBNE));
	return i2c_data_receive(i2c_num);
}


// Send START condition, then send address.
void send_add(uint8_t rw)
{
	i2c_start_on_bus(i2c_num);
	while(!i2c_flag_get(i2c_num, I2C_FLAG_SBSEND));
	//***DO NOT CLEAR THIS*** i2c_flag_clear(i2c_num, I2C_FLAG_SBSEND);
	
	if (rw)
		i2c_master_addressing(i2c_num, addr_MPU6050<<1, I2C_RECEIVER);
	else 
		i2c_master_addressing(i2c_num, addr_MPU6050<<1, I2C_TRANSMITTER);

	while(!i2c_flag_get(i2c_num, I2C_FLAG_ADDSEND));
	i2c_flag_clear(i2c_num, I2C_FLAG_ADDSEND); 

}

// Write [length] bytes of data, then send STOP condition.
void write(uint8_t *data, uint8_t length)
{
	//send address with write bit
	send_add(0); 

	//precaution, probably unnecessary
	while(!i2c_flag_get(i2c_num, I2C_FLAG_TBE)); 

	for (int i = 0; i<length;  i++) {

		i2c_data_transmit(i2c_num, data[i]); 
		while(!i2c_flag_get(i2c_num, I2C_FLAG_TBE));
	}

	i2c_stop_on_bus(i2c_num);
}

// Attempt at usb print function
void debug(const char *format, ...) 
{
	va_list arg; 
	va_start(arg, format);
	vsnprintf((char *)usb_data_buffer, CDC_ACM_DATA_PACKET_SIZE, format, arg);
	va_end(arg);

	if (USB_OTG_dev.dev.cur_status == USBD_CONFIGURED) {
		cdc_acm_data_send(&USB_OTG_dev, strlen((char *)usb_data_buffer));
		// printing too quickly can **sometimes** crash this 
		//delay_1ms(10);
	}
}

/* Read N bytes into my_buffer and send NACK and stop.
 *
 * Signal structure:
 * S AD+W     RA     S AD+R             ACK ...         NACK P
 * 	  ACK	 ACK        ACK DATA(1)     ... DATA(N)
 */
void full_reading (int16_t *my_buffer, uint8_t N, uint8_t read_addr) 
{
	//Send RA, and wait for ACK
	write(&read_addr, 1);
	
	// Send address with read bit, then read 2N-2 bytes
	send_add(1);
	for (int i=0; i <= N-2; i++) {
		my_buffer[i]=read()<<8|read();
	}

	// read N-1 byte
	my_buffer[N-1]=read()<<2;

	// disable ACK
	i2c_ack_config(i2c_num, I2C_ACK_DISABLE);

	// read last byte, without sending ACK
	my_buffer[N-1]+=read();

	i2c_stop_on_bus(i2c_num);

	// remember to re-enable this
	i2c_ack_config(i2c_num, I2C_ACK_ENABLE);
}

// Scan for I2C responses.
void scanI2C(void) {

	// For some reason this stops at 0xf8 ???
	for (uint8_t i = 0x00; i<0xff; i++) {
		debug("Trying: 0x%x\n", i);
		i2c_start_on_bus(i2c_num);
		while(!i2c_flag_get(i2c_num, I2C_FLAG_SBSEND));
		i2c_master_addressing(i2c_num, i, I2C_TRANSMITTER);
		//seems some delat is needed
		delay_1ms(50);
		if (i2c_flag_get(i2c_num, I2C_FLAG_ADDSEND))
			debug("Found device on 0x%x!\n", i);
		i2c_flag_clear(i2c_num, I2C_FLAG_ADDSEND);
		i2c_flag_clear(i2c_num, I2C_FLAG_SBSEND);
	}
}

// Set up the USB 
void setup_usb (void)
{
	eclic_global_interrupt_enable();
	eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL2_PRIO2);

	usb_rcu_config();
	usb_timer_init();
	usb_intr_config();

	usbd_init(&USB_OTG_dev, USB_CORE_ENUM_FS, &usbd_cdc_cb);

	/* check if USB device is enumerated successfully */
	while (USBD_CONFIGURED != USB_OTG_dev.dev.cur_status);
}


// Set up the I2C
void setup_i2c(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_I2C0);

	// Use pins b6 and b7 as SCL0 and SDA0 respectively.
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

	//reset
	i2c_deinit(i2c_num);

	i2c_clock_config(i2c_num, 400000, I2C_DTCY_2);
	//i2c_mode_addr_config(i2c_num, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, addr_GD32);

	i2c_enable(i2c_num);
	i2c_ack_config(i2c_num, I2C_ACK_ENABLE);

	//wait
	while(i2c_flag_get(i2c_num, I2C_FLAG_I2CBSY));
}

// Set up the MPU6050
void setup_mpu() {

	uint8_t buff[2];
	//Reset
	buff[0] = 0x6b;
	buff[1] = 0x00;
	write(buff, 2);

	//Digital low pass filter
	buff[0] = 0x1a;
	buff[1] = 0x04;
	write(buff, 2);
}

int main(void)
{
	uint8_t reg_addr = 0x3B;
	reading_t reading;

	setup_i2c();
	setup_usb();
	setup_mpu();

	delay_1ms(3000);
	debug("Setup ok.\n");

	while (1){

		full_reading((int16_t *)&reading, sizeof(reading)/2, reg_addr);
		
		debug("Ax: %d\tAy:%d\tAz:%d\tT:%d\tGx:%d\tGy:%d\tGz:%d\n", 
			reading.aX, reading.aY, reading.aZ, (int16_t)reading.T,
			reading.gX, reading.gY, reading.gZ);

		delay_1ms(20);
	}
	return 0;
}
