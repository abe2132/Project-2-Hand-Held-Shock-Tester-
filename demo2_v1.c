/*
 * EEPROPM_Program.c
 *
 * Created: 5/3/2020 12:35:41 PM
 * Author : IB
 */ 
#define F_CPU 16e6
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "i2cmaster.h"
#include "usart.h"
#include "lcd.h"
#include "mma8451_pi.h"
#include "mma8451_pi/mma8451_pi.h"



#define ADC_ref 5.0
#define zero_x 1.575
#define zero_y 1.575
#define zero_z 1.575
#define sensitivity_x 0.315
#define sensitivity_y 0.315
#define sensitivity_z 0.315






void init_ADC();
uint16_t ADC_read(uint8_t channel);
void init_MPU();

uint16_t Digital_read();

int main(void)
{
 		 
			uart_init();
			io_redirect();
			i2c_init();
			init_ADC();
			LCD_init();
			
		 mma8451 sensor = mma8451_initialise(1, 0x1D);
		 mma8451_vector3 acceleration;
			
			uint16_t DataX;
			uint16_t DataY;
			uint16_t DataZ;
			float X;
			float Y;
			float Z;
		
   

 
    while (1) 
    {
		
		
		
		DataX = ADC_read(0);
		DataY = ADC_read(1);
		DataZ = ADC_read(2);
		
		
		
		
		
		X =((DataX/1024.0*ADC_ref)-zero_x)/sensitivity_x;
		LCD_set_cursor(8,0);
		printf("x value: %1.2f", X);

		Y =((DataY/1024.0*ADC_ref)-zero_y)/sensitivity_y;
		LCD_set_cursor(8,1);
		printf("y value: %1.2f", Y);
	
		Z =((DataZ/1024.0*ADC_ref)-zero_z)/sensitivity_z;
		LCD_set_cursor(8,2);
		printf("z value: %1.2f", Z);
	
		_delay_ms(500);
		// for digital accelerometer
		 mma8451_get_acceleration(&sensor, &acceleration);
		 printf("acceleration = (%f, %f, %f)\n"), acceleration.x, acceleration.y, acceleration.z);
		
		
		
		
		// X axis acceleration
		LCD_set_cursor(0,0);
		printf("X:");
		LCD_set_cursor(2,0);
		printf(acceleration.x);
		// Y axis acceleration
		LCD_set_cursor(0,1);
		printf("Y:");
		LCD_set_cursor(2,1);
		printf(acceleration.y);
		// Z axis acceleration
		LCD_set_cursor(0,2);
		printf("Z:");
		LCD_set_cursor(2,2);
		printf(acceleration.z);
		
	}
}




















void init_ADC(void)
{
	
	
	ADMUX |=(1<<REFS0); //Voltage ref from AVcc
	ADCSRA|=(1<<ADEN);  // Turn on ADC
	ADCSRA|=(1<<ADSC);  // Start the conversion
	ADCSRA|=((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));  // 16 MHz/128 =125Khz the ADC reference clock.
	
	
}


uint16_t ADC_read(uint8_t channel)

{
	
	ADMUX &=0xF0; //Clear the older channel that was read
	ADMUX|= channel;  // Defines the new ADC channel to be read
	ADCSRA|=(1<<ADSC); // Start a new conversion (single conversion)
	
	while (ADCSRA & (1<<ADSC));  //Wait until the conversion is done
	return ADC;                  // Returns the ADC value of the chosen channel.
	
}