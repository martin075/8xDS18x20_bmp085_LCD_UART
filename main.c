/*
* Program: Multitherm - A multiple DS18x20 thermometer control and logging device
* File Name: crc.c
* Purpose: Implements an AVR based temperature logging device
*
*	Reads multiple Dallas Semiconductor DS18x20 1-Wire
*	thermometers and sends the data via serial port for
*	logging by an external program.  Displays the
*	temperature data via an attached LCD as well.  Serial records
*	are timestamped using an elapsed time counter value, or a DS2415
*	1-Wire Real Time Clock timestamp if the device is present
*	on the 1-Wire bus.
*
*	Uses Peter Fleury's uart-library and lcd-library
*	which are very portable between AVRs.  Martin Thomas
*	added some functions in the uart-lib.
*
*	Contains code from DS18x20 Demo-Programm by
*	Martin Thomas <eversmith@heizung-thomas.de>
*	http://www.siwawi.arubi.uni-kl.de/avr-projects
*
*	Created using Atmel STK500, Atmega16L @ 8MHz
*	internal oscillator clock.
*
*    Programmer: Tom Blough
*   Host System: ATMega16L tested with both internal RC osc at 8MHz
*                and external 8MHz crystal
*  Date Created: 2007/09/20
*      Revision: 2007/12/18 - loop page display and sensor display
*                             fix bug in initial sensor order - was not writing to nvRam
*                2008/08/16 - added sanity checks when loading data from SRAM
*                2017/03     added saving adresses into EEPROM 
*				 2017/03 - upgraded libs UART.c, UART.h for avr-gcc 4.8.1 (Fleury 2015)
*				 2017/03 - reading data from BMP085 - temp, altitude, pressure
*                do not mix 18b20 family and 1820/18s20
*
* Host Configuration ATMega16:
*	PB0  LCD_DATA_4
*	PB1  LCD_DATA_5
*	PB2  LCD_DATA_6
*	PB3  LCD_DATA_7
*	PB4  LCD_DATA_RS
*	PB5  LCD_DATA_R/W
*	PB6  LCD_DATA_E
*
*	PD0  RxD, 19200,8,n,1
*	PD1  TxD
*	PD6  1-Wire Data, external 15K pullup
*	PD7  PWM LCD Backlight (OC2)
*
*	PB7  momentary MODE button
*	PD2  momentary UP/LEFT/FORWARD button
*	PD3  momentary DOWN/RIGHT/BACK button
*	PD4  momentary SELECTION button
*
* Host Configuration ATMega168, 8MHz:
*  note- on STK500, PB6 and PB7 are not routed to PortB header
*  instead they are routed to PortE XT1/XT2
*	PB0  LCD_DATA_4
*	PB1  LCD_DATA_5
*	PB2  LCD_DATA_6
*	PB3  LCD_DATA_7
*	PB4  LCD_DATA_RS
*	PB5  LCD_DATA_R/W
*	PB6  LCD_DATA_E
*
*	PD0  RxD, 19200,8,n,1
*	PD1  TxD
*	PD6  1-Wire Data, external 15K pullup
*	PD5  PWM LCD Backlight (OC2)
*
*	PB7  momentary MODE button
*	PD2  momentary UP/LEFT/FORWARD button
*	PD3  momentary DOWN/RIGHT/BACK button
*	PD4  momentary SELECTION button
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>

#include "uart.h"
#include "owi_highlevel.h"
#include "ds18x20.h"
#include "ds2415.h"
#include "lcd.h"
#include "delay.h"
#include "crc.h"
#include "version.h"

// libs for BMP085
#include "bmp085.h"
#include <math.h>

#define false 0
#define true !false

#define BAUD 19200
#define TMR_TICK 200L	// 200Hz = 5ms

#define MAX_SENSORS 8			// we can handle only sensor ID's from 0-9 (display constraint on order page)
#define MAX_INTERVAL 9999	// maximum number of seconds between log entries
#define MIN_INTERVAL 5		// minimum number of seconds between log entries
#define DEF_INTERVAL 15		//default interval of log entries, sec
#define HYSTER 10					// hysterezia 
#define DEF_BACKLIGHT 240	//default value of backlight
#define BUTTON_HELD 3			// number of CMD_DELAY cycles until button considered held
#define CMD_DELAY 200			// ms to wait between command executions

// operating states
#define MODE_LOGGING 0	// actively reading and logging temperatures
#define MODE_START 1	// displaying start logging message
#define MODE_ORDER 2	// displaying change view order
#define MODE_INTERVAL 3	// displaying interval change
#define MODE_SENSOR 4	// displaying extended sensor info
#define MODE_BACKLIGHT 5	// displaying PWM LCD backlight change
#define MODE_SHOW_DATE 6	// display RTC date
#define MODE_SET_YEAR 7	// change year value
#define MODE_SET_MONTH 8	// change month value
#define MODE_SET_DAY 9	// change day value
#define MODE_SET_HOUR 10	// change hour value
#define MODE_SET_MIN 11	// change minutes value
#define MODE_SET_SEC 12	// change seconds value
#define MODE_WRITE_DATE 13	// commit changes to RTC


// EEPROM storage of persistant variables
uint16_t EEMEM nvLog_interval = 10;	// default program load with 10 second timer
uint8_t EEMEM nvBacklight = DEF_BACKLIGHT;	// default program load to half-bright
uint8_t EEMEM nvNum_sensors;	// previous number of sensors
uint8_t EEMEM nv1Wire_order[MAX_SENSORS];	// previous display order
//uint8_t EEMEM nvgSensorIDsEE[MAX_SENSORS][OWI_ROMCODE_SIZE];
uint8_t EEMEM nvgSensorIDs1[OWI_ROMCODE_SIZE]; //out temp
uint8_t EEMEM nvgSensorIDs2[OWI_ROMCODE_SIZE]; //pumpwell temp
uint8_t EEMEM nvgSensorIDs3[OWI_ROMCODE_SIZE]; //cellar temp
uint8_t EEMEM nvgSensorIDs4[OWI_ROMCODE_SIZE]; //boiler temp
uint8_t EEMEM nvgSensorIDs5[OWI_ROMCODE_SIZE];
uint8_t EEMEM nvgSensorIDs6[OWI_ROMCODE_SIZE];
uint8_t EEMEM nvgSensorIDs7[OWI_ROMCODE_SIZE];
uint8_t EEMEM nvgSensorIDs8[OWI_ROMCODE_SIZE];

// buttons - checks if pressed and returns 0x01 or 0 if not pressed
#define BTN_MODE ((~PINB & (1 << PB7)) >> PB7)
#define BTN_UP ((~PIND & (1 << PD2)) >> PD2)
#define BTN_DOWN ((~PIND & (1 << PD3)) >> PD3)	
#define BTN_SEL ((~PIND & (1 << PD4)) >> PD4)	

// custom characters for LCD display
static const PROGMEM unsigned char lcdChars[] =
{
	0x04, 0x04, 0x04, 0x04, 0x15, 0x1F, 0x0E, 0x04,	// down arrow - chr(0)
	0x04, 0x0E, 0x1F, 0x15, 0x04, 0x04, 0x04, 0x04	// up arrow - chr(1)
};

// Module global variables
uint8_t gRTC_ID[OWI_ROMCODE_SIZE];	// id for DS2415 RTC if available
uint8_t gSensorIDs[MAX_SENSORS][OWI_ROMCODE_SIZE];	// active 1-wire thermometer sensors
uint8_t gRTC[OWI_ROMCODE_SIZE];	// active 1-wire RTC
uint8_t g1Wire_order[MAX_SENSORS];	// contains output ordering of 1-Wire thermometer devices
int gReadings[MAX_SENSORS];	// contains decicelsius readings for all thermometer sensors
int gReadings_old[MAX_SENSORS];	//previous reading temp
char gBuffer[21];	// used for itoa conversions and sprintf buffer
int gLog_interval;	// time period between log entries
uint8_t gBacklight;	// holds PWM value for LCD backlight brightness

//for bmp085
long l;
double d;
char printbuff[10];


//int *pom2; // pomocna premenna predchadzajuci odpocet teploty

// folowing are static because they are accessed within timer ISR
static uint8_t gRTC_status = 0;	// RTC status byte
static uint32_t gRTC_clk = 0;	// RTC epoch clock
static volatile uint16_t gElapsed;	// gElapsed counter for temperature reading period
//static volatile uint16_t min_counter; // counter for minutes

// forward declarations
uint8_t onewire_search( void);
void display_temperatures( uint8_t num_sensors, uint8_t page);
void log_temperatures( uint8_t num_sensors);
void read_temperatures( uint8_t num_sensors);
void show_id_uart( uint8_t *id);
void show_exinfo_lcd( uint8_t index);



/****************************************************************************************
 *                                                                                      *
 *           Main Entry Point                                                           *
 *                                                                                      *
 ***************************************************************************************/
int main( void)
{
	uint8_t i, num_sensors, device, page, mode, pos, tmp, up_held, dn_held;
	uint16_t bitmap;
//	DS2415_date_t dt;	// date structure for RTC

	// set unused I/O to input and enable pull-ups;
	DDRC = 0;	// all of port C is unused
	PORTC = 0x0f;
#if defined(__AVR_ATmega168__)
	DDRD &= ~(1 << PD7);
	PORTD |= (1 << PD7);
#elif defined(__AVR_ATmega16__)
	DDRD &= ~(1 << PD5);
	PORTD |= (1 << PD5);
#else
#error "Unknown target device"
#endif
	
	// enable our push buttons
	DDRB &= ~(1 << PB7);	// set for input - button 1
	PORTB |= (1 << PB7);	// enable pull-up
	DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4));	// buttons 2,3,4
	PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4);

	// set up PWM for LCD backlight control
#if defined(__AVR_ATmega168__)
#define PWM_OUT OCR0B
	DDRD |= _BV( PD5);	// OC0B pin as output
	TCCR0A = _BV( WGM00) | _BV( WGM01) | _BV( COM0B1);	// Fast PWM, CTC+TOP
	TCCR0B = _BV( CS01);	// 1/8 prescale
#elif defined(__AVR_ATmega16__)
#define PWM_OUT OCR2
#define TIMSK1 TIMSK
	DDRD |= _BV( PD7);	// OC2 pin as output
	TCCR2 = _BV( WGM20) | _BV( WGM21) | _BV( COM21) | _BV( CS21);	// fast PWM, CTC+TOP, 1/8 prescale
#else
#error "Unknown target device"
#endif
	PWM_OUT = gBacklight = eeprom_read_byte( &nvBacklight);	// grab our persistant data

	// set up the elapsed timer
	OCR1A = F_CPU / TMR_TICK;	// timer = 5 msec
	TCCR1B = _BV( WGM12) + _BV( CS10);	// clear timer on compare match, no prescaler

	sei();	// enable global interrupts
	
	// load userdefined characters from program memory
	// into LCD controller CG RAM
	lcd_init( LCD_DISP_ON);
	lcd_command( _BV( LCD_CGRAM));	/* set CG RAM start address 0 */
	for( i = 0; i < (sizeof lcdChars / sizeof lcdChars[0]); i++)
		lcd_data( pgm_read_byte_near( &lcdChars[i]));
	lcd_clrscr();

	// set up the comm port
	uart_init( UART_BAUD_SELECT( BAUD, F_CPU));
	
	// set up bmp085board
	bmp085_init();

	// announce our birth
	uart_puts_P( "DS1820,BMP180 Logger v" MAJOR_VER "." MINOR_VER "\r");
	lcd_gotoxy( 0, 0);
	lcd_puts_P( "DS18x20 Logger v" MAJOR_VER "." MINOR_VER "\r");
	//lcd_gotoxy( 0, 1);
	lcd_puts_P( "   BMP180 Logger");
	
	delay_ms( 1000);	// pause lcd display for a moment

	// get attached 1-wire sensor info
	num_sensors = onewire_search();
	if( num_sensors == 0 && ! gRTC_status) // bolo povodne
//	if( num_sensors == 0) 
	{
		lcd_gotoxy( 0, 2);
		lcd_puts_P( "No DS Sensors Found");
		uart_puts_P( "No Sensors Found");
	} 
	else
	{
		// display the count of sensors found on 1-Wire bus
		itoa( (int)num_sensors, gBuffer, 10);
		lcd_gotoxy( 0, 2);
		lcd_puts( gBuffer);
		lcd_puts_P( " Sensor(s)");
		uart_puts( gBuffer);
		uart_puts_P( " DS18X20 Sensor(s) available\r");
	}
	delay_ms( 2000);	// pause lcd display for a moment
	
	
	// initialize 1-wire output order from EEPROM.  make sure the order we read in
	// accounts for every sensor number by flipping the corresponding bit for each sensor
	// in the bitmap
	bitmap = 0;
	for( i = 0; i < num_sensors; i++)
		bitmap |= _BV( i);
		if( eeprom_read_byte( &nvNum_sensors) == num_sensors)
		{
		// assume if we have same number of sensors, we want to use the prev. order
		 eeprom_read_block( g1Wire_order, nv1Wire_order, MAX_SENSORS);

		// in the order we just loaded, check to see if all sensor
		// numbers were accounted for, by turning off bits in the bitmap
		 for( i = 0; i < num_sensors; i++)
			 bitmap &= ~_BV( g1Wire_order[i]);
		}

	if( bitmap != 0)	// were any sensors unaccounted for?
	{
		// yes, so load the default order
		eeprom_write_byte( &nvNum_sensors, num_sensors);
		for( i = 0; i < num_sensors; i++)
			g1Wire_order[i] = i;
		eeprom_write_block( g1Wire_order, nv1Wire_order, MAX_SENSORS);
	}

	// load and sanitize the reading interval from EEPROM
	gLog_interval = eeprom_read_word( &nvLog_interval);
	if( gLog_interval > MAX_INTERVAL || gLog_interval < MIN_INTERVAL)
	{
		gLog_interval = DEF_INTERVAL;  // bola min hodnota
		eeprom_write_word( &nvLog_interval, gLog_interval);	// save the persistant value
	}
		
	// don't worry about backlight since it's only 1 byte it WILL be between 0 and 255
		
		
	//					 setup for main loop*********************************************
	page = 0;
	lcd_clrscr();
	mode = MODE_START;	// first choice 
	TIMSK1 = _BV( OCIE1A);	// enable Output Compare 1 overflow interrupt to start elapsed timer

	// main loop - state machine
	while( true)
	{
		switch( mode)
		{
		case MODE_LOGGING:	// actively reading and logging temperatures***********
			{
				while( true)	// display loop
				{
					if( BTN_DOWN)	//vyhodnotenie stavu PINu - show previous 2 readings on LCD
					{
						// give a small delay to see if this is a simultaneous press
						delay_ms( CMD_DELAY);
						if( BTN_UP) break;	// exit display loop
						// cycle if needed
						if( page >= 4) page -= 2;          // if( page >= 2) page -= 2;
						else page = ((num_sensors - 1) >> 1) << 1; // bitovy posun - nasobenie
						lcd_clrscr();
						display_temperatures( num_sensors, page);
						while( BTN_DOWN);	// wait until switch released
					}
					else if( BTN_UP)	// show next 2-4 readings on LCD
					{
						// give a small delay to see if this is a simultaneous press
						delay_ms( CMD_DELAY);
						if( BTN_DOWN) break;	// exit display loop
						// cycle if needed
						if( page + 4 < num_sensors) page += 2; //boli 2 a 2
						else page = 0;
						lcd_clrscr();
						display_temperatures( num_sensors, page);
						while( BTN_UP);	// wait until switch released
					}
					else
					{
						// time for a new reading?
						if( gElapsed >= gLog_interval)
						{
							read_temperatures( num_sensors);
							log_temperatures( num_sensors);
							lcd_clrscr();
							display_temperatures( num_sensors, page);
							gElapsed = 0;	// restart the interval timer
						}
						// update the countdown.
						// else use the pseudo elapsed time counter
						lcd_gotoxy( 15, 0);
						sprintf( gBuffer, "%4is", gLog_interval - gElapsed);
						lcd_puts( gBuffer);
					}
				}
				mode = MODE_START;
				lcd_clrscr();	// provide feedback on button push
				while( BTN_UP || BTN_DOWN);	// wait until switches released
				break;	// next case
			}
		case MODE_START:	// displaying start logging message**********************
			{
				if( ! num_sensors > 0)
					{
					// no sensors found so skip all sensor related modes
					mode = MODE_BACKLIGHT;
					break;
					}
				lcd_clrscr();
				lcd_puts_P( "Start Logging?\n");
				lcd_putc( 0);	// down arrow
				lcd_puts_P( " or ");
				lcd_putc( 1);	// up arrow
				lcd_puts_P( " to start\n\n");
				lcd_puts_P( "press MODE to other");
				while( ! (BTN_MODE || BTN_UP || BTN_DOWN || BTN_SEL));	// wait for a button press
				// if mode button pressed, change to next state
				if( BTN_MODE)
					{
					lcd_clrscr();	// provide feedback on button push
					mode = MODE_ORDER;
					while( BTN_MODE);	// wait for release
					}
				else	// any other button, start logging
					{
						lcd_clrscr();	// provide feedback on button push
						mode = MODE_LOGGING;
						while( BTN_UP || BTN_DOWN || BTN_SEL);	// wait for release
								// send the extended sensor info out the comm port
						uart_puts_P( "\r\r");
						for( i = 0; i < num_sensors; i++)
							{
							device = g1Wire_order[i];
							uart_puts_P( "Sensor Display");
							uart_puti( (int)i);
							uart_puts_P( " / Bus");
							uart_puti( g1Wire_order[i]);
							uart_puts_P( " is a ");
							if( OWI_FAMILY( gSensorIDs[device][0]) == DS18S20_FAMILY_CODE)
									uart_puts_P( "DS18S20/DS1820");
							else uart_puts_P( "DS18B20");
							uart_puts_P( " = ");
							show_id_uart( &gSensorIDs[device][0]);
							uart_puts_P("\r");
							}
						uart_puts_P( "\r\r");
							// if no RTC clock present, reset the elapsed timer
						if( ! gRTC_status)
									gRTC_clk = 0;
						lcd_clrscr();
						read_temperatures( num_sensors);
						log_temperatures( num_sensors);
						display_temperatures( num_sensors, page);
						gElapsed = 0;	// restart the interval timer
					}
				break;	// next case MODE LOGGING
			}
		case MODE_ORDER:	// displaying change view order**************************
			{
				// this alters the Dallas 1Wire search order which appears random,
				// but is deterministic, to a more meaningful user selected order
				pos = 0;	// start on the left
				lcd_gotoxy( 1, 3);
				lcd_puts_P( "write to EEPROM");
				lcd_gotoxy( 0, 0);
				lcd_puts_P( "1Wire display order");
				lcd_gotoxy( 0, 1);
				for( i = 0; i < num_sensors; i++)	// display the 1-wire order
					lcd_putc( g1Wire_order[i] + '0');
				lcd_gotoxy( pos, 1);
				lcd_command( LCD_DISP_ON_BLINK);	// turn on flashing block cursor
				
				while( ! BTN_MODE)
				{
					if( BTN_SEL)	// next ID
					{
						// move the cursor position right one - recycle if at end
						pos++;
						if( pos > num_sensors - 1) pos = 0;
						lcd_gotoxy( pos, 1);

						while( BTN_SEL);	// wait for release
					}
					else if( BTN_UP)	// move selection to the left
					{
						if( pos > 0)
						{
							// swap marker positions
							tmp = g1Wire_order[pos];
							g1Wire_order[pos] = g1Wire_order[pos - 1];
							g1Wire_order[pos - 1] = tmp;
							pos--;
							// display the markers in the new order
							lcd_gotoxy( 0, 1);
							for( i = 0; i < num_sensors; i++)
								lcd_putc( g1Wire_order[i] + '0');
							lcd_gotoxy( pos, 1);
						}
						while( BTN_UP);	// wait until switch released
					}
					else if( BTN_DOWN)	// move selection to the right
					{
						if( pos < num_sensors - 1)
						{
							// swap marker positions
							tmp = g1Wire_order[pos];
							g1Wire_order[pos] = g1Wire_order[pos + 1];
							g1Wire_order[pos + 1] = tmp;
							pos++;
							// display the markers in the new order
							lcd_gotoxy( 0, 1);
							for( i = 0; i < num_sensors; i++)
								lcd_putc( g1Wire_order[i] + '0');
							lcd_gotoxy( pos, 1);
						}
						while( BTN_DOWN);	// wait until switch released
					}
				}
				eeprom_write_block( g1Wire_order, nv1Wire_order, MAX_SENSORS);
				
				if(num_sensors==1)
						{ nvgSensorIDs1[1] = gSensorIDs[1][0];
							eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						}
				if(num_sensors==2)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						}
				if(num_sensors==3)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						}
				if(num_sensors==4)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						nvgSensorIDs4[1] = gSensorIDs[4][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs4, 8);
						}
			/*	if(num_sensors==5)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						nvgSensorIDs4[1] = gSensorIDs[4][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs4, 8);
						nvgSensorIDs5[1] = gSensorIDs[5][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs5, 8);
						}
				if(num_sensors==6)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						nvgSensorIDs4[1] = gSensorIDs[4][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs4, 8);
						nvgSensorIDs5[1] = gSensorIDs[5][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs5, 8);
						nvgSensorIDs6[1] = gSensorIDs[6][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs6, 8);
						}
				if(num_sensors==7)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						nvgSensorIDs4[1] = gSensorIDs[4][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs4, 8);
						nvgSensorIDs5[1] = gSensorIDs[5][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs5, 8);
						nvgSensorIDs6[1] = gSensorIDs[6][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs6, 8);
						nvgSensorIDs7[1] = gSensorIDs[7][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs7, 8);
						}
				if(num_sensors>=8)
						{nvgSensorIDs1[1] = gSensorIDs[1][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs1, 8);
						nvgSensorIDs2[1] = gSensorIDs[2][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs2, 8);
						nvgSensorIDs3[1] = gSensorIDs[3][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs3, 8);
						nvgSensorIDs4[1] = gSensorIDs[4][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs4, 8);
						nvgSensorIDs5[1] = gSensorIDs[5][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs5, 8);
						nvgSensorIDs6[1] = gSensorIDs[6][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs6, 8);
						nvgSensorIDs7[1] = gSensorIDs[7][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs7, 8);
						nvgSensorIDs8[1] = gSensorIDs[8][0];
						eeprom_write_block( gSensorIDs, nvgSensorIDs8, 8);
						}
			*/	
				lcd_command( LCD_DISP_ON);	// turn off flashing cursor
				lcd_clrscr();	// acknowledge button press
				mode = MODE_INTERVAL;
				while( BTN_MODE);	// wait until switch released
				break;	// next case
			}
		case MODE_INTERVAL:	// displaying logging interval change******************
			{
				lcd_clrscr();
				lcd_puts_P( "Logging interval\n");
				sprintf( gBuffer, "%4i", gLog_interval);
				lcd_puts( gBuffer);
				//lcd_puts_P( "Logging interval\n");

				while( ! BTN_MODE)
				{
					/* since the interval range is from MIN_INTERVAL to 9999
					 * we want to be able to move faster than a few times per
					 * second.  we will wait a small period of time, and then
					 * if the button is still held, geometrically increase
					 * the step amount each loop through until the button
					 * is released
					 */
					up_held = dn_held = 0;
					while( BTN_UP)	// increase interval
					{
						if( gLog_interval < MAX_INTERVAL)
						{
							// geometrically increase step size if button held
							if( up_held >= BUTTON_HELD)
								gLog_interval += up_held << 2;
							else
								gLog_interval++;
							
							// fixup on overflow
							if( gLog_interval > MAX_INTERVAL) gLog_interval = MAX_INTERVAL;

							lcd_gotoxy( 0, 1);
							sprintf( gBuffer, "%4i", gLog_interval);
							lcd_puts( gBuffer);
						}
						delay_ms( CMD_DELAY);	// allow time to release button
						up_held++;
					}

					while( BTN_DOWN)	// decrease interval
					{
						if( gLog_interval > MIN_INTERVAL)
						{
							if( dn_held >= BUTTON_HELD)
							{
								// geometrically increase step size if button held
								// need to worry about underflow since interval is unsigned
								if( (gLog_interval - MIN_INTERVAL) > dn_held << 2)
									gLog_interval -= dn_held << 2;
								else
									gLog_interval = MIN_INTERVAL;
							}
							else
								gLog_interval--;
							lcd_gotoxy( 0,1);
							sprintf( gBuffer, "%4i", gLog_interval);
							lcd_puts( gBuffer);
						}
						delay_ms( CMD_DELAY);
						dn_held++;
					}
				}
				mode = MODE_SENSOR;
				eeprom_write_word( &nvLog_interval, gLog_interval);	// save the persistant value
				lcd_clrscr();	// ack button press
				while( BTN_MODE);	// wait until switch released
				break;	// next case
			}
		case MODE_SENSOR:	// displaying extended sensor info***********************
			{
				i = 0;	// display order
				show_exinfo_lcd( i);

				while( ! BTN_MODE)	// loop until mode button pressed
				{
					/* continously display the updated sensor count
					 * this takes appx 3/4 of a second so it sometimes
					 * makes the buttons appear unresponsive.  the only
					 * way I can think of to fix this is to handle the
					 * buttons with an ISR, but this adds a level of
					 * complexity I don't want
					 */
					device = g1Wire_order[i];
					lcd_gotoxy( 10, 0);
					DS18X20_start_meas( &gSensorIDs[device][0]);
					sprintf( gBuffer, "0x%04x", DS18X20_get_meas( &gSensorIDs[device][0]));
					lcd_puts( gBuffer);

					if( BTN_UP)	// display next
					{
						// cycle if needed
						if( i < num_sensors - 1) i++;
						else i = 0;

						show_exinfo_lcd( i);
						while( BTN_UP);	// wait until switch released
					}
					else if( BTN_DOWN)	// display previous
					{
						// cycle if needed
						if( i > 0) i--;
						else i = num_sensors - 1;
						show_exinfo_lcd( i);
						while( BTN_DOWN);	// wait until switch released
					}
				}
				lcd_clrscr();	// acknowledge the mode button press
				while( BTN_MODE);	// wait until switch released
				mode = MODE_BACKLIGHT;
				break;	// next case
			}
		case MODE_BACKLIGHT:	// displaying interval change   *********************
			{
				lcd_clrscr();
				lcd_puts_P( "LCD brightness\n");
				sprintf( gBuffer, "%4i", gBacklight);
				lcd_puts( gBuffer);

				// change PWM value to modify on-time of LED backlight
				while( ! BTN_MODE)
				{
					while( BTN_UP)	// increase interval
					{
						if( gBacklight < 255)
						{
							PWM_OUT = ++gBacklight;
							lcd_gotoxy( 0,1);
							sprintf( gBuffer, "%4i", gBacklight);
							lcd_puts( gBuffer);
						}
						// allow time to release button but still have fast cycle rate
						delay_ms( 25);
					}
					while( BTN_DOWN)	// decrease interval
					{
						if( gBacklight > 0)
						{
							PWM_OUT = --gBacklight;
							lcd_gotoxy( 0,1);
							sprintf( gBuffer, "%4i", gBacklight);
							lcd_puts( gBuffer);
						}
						// allow time to release button but still have fast cycle rate
						delay_ms( 25);
					}
				}
				//mode = MODE_SHOW_DATE;
				eeprom_write_byte( &nvBacklight, gBacklight);	// save the persistant value
				lcd_clrscr();	// ack button press
				while( BTN_MODE);	// wait until switch released
				mode = MODE_START;  //odkaz na zaciatok
				break;	// next case
			}
				//while( BTN_MODE);	// wait until switch released
				//break;	// next case
		}     //switch mode
	}    //while
} // main

/****************************************************************************************
 *                                                                                      *
 *           Helper Functions                                                           *
 *                                                                                      *
 ***************************************************************************************/

// service routine should occur every 1/TMR_TICK seconds.
ISR( TIMER1_COMPA_vect)
{
	static uint8_t second_counter;	// second_counter == TMR_TICK = 1 second
//static uint8_t tcounter = 0; // pocita interval
	// increment the counters
	if( ++second_counter >= TMR_TICK)	// one second elapsed?
	{
		gElapsed++;
		second_counter = 0;
		/* if no RTC installed, increment it's counter.  this will act
		 as a transparent pseudo elapsed seconds counter.  The timestamp
		 routines will still work, they will just display seconds elapsed
		 and think the date is January 1, 1970.  if the RTC is installed,
		 then the counter will be set by reading the RTC in the display 
		 loop and before temp readings are taken.
		*/
		//tcounter++;
		//if (tcounter >= 60)
		//	{tcounter = 0; // nulovanie pocitadla po 1 min
		//	min_counter++;
		//	}
		if( gRTC_status == 0) gRTC_clk++;
	}
}
/********************************************************/
// search 1-wire bus for active devices
uint8_t onewire_search( void)
{
	uint8_t i, diff, num_sensors;
	uint8_t id[OWI_ROMCODE_SIZE];
	
	num_sensors = 0;
	
	for( diff = OWI_SEARCH_FIRST; 
			diff != OWI_LAST_DEVICE && num_sensors < MAX_SENSORS ;)
	{
		diff = owi_search( diff, &id[0], OWI_SEARCH_ROM);	// search all sensors
		
		if( diff == OWI_PRESENCE_ERROR)
		{
			lcd_gotoxy(0,1);
			lcd_puts_P( "No ow Sensors found");
			uart_puts_P( "No ow Sensors found\r"); 	
			break;
		}
		else if( diff == OWI_DATA_ERR)
		{
			lcd_gotoxy(0,1);
			lcd_puts_P( "Bus Error");
			uart_puts_P( "Bus Error\r"); 	
			break;
		}
		else if( ! crc8( id, OWI_ROMCODE_SIZE))	// is it a valid id?
		{
			if( OWI_FAMILY( id[0]) == DS18B20_FAMILY_CODE || OWI_FAMILY( id[0]) == DS18S20_FAMILY_CODE)
			{
				for( i = 0; i < OWI_ROMCODE_SIZE; i++)
					gSensorIDs[num_sensors][i] = id[i];
				num_sensors++;
			}
			else if( OWI_FAMILY( id[0]) == DS2415_FAMILY_CODE)
			{
				for( i = 0; i < OWI_ROMCODE_SIZE; i++)
					gRTC_ID[i] = id[i];

				// make sure the RTC oscillator is turned on
				if( DS2415_read_clock( &gRTC_status, &gRTC_clk, id) == DS2415_CLOCK_OK)
				{
					DS2415_write_clock( DS2415_OSC_ON( gRTC_status), gRTC_clk, id);
				}
			}
		}
		else
		{
			// id failed CRC check
			uart_puts_P( "CRC failed - ignoring sensor: "); 	
			for( i = 0; i < OWI_ROMCODE_SIZE; i++)
				uart_puthex_byte( id[i]);
			uart_puts_P( "\r"); 	
		}
	}
	return num_sensors;
}
/************************************************************/
// display one page of most recent temperature data on lcd
void display_temperatures( uint8_t num_sensors, uint8_t page)
{
	uint8_t device;
	int *pom1 = &gReadings[0];
	int *pom2 = &gReadings_old[0];
	//int *pom1;
	// should always have at least one sensor on a page
	//temperature = ( gReadings[device] / 10 );
	lcd_home();
	device = g1Wire_order[page];
	//*pom1 = &gReadings[0];
	//podmienka so zobrazenim sipky hore/dole/rovno -kod podla displeja C5hore/C6dole/C7rovno, pre vonkajsiu teplotu
			if( ( ( pom1[0] ) < ( pom2[0]) ) && ((pom2[0]-pom1[0]) > HYSTER) )
				{sprintf( gBuffer, "%i#Out %+04i.%c\xC6\n",(int)page, gReadings[device] / 10,(gReadings[device] % 10) + '0');
				lcd_puts( gBuffer);
				}
			else if( ( ( pom1[0] ) > ( pom2[0]) ) && ((pom1[0]-pom2[0]) > HYSTER) )
				{sprintf( gBuffer, "%i#Out %+04i.%c\xC5\n",(int)page, gReadings[device] / 10,(gReadings[device] % 10) + '0');
				lcd_puts( gBuffer);
				}
			else
			{sprintf( gBuffer, "%i#Out %+04i.%c\xC7\n",(int)page, gReadings[device] / 10,(gReadings[device] % 10) + '0');
			lcd_puts( gBuffer);
			}
			pom2[0] = pom1[0];
			
			//min_counter = 0;
		
			// is there a second sensor to display?
		if( (page + 1) < num_sensors)
		{
				device = g1Wire_order[page + 1];
				sprintf( gBuffer, "%i#PmpW%+04i.%c\n",(int)page + 1, gReadings[device] / 10,(gReadings[device] % 10) + '0'); // pumpwell
				lcd_puts( gBuffer);
				if( (page + 2) < num_sensors)
	      	 {
	 	      	 device = g1Wire_order[page + 2];
	 	       	sprintf( gBuffer, "%i#Cllr%+04i.%c\n",(int)page + 2, gReadings[device] / 10,(gReadings[device] % 10) + '0'); //cellar
           	lcd_puts( gBuffer);
            if( (page + 3) < num_sensors)
	        	   {
	 	        	  device = g1Wire_order[page + 3];
	 	          	sprintf( gBuffer, "%i#Bler%+04i.%c\n",(int)page + 3, gReadings[device] / 10,(gReadings[device] % 10) + '0'); //boiler
               	lcd_puts( gBuffer);
	           		}
             	else lcd_puts_P( "            ");
	       		}
         	else lcd_puts_P( "            ");
      }
		else lcd_puts_P( "            ");
		{
		//get temperature
		d = bmp085_gettemperature();
		dtostrf(d, 10, 2, printbuff);
		uart_puts("temperature: "); 
		uart_puts(printbuff);  
		uart_puts(" C deg"); 
		uart_puts("\r\n");

		//get pressure
		l = bmp085_getpressure();
		ltoa(l, printbuff, 10); 
		uart_puts("rel.pressure:   "); 
		uart_puts(printbuff);
		uart_puts(" Pa");
		uart_puts("\r\n");
		lcd_gotoxy( 17, 2);
		lcd_puts_P( "hPa");
		lcd_gotoxy( 13, 2);
		ltoa((l/100) , printbuff, 10); // in hPa (/100) in relative pressure to sea level, bmp085.h
		lcd_puts( printbuff);

		//get altitude
		d = bmp085_getaltitude();
		dtostrf(d, 10, 2, printbuff);
		uart_puts("altitude   : ");
		uart_puts(printbuff);
		uart_puts(" M above sea");
		uart_puts("\r\n");
		//lcd_gotoxy( 19, 1);
		//lcd_puts_P( "m");
		//lcd_gotoxy( 15, 1);
		//lcd_puts( printbuff);
		//uart_puts("\r\n");
		delay_ms(500);
		}
}
/**************************************************************/
// output most recent temperatures to comm port
void log_temperatures( uint8_t num_sensors)
{
	uint8_t i, device;
//	DS2415_date_t dt;

	// output the timestamp at the start of the record
	//DS2415_epoch_to_gregorian( gRTC_clk, &dt);
	uart_putc( '#');	// marker for data record to help parsing via external program
	//if( gRTC_status)	// RTC installed; show yyyy/mm/dd
		//sprintf( gBuffer, "%04i/%02i/%02i", (int)dt.year + 1900, (int)dt.month, (int)dt.day);
	//else	// no RTC so show elapsed days
		//sprintf( gBuffer, "%04id", (int)(gRTC_clk / 24 / 60 / 60));
	//uart_puts( gBuffer);

	//sprintf( gBuffer, " %02i:%02i:%02i;", (int)dt.hour, (int)dt.min, (int)dt.sec);
	//uart_puts( gBuffer);
	
	// output a semicolon delimited list of the sensors data
	for( i = 0; i < num_sensors; i++)
	{
		device = g1Wire_order[i];   //premenna, zoradene snimace
		sprintf( gBuffer, "%+04i.%c;", gReadings[device] / 10, (gReadings[device] % 10) + '0');
		uart_puts( gBuffer);
	}
	uart_puts_P( "\r");      //vycital vsetky za sebou a poslal znak novy riadok
}
/**********************************************************/
// read all DS18x20 sensors on 1-wire buss, 12-bit resolution
void read_temperatures( uint8_t num_sensors)
{
	uint8_t i;
	
	if( DS18X20_start_meas( NULL) == OWI_BUS_OK)
	{
		//if( gRTC_status) DS2415_read_clock( &gRTC_status, &gRTC_clk, gRTC_ID);
		for( i = 0; i < num_sensors; i++)
			gReadings[i] = DS18X20_get_temp( &gSensorIDs[i][0]);
	}
	else
	{
		for( i = 0; i < num_sensors; i++)
			gReadings[i] = 0;
		uart_puts_P( "Start meas. fail\r"); 	
	}
}
/***************************************************/
// display the sensor ID info on the serial port
void show_id_uart( uint8_t *id)
{
	size_t i;
	for( i = 0; i < OWI_ROMCODE_SIZE; i++)
	{
		if( i == 0) uart_puts_P( "FC:");
		else if( i == OWI_ROMCODE_SIZE - 1) uart_puts_P( " CRC:");
		if( i == 1) uart_puts_P( " SN:");
		uart_puthex_byte( id[i]);
		if( i == 0)
		{
			if( OWI_FAMILY( id[0]) == DS18S20_FAMILY_CODE) uart_puts_P( "(18S)");
			else if( OWI_FAMILY( id[0]) == DS18B20_FAMILY_CODE) uart_puts_P( "(18B)");
			else uart_puts_P( "(?)");
		}
	}
}
/*****************************************************/
// condense the extended 1Wire device info into
// two 16-character lines and display on the LCD
void show_exinfo_lcd( uint8_t index)
{
	uint8_t device, i;

	// show the displayed order and the identified order
	device = g1Wire_order[index];
	lcd_gotoxy( 0, 0);
	lcd_puts_P( "Dsp");
	lcd_putc( index + '0');
	lcd_puts_P( "/Bus");
	lcd_putc( device + '0');

	lcd_puts_P( "       ");//	 clear any previously displayed measurement data

	// show the device information on next line
	lcd_gotoxy( 0, 1);
	if( OWI_FAMILY( gSensorIDs[device][0]) == DS18S20_FAMILY_CODE)
		lcd_puts_P( "18S:");
	else if( OWI_FAMILY( gSensorIDs[device][0]) == DS18B20_FAMILY_CODE)
		lcd_puts_P( "18B:");
	else
	{
		 // unknown device type
		 lcd_puthex_byte( gSensorIDs[device][0]);
		 lcd_puts_P( "?:");
	}

	// show the serial number and CRC byte
	for( i = 1; i < OWI_ROMCODE_SIZE - 1; i++)
		lcd_puthex_byte( gSensorIDs[device][i]);
}
