/*	Data structure
	HIGH 1SSFFFDD
	LOW  0DDDDDDD
*/ 
#define TRUE 1
#define FALSE 0
#define F_CPU 20000000UL
#define BAUD  57600
#define MYUBBR F_CPU/16/BAUD-1
#define NUMBER_OF_ATTEMPT 100
#define BUFFER_SIZE 256
#define HOSTMSG_BUFFER_SIZE 24
#define PACKET_LENGTH 25

// data headers
#define LEFT_HAND 0x80
#define RIGHT_HAND 0x81
#define PEDAL 0x82
// system call headers
#define PRESET_CHANGE 0x90
#define PING 0x91
#define QUERY 0x92
// message from the host
#define HOST_PRESET_CHANGE 0xA0 
#define HOST_PING 0xA1
#define HOST_RESPONSE 0xA2
#define HOST_PRESET_CHANGE_CONFIRM 0xA3
// system halt
#define SYSTEM_HALT 0xF0
// EOT (End Of Transfer)
#define EOT 0xFF

//XBEE
#define XBEE_START_DELIMETAR 0x7E

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "suart.h"
#include "lcd.h"

/* global variables */
volatile uint8_t buffer0[BUFFER_SIZE];
volatile uint8_t packet0[PACKET_LENGTH];
volatile uint16_t counter0_write, counter0_read;
volatile uint16_t packetCounter0;

volatile uint8_t buffer1[BUFFER_SIZE];
volatile uint8_t packet1[PACKET_LENGTH];
volatile uint16_t counter1_write, counter1_read;
volatile uint16_t packetCounter1;

volatile uint8_t hostBuffer[HOSTMSG_BUFFER_SIZE];
volatile uint8_t counterh_write;

volatile uint8_t preset, bank, proposedPreset, proposedBank;
volatile uint8_t buttonState, updateFlag, proposalFlag;
volatile uint8_t proposalCount;

/* prototypes */
void timer1Init(void);
void timer1Start(void);
void timer1Stop(void);
void uartInit(unsigned int baud);
void uartSendByte(char byte);
void adcInit(void);
uint8_t adcRead(uint8_t ch);
void buttonInit(void);
void ledInit(void);
void post(char* msg, uint8_t row, uint8_t clear);
void notifyUpdate(void);
void waitForSynthesizer(void);
uint8_t checkSum(volatile uint8_t *buffer);

void timer1Init(void){
	TIMSK1 = 1 << OCIE1A; // out put compare match A interrupt enable for counter 0
	TCCR1A = 0x00; // normal mode
	TCCR1B = 0x00; // clk / 1024 prescaling
	OCR1AH = 200; // ca' 2 second
	OCR1AL = 0;
	TCNT1H = 0; // reset timer
	TCNT1L = 0;
}

void timer1Start(void){
	TCNT1H = 0;
	TCNT1L = 0;
	TCCR1B = 0x05; // clk / 1024
}

void timer1Stop(void){
	TCCR1B = 0x00;
}

void uartInit(unsigned int baud){
	UBRR0H = (unsigned char)(baud >> 8);
	UBRR0L = (unsigned char)baud; 
	UCSR0B = (1 << RXEN0 ) | (1 << TXEN0) | 1 << (RXCIE0);
	UCSR0C = (3 << UCSZ00);
    UBRR1H = (unsigned char)(baud >> 8);
	UBRR1L = (unsigned char)baud;
	UCSR1B = (1 << RXEN1 ) | ( 1 << RXCIE1 );
}

void uartSendByte(char byte){
	UDR0 = byte;
	loop_until_bit_is_set(UCSR0A, UDRE0);
}

void adcInit(void){
	ADMUX |= (1 << REFS0) | ( 1 << ADLAR ); // AVcc as reference - only 8 bit needed
	ADCSRA |= (1 << ADEN) |(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // divided by 128
}

uint8_t adcRead(uint8_t ch)
{
	ADMUX = (ADMUX & 0xF8)|ch; 
	ADCSRA |= (1<<ADSC); // start conversion
	while(ADCSRA & (1<<ADSC));  // wait for conversion
	return (ADCH); //only upper 8 bit
}

void buttonInit(void){
	DDRB &= ~((1 << PORTB2) | (1 << PORTB3));
	PORTB |= (1 << PORTB2) | (1 << PORTB3);
	buttonState = 0x03; // both state high as default
	PCICR |= (1<<PCIE1); // enable interrupt on PCIN 15 ... 8
	PCMSK1 |= (1<< PCINT10) | (1<< PCINT11); // enable PCINT on both ports
}

void globalInit(void){
	updateFlag = 0;
	preset = 0;
	bank = 0;
	proposedBank = 0;
	proposedPreset = 0;
	proposalFlag = FALSE;
	proposalCount = 0;
	
	counter0_read = 0;
	counter0_write = 0;
	counter1_read = 0;
	counter1_write = 0;
	counterh_write = 0;
}

void ledInit(void){	
	DDRA = (1 << PORTA6);
	DDRD = (1 << PORTD1);
}

void notifyUpdate(){
	// to lcd
	char buf[16];
	sprintf(buf,"Bnk:%3d Prs:%3d",bank, preset);
	post(buf,0,TRUE);
	updateFlag = 0;
}



void post(char* msg, uint8_t row, uint8_t clear){
	if(clear) lcd_clrscr();
	lcd_gotoxy(0, row);
	lcd_puts(msg);
}

void waitForSynthesizer(void){
	uint8_t count = 0;
	char buf[4];
	
	softuart_turn_rx_on();
	for(count = 0; count < NUMBER_OF_ATTEMPT ; count++){
		// ping
		lcd_gotoxy(0,1);
		lcd_puts("ping count:");
		lcd_puts(itoa(count, buf, 10));
		
		uartSendByte(PING);
		uartSendByte(EOT);
		
		if (softuart_kbhit())
		{
			char c = softuart_getchar();
			if(HOST_RESPONSE == c){
				post("Response received\nSystem is ready", 0, TRUE);
				return;
			}else{
				post("Err:\n Wrong Response",0,TRUE);
			}
		}
		_delay_ms(500);
	}

	post("Err: no response",0,TRUE);
	_delay_ms(1000);
}

uint8_t checkSum(volatile uint8_t *buffer){
	uint16_t cs = 0;
	for(int i = 3;i < 21;i++)
	{
		cs += buffer[i];
	}
	cs &= 0xff;
	return (uint8_t)(0xff - cs);
}

int main(void)
{	
	uint8_t temp;

	MCUCR |=(1<<JTD); MCUCR |=(1<<JTD); //jtag disable
	lcd_init(LCD_DISP_ON); // lcd init
	softuart_init(); // suart init
	globalInit();
	timer1Init();
	ledInit();
	uartInit(MYUBBR);
	buttonInit();
	adcInit();
	post("Invisible Piano\nby C.Miyama 2014",0,TRUE);
	_delay_ms(1000);
	post("booting system...",0,TRUE);
	sei();
	waitForSynthesizer();
	_delay_ms(1000);
	softuart_flush_input_buffer();	
	notifyUpdate();

    while(1) // main loop
    {
		if(updateFlag){
			notifyUpdate();
		}
		//left hand
		while(counter0_write != counter0_read){
			temp = buffer0[counter0_read];
			if(temp == XBEE_START_DELIMETAR){
				packetCounter0 = 0;
			}
			packet0[packetCounter0] = temp;
			if(packetCounter0 == 21){ // packet full
				if(packet0[21] == checkSum(packet0)){
				uartSendByte(LEFT_HAND);
				uartSendByte((packet0[11] << 5) | (packet0[12] >> 3));
				uartSendByte((packet0[13] << 5) | (packet0[14] >> 3));
				uartSendByte((packet0[15] << 5) | (packet0[16] >> 3));				
				uartSendByte((packet0[17] << 5) | (packet0[18] >> 3));
				uartSendByte((packet0[19] << 5) | (packet0[20] >> 3));
				uartSendByte(EOT);
				}
			}
			packetCounter0++;	
			counter0_read++;
			counter0_read &= 0xff; // wrap 255
		}	
		
		// right hand
		while(counter1_write != counter1_read){
			temp = buffer1[counter1_read];
			if(temp == 126){
				packetCounter1 = 0;
			}
			packet1[packetCounter1] = temp;
			if(packetCounter1 == 21){ // packet full
				
				if(packet1[21] == checkSum(packet1)){
					uartSendByte(RIGHT_HAND);
					uartSendByte((packet1[11] << 5) | (packet1[12] >> 3));
					uartSendByte((packet1[13] << 5) | (packet1[14] >> 3));
					uartSendByte((packet1[15] << 5) | (packet1[16] >> 3));
					uartSendByte((packet1[17] << 5) | (packet1[18] >> 3));
					uartSendByte((packet1[19] << 5) | (packet1[20] >> 3));
					uartSendByte(EOT);
				}
			}
			packetCounter1++;
			counter1_read++;
			counter1_read &= 0xff; // wrap 255
		}

		// pedal
		uartSendByte(PEDAL);
		uartSendByte(adcRead(0));
		uartSendByte(adcRead(1));
		uartSendByte(adcRead(2));
		uartSendByte(EOT);
		
		// check the mail box
		while(softuart_kbhit()){
			uint8_t c = softuart_getchar();
			if(c != EOT){ 
				hostBuffer[counterh_write] = c;
				counterh_write++;
				if(counterh_write > HOSTMSG_BUFFER_SIZE){
					post("Err:msg overflow",1,FALSE);
				}
			}
			else{
				switch (hostBuffer[0])
				{
					case HOST_PRESET_CHANGE:{
						bank = hostBuffer[1];
						preset = hostBuffer[2];
						updateFlag = 1;
						break;
					}
					case HOST_PING:{
					
						break;	
					}
					case HOST_PRESET_CHANGE_CONFIRM:{
						if(proposalFlag){
							if((proposedBank == hostBuffer[1]) && (proposedPreset == hostBuffer[2])){
								bank = proposedBank;
								preset = proposedPreset;
								proposalCount = 0;
								proposalFlag = FALSE;
								updateFlag = TRUE;
							}else{
								post("Err:can't change", 1, FALSE);
							}
						}
						break;	
					}
					default:{
						char buf[2];
						post(itoa(hostBuffer[0],buf,16),1,0);
					}
				}
				counterh_write = 0;
			}
		}
		if(proposalFlag){
			// send request max 255 times
			uartSendByte(PRESET_CHANGE);
			uartSendByte(proposedBank);
			uartSendByte(proposedPreset);
			uartSendByte(EOT);
			
			if(proposalCount == 0xFF){ // wait 255 times
				post("Err:can't change", 1, FALSE);
				proposalFlag = FALSE; // reset parameter
				proposalCount = 0;
			}
			proposalCount++;
		}
		
		_delay_ms(5);
    }
}

/* interruptions */
ISR(TIMER1_COMPA_vect){
	uint8_t status = (PINB >> 2) & 0x03;
	timer1Stop();
	switch (status)
	{
		case 2:{
			if(bank != 255){
				proposalFlag = TRUE;
				proposedBank = bank + 1;
				proposedPreset = 0;
			}
			break;
		}
		case 1:{
			if(bank != 0){
				proposalFlag = TRUE;
				proposedBank = bank - 1;
				proposedPreset = 0;
			}
			break;
		}
		case 0:{
			post("System halt requested.",0, TRUE);
			uartSendByte(SYSTEM_HALT); //system halt
			uartSendByte(0xFF);
			break;
		}
	}
	preset = 0;
	updateFlag = 1;
}

ISR(PCINT1_vect){
	_delay_ms(20); // anti chattering
	PCIFR |= (1 << PCIF1); // cancel all scheduled interruption caused by chattering
	uint8_t status = (PINB >> 2) & 0x03;

	timer1Stop();
	switch (status)
	{
		case 2:{
			if(preset != 255){
				proposalFlag = TRUE;
				proposedPreset = preset + 1;
			}
			timer1Start();
			break;
		}
		case 1:{
			if(preset != 0){
				proposalFlag = TRUE;
				proposedPreset = preset - 1;

			}
			timer1Start();
			break;
		}
		case 0:{
			proposedPreset = 0;
			proposalFlag = TRUE;
			timer1Start();
			break;
		}
		default:{
			return; // don't set update frag if buttons are released
		}
	}
	updateFlag = 1;
}

ISR(USART0_RX_vect){
	buffer0[counter0_write] = UDR0;
	PORTA = PINA ^ (1 << PORTA6);
	counter0_write++;
	counter0_write &= 0xff;
}

ISR(USART1_RX_vect){
	buffer1[counter1_write] = UDR1;
	PORTA = PINA ^ (1 << PORTA6);
	counter1_write++;
	counter1_write &= 0xff;
}
