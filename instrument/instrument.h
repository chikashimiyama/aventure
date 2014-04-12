
// choose the instrument from PIANO FLUTE TROMBONE PERCUSSION
// and provide the compiler with the symbol with -D option 

#include "lcd.h"
//debugging
//#define LED_CHECK // if defined, just blink led
//#define BUTTON_CHECK // if defined, Button and LEDs will be linked
//#define UART_TX_CHECK
#define UART_SOFT_RX_CHECK

//SPLASH Text
#ifdef PIANO
	#define SPLASH "Invisible Piano\nby C.Miyama 2014"
#endif	
#ifdef FLUTE
	#define SPLASH "Invisible Flute\nby C.Miyama 2014"
#endif
#ifdef TROMBONE
	#define SPLASH "Invisible Trb.\nby C.Miyama 2014"
#endif
#ifdef PERCUSSION
	#define SPLASH "Invisible Perc.\nby C.Miyama 2014"
#endif

#define TRUE 1
#define FALSE 0
#define F_CPU 20000000UL // always 20 Mhz
#define BAUD  57600 
#define MYUBBR F_CPU/16/BAUD-1 // for Hardware UART
#define NUMBER_OF_ATTEMPT 100 // number of ping to Pd patch
#define BUFFER_SIZE 256 // UART Buffer
#define HOSTMSG_BUFFER_SIZE 24 // buffer for the message from the host
#define PACKET_LENGTH 25 // depends on instrument

#ifdef PIANO // invisible piano ... old type
	// button 
	#define BUTTON_PORT PORTB
	#define BUTTON_PIN PINB
	#define BUTTON_PIN_SHIFT 2
	#define BUTTON_PIN_L PB2
	#define BUTTON_PIN_R PB3
	#define BUTTON_DDR DDRB
	#define BUTTON_PCINT_L PCINT10
	#define BUTTON_PCINT_R PCINT11
	#define PCINT_EN_GROUP PCIE1
	#define BUTTON_PCINT_VECT PCINT1_vect
	#define BUTTON_PCIF PCIF1
	
	// xbee led
	#define XBEE_LED_PORT PORTA
	#define XBEE_LED_DDR DDRA
	#define XBEE_LED_PIN PA6
	// host
	#define HOST_LED_PORT PORTA
	#define HOST_LED_DDR DDRA
	#define HOST_LED_PIN PA7
	
#else  // other inst ... new type
	
	// button
	#define BUTTON_PORT PORTD
	#define BUTTON_PIN PIND
	#define BUTTON_PIN_SHIFT 5
	#define BUTTON_PIN_L PD5
	#define BUTTON_PIN_R PD6
	#define BUTTON_DDR DDRD
	#define BUTTON_PCINT_L PCINT29
	#define BUTTON_PCINT_R PCINT30
	#define PCINT_EN_GROUP PCIE3
	#define BUTTON_PCINT_VECT PCINT3_vect
	#define BUTTON_PCIF PCIF3

	// xbee
	#define XBEE_LED_PORT PORTD
	#define XBEE_LED_DDR DDRD
	#define XBEE_LED_PIN PD7
	
	// host
	#define HOST_LED_DDR DDRC
	#define HOST_LED_PORT PORTC
	#define HOST_LED_PIN PC7
	
#endif

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

