/*
	HwSerial.h - Hardware Serial Library for GCC AVR (Atmel Studio 6)
	
	This library is modified version of HardwareSerial.h library written 
	for Wiring originally by Nicholas Zambetti in 2006.
	
	This library is modified to be used under GNU C Compiler for AVR series
	of microcontrollers. Structures and Classes are removed from the library
	so that it can be used with C as well as C++ compilers. 
	
	Till now this library support AVR microcontroller with only 1 UART module.
	
	Pardeep Kumar
*/

#ifndef HWSERIAL_H
#define HWSERIAL_H

#include <avr/interrupt.h>
#include <inttypes.h>

//------------ Macros for Setting or Clearing Single Bit of a PORT -------------
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//------------ Macros for SERIAL Buffer Size -------------
#if (RAMEND < 1000)
#define SERIAL_BUFFER_SIZE 16
#else
#define SERIAL_BUFFER_SIZE 64
#endif

//Serial Buffer Creation
volatile char buffer[SERIAL_BUFFER_SIZE];
volatile int8_t head;
volatile int8_t tail;

//Serial Receive Interrupt Vector
#if defined(USART_RX_vect)
ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
ISR(USART0_RX_vect)
#elif defined(USART_RXC_vect)
ISR(USART_RXC_vect) //For ATmega8
#endif
{
	#if defined(UDR)
	unsigned char sData=UDR;
	#elif defined(UDR0)
	unsigned char sData=UDR0;
	#endif
	if(((tail==SERIAL_BUFFER_SIZE-1) && head==0) || ((tail+1)==head))
	{
		head++;
		if(head==SERIAL_BUFFER_SIZE)
		{
			head=0;
		}
	}
	if(tail == SERIAL_BUFFER_SIZE-1)
	{
		tail=0;
	}
	else
	{
		tail++;
	}
	buffer[tail] = sData;
	if(head == -1)
	{
		head=0;
	}
}

//Funtions for Serial Communication

void USART_Init(uint16_t baud)
{
	//Enable Global Interrupts
	sei();
	//Initialize Buffer Start & End
	head = -1;
	tail = -1;
	uint16_t UBR_Val;
	uint8_t use_u2x = 1;
	#if F_CPU == 16000000UL
	if (baud == 57600) 
	{
		use_u2x = 0;
    }
	#endif
try_again:
	//Generating Register Values for Specified Baud Rate and Crystal Frequency
    if (use_u2x) {
		#if defined(UCSRA)
			UCSRA = (1<<U2X);
		#elif defined (UCSR0A)
			UCSR0A = (1<<U2X0);
		#endif
		UBR_Val = (F_CPU / 4 / baud - 1) / 2;
	}
	else
	{
		UBR_Val = (F_CPU / 8 / baud - 1) / 2;
	}
	if ((UBR_Val > 4095) && use_u2x)
	{
		use_u2x = 0;
		goto try_again;
	}
	// Set baud rate, a.k.a. ubbr (USART Baud Rate Register)
	#if defined(UBRRH) && defined(UBRRL)
	UBRRH = (uint8_t)(UBR_Val>>8);
	UBRRL = (uint8_t)(UBR_Val);
	#elif defined(UBRR0L) && defined(UBRR0H)
	UBRR0H = (uint8_t)(UBR_Val>>8);
	UBRR0L = (uint8_t)(UBR_Val);
	#endif
	// Set frame format to 8 data bits, no parity, 1 stop bit
	#if defined(UCSRC)
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	#elif defined(UCSR0C)
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	#endif
	//enable transmission, reception and receive interrupt
	#if defined(UCSRB)
	UCSRB |= (1<<RXEN)|(1<<TXEN);
	sbi(UCSRB, RXCIE);
	cbi(UCSRB, UDRIE);
	#elif defined(UCSR0B)
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	sbi(UCSR0B, RXCIE0);
	cbi(UCSR0B, UDRIE0);
	#endif
}

char USART_Read()
{
	if(head==-1) return 0;
	char c = buffer[head];
	// if the head isn't ahead of the tail, we don't have any characters
	if(head == tail)
	{
		head=-1;
		tail=-1;
	}
	else
	{
		head++;
		if(head == SERIAL_BUFFER_SIZE)	head=0;
	}
	return c;
}
void USART_Send(char character)
{
	#if defined(UCSRA)
	while(!(UCSRA & (1<<UDRE)));
	UDR = character;
	#elif defined (UCSR0A)
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = character;
	#endif
}

uint8_t USART_Data_Ready()
{
	if(head==-1) return 0;
	if(head<tail)
		return(tail-head+1);
	else if(head>tail)
		return (SERIAL_BUFFER_SIZE-head+tail+1);
	else
		return 1;
}
void USART_Send_Text(char *str)
{
	while(*str != '\0')
	{
		USART_Send(*str);
		str++;
	}
}


#endif
