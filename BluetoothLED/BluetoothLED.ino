#include <avr/interrupt.h>
#include "stdlib.h"

//baud rate calculation - set baude rate here
#define FOSC 16000000 // Clock Speed
#define BAUD 9600       //
#define MYUBRROLD FOSC/16/BAUD-1
#define MYUBRR 0x067 //9600 baud


//rgb led values
unsigned char ledRed = 255;
unsigned char  ledBlue = 255;
unsigned char  ledGreen = 255;

//serial comm variables
char receive[11], *ptr, rx_done; //stores the received data, pointer to the data, done flag
char temp_value[3]; //temp storage for int conversion

//comm data format #RRRGGGBBB% 

void setup() {
  cli();//stop interrupts
  
  //serial port setup
  UBRR0H = (unsigned char)(MYUBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR0L = (unsigned char)MYUBRR; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // Turn on the transmission, reception, and Receive interrupt
  UCSR0C |= (1 << USBS0) | (3 << UCSZ00); // Use 8-bit character sizes

  sei();//allow interrupts
}


ISR(USART_RX_vect)
{
  unsigned char ch;
  //check flag
  if (UCSR0A & (1 << RXC0))
  {
    ch = UDR0; //load data from register
    //check for start char
    if (ch == '#')
      ptr = receive; //reset the pointer
    if (ptr >= &receive[11]) // ensure pointer is restrict to the buffer
      ptr = receive;
    *ptr++ = ch; //save char and increment pointer
    //check for terminating char
    if (ch == '%') {
      rx_done = 1;
    }
  }
}

void loop() {
  if (rx_done) //check if new data is avaiable
  {
    rx_done = 0; //reset the flag
    //use the temporary char array for int conversion
    temp_value[0] = receive[1];
    temp_value[1] = receive[2];
    temp_value[2] = receive[3];
    ledRed = atoi(temp_value);
    temp_value[0] = receive[4];
    temp_value[1] = receive[5];
    temp_value[2] = receive[6];
    ledGreen = atoi (temp_value);
    temp_value[0] = receive[7];
    temp_value[1] = receive[8];
    temp_value[2] = receive[9];
    ledBlue = atoi (temp_value);
    
    //transmit 1
    USART_Transmit_String("1\r\n");
    
    //update the duty cycle
    analogWrite (11, ledRed);
    analogWrite (6, ledGreen);
    analogWrite (5, ledBlue);

  }

}

//send a char over the serial port
void USART_Transmit(char data )
{
  //Wait for empty transmit buffer
  while (!( UCSR0A & (1 << UDRE0)));
  //Put data into buffer, sends the data
  UDR0 = data;
}

//send a string over the serial port
void USART_Transmit_String ( char *s) {
  while (*s) //transmit until the null char
    USART_Transmit(*s++);
}

