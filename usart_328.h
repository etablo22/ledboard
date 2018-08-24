//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega16
//
//  Compiler....: GCC
//
//  Description.: драйвер USART/UART с кольцевым буфером
//
//  Data........: 
//
//***************************************************************************
#ifndef USART_H
#define USART_H

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#if ( (CPU_TYPE == ATMEGA328) || (CPU_TYPE == ATMEGA88) )
//for Atmega88 and Atmega328
//USART module
#define UDR		UDR0
#define UDRIE	UDRIE0
#define U2X		U2X0

#define	UCSRA	UCSR0A
#define	UCSRB	UCSR0B
#define	UCSRC	UCSR0C
#define	UBRRH	UBRR0H
#define	UBRRL	UBRR0L

#define	RXCIE	RXCIE0
#define	RXEN	RXEN0
#define	TXEN	TXEN0
#define	URSEL	UMSEL01
#define	UCSZ0	UCSZ00
#define	UCSZ1	UCSZ01

#define USART_TXC_vect USART_TX_vect
#define USART_RXC_vect USART_RX_vect

#elif	(CPU_TYPE == ATMEGA8)
//for Atmega8A
//is default
#endif


#define USART_NORMAL  0
#define USART_DOUBLED 1

#define SIZE_BUF_RX 64       //задаем размер кольцевых буферов - <255
#define SIZE_BUF_TX 64

//****************************************************************************
void USART_Init(uint8_t regime, uint16_t baudRate); //инициализация usart`a
uint8_t USART_GetTxCount(void); //взять число символов передающего буфера
void USART_FlushTxBuf(void); //очистить передающий буфер
void USART_PutChar(char sym); //положить символ в буфер
void USART_SendStr(char * data); //послать строку из озу по usart`у
void USART_SendStrFl(char const * data); //послать строку из флэша по usart`у
uint8_t USART_GetRxCount(void); //взять число символов в приемном буфере
void USART_FlushRxBuf(void); //очистить приемный буфер
char USART_GetChar(void); //прочитать приемный буфер usart`a 
void USART_GetBuf(uint8_t num, char *buf); //скопировать приемный буфер usart`a 
#endif //USART_H