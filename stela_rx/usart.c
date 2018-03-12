//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: IAR EWA 5.11A
//
//  Description.: ������� USART/UART � ��������� �������
//
//  Data........: 6.01.13 
//
//***************************************************************************
#include "usart.h"

//���������� �����
static volatile char usartTxBuf[SIZE_BUF_TX];
static volatile uint8_t txBufTail = 0;
static volatile uint8_t txBufHead = 0;
static volatile uint8_t txCount = 0;

//�������� �����
static volatile char usartRxBuf[SIZE_BUF_RX];
static volatile uint8_t rxBufTail = 0;
static volatile uint8_t rxBufHead = 0;
static volatile uint8_t rxCount = 0;

#ifndef F_CPU
#error "F_CPU is not defined"
#endif


//������������� usart`a
void USART_Init(uint8_t regime, uint16_t baudRate)
{
  uint16_t ubrrValue;

  uint8_t save = SREG;
  cli();	
	
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
  
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
  
  UCSRB = 0;
  UCSRC = 0;
  
  if (regime == USART_NORMAL){
    ubrrValue = F_CPU/(16UL*baudRate) - 1;
  }
  else{
    ubrrValue = F_CPU/(8UL*baudRate) - 1;
  }
  
  UBRRH = (uint8_t)(ubrrValue >> 8);  
  UBRRL = (uint8_t)ubrrValue;

  UCSRA = (1<< (1 & U2X));
  UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); //����. ������ ��� ������, ���� ������, ���� ��������.
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); //������ ����� 8 ��������

  SREG = save;
}

//______________________________________________________________________________
//���������� ����������� �������� ����������� ������
uint8_t USART_GetTxCount(void)
{
  return txCount;  
}

//"�������" ���������� �����
void USART_FlushTxBuf(void)
{
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
}

//�������� ������ � �����, ���������� ������ ��������
void USART_PutChar(char sym)
{
    while(txCount == SIZE_BUF_TX);
    
	cli();
    if (!txCount){
       UCSRB |= (1<<UDRIE);
    }
    if (txCount < SIZE_BUF_TX){    //���� � ������ ��� ���� �����
      usartTxBuf[txBufTail] = sym; //�������� � ���� ������
      txCount++;                   //�������������� ������� ��������
      txBufTail++;                 //� ������ ������ ������
      if (txBufTail == SIZE_BUF_TX) txBufTail = 0;
    }
	sei();
}

//������� ���������� ������ �� ��� �� usart`�
void USART_SendStr(char * data)
{
  char sym;
  while(*data){
    sym = *data++;
    USART_PutChar(sym);
  }
}

//������� ���������� ������ �� ����� �� usart`�
void USART_SendStrFl(char const *data)
{
  char sym = pgm_read_byte(data);
  
  while(sym){
    USART_PutChar(sym);
	data++;
    sym = pgm_read_byte(data);
  }
}

//���������� ���������� 
ISR(USART_UDRE_vect) 
{
  if (txCount > 0){              //���� ����� �� ������
    UDR = usartTxBuf[txBufHead]; //���������� � UDR ������ �� ������
    txCount--;                   //��������� ������� ��������
    txBufHead++;                 //�������������� ������ ������ ������
    if (txBufHead == SIZE_BUF_TX) txBufHead = 0;
  } 
  else{
    UCSRB &= ~(1<<UDRIE);
  }
} 

//______________________________________________________________________________
//���������� ����������� �������� ����������� � �������� ������
uint8_t USART_GetRxCount(void)
{
  return rxCount;  
}

//"�������" �������� �����
void USART_FlushRxBuf(void)
{
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
}

//������ ������
char USART_GetChar(void)
{
  char sym;
  if (rxCount > 0){                     //���� �������� ����� �� ������  
    sym = usartRxBuf[rxBufHead];        //��������� �� ���� ������    
    rxCount--;                          //��������� ������� ��������
    rxBufHead++;                        //���������������� ������ ������ ������  
    if (rxBufHead == SIZE_BUF_RX) rxBufHead = 0;
    return sym;                         //������� ����������� ������
  }
  return 0;
}

volatile uint8_t block = 0;

//���������� �� ���������� ������
ISR(USART_RXC_vect) 
{
    char data = UDR;
    if (rxCount < SIZE_BUF_RX){                    //���� � ������ ��� ���� �����   
      usartRxBuf[rxBufTail] = data;                //������� ������ �� UDR � �����
      rxBufTail++;                                 //��������� ������ ������ ��������� ������ 
      if (rxBufTail == SIZE_BUF_RX) rxBufTail = 0;  
      rxCount++;                                   //��������� ������� �������� ��������
    }
} 
      


