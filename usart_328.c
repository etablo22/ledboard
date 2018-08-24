//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: IAR EWA 5.11A
//
//  Description.: драйвер USART/UART с кольцевым буфером
//
//  Data........: 6.01.13 
//
//***************************************************************************
#include "usart_328.h"

//передающий буфер
static volatile char usartTxBuf[SIZE_BUF_TX];
static volatile uint8_t txBufTail = 0;
static volatile uint8_t txBufHead = 0;
static volatile uint8_t txCount = 0;

//приемный буфер
static volatile char usartRxBuf[SIZE_BUF_RX];
static volatile uint8_t rxBufTail = 0;
static volatile uint8_t rxBufHead = 0;
static volatile uint8_t rxCount = 0;

#ifndef F_CPU
#error "F_CPU is not defined"
#endif


//инициализация usart`a
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
  
  if (regime == USART_NORMAL){
	  ubrrValue = F_CPU/(16UL*baudRate) - 1;
  }
  else{
	  ubrrValue = F_CPU/(8UL*baudRate) - 1;
  }
  
	UCSRB = 0;
	UCSRC = 0;
	
    UBRRH = (uint8_t)(ubrrValue >> 8);
	UBRRL = (uint8_t)ubrrValue;
	UCSRA = (1<< (1 & U2X));
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN); //разр. прерыв при приеме, разр приема, разр передачи.
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); //размер слова 8 разрядов

	SREG = save;

}

//______________________________________________________________________________
//возвращает колличество символов передающего буфера
uint8_t USART_GetTxCount(void)
{
  return txCount;  
}

//"очищает" передающий буфер
void USART_FlushTxBuf(void)
{
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
}

//помещает символ в буфер, инициирует начало передачи
void USART_PutChar(char sym)
{
    while(txCount == SIZE_BUF_TX);
    
	cli();
    if (!txCount){
       UCSRB |= (1<<UDRIE);
    }
    if (txCount < SIZE_BUF_TX){    //если в буфере еще есть место
      usartTxBuf[txBufTail] = sym; //помещаем в него символ
      txCount++;                   //инкрементируем счетчик символов
      txBufTail++;                 //и индекс хвоста буфера
      if (txBufTail == SIZE_BUF_TX) txBufTail = 0;
    }
	sei();
}

//функция посылающая строку из озу по usart`у
void USART_SendStr(char * data)
{
  char sym;
  while(*data){
    sym = *data++;
    USART_PutChar(sym);
  }
}

//функция посылающая строку из флэша по usart`у
void USART_SendStrFl(char const *data)
{
  char sym = pgm_read_byte(data);
  
  while(sym){
    USART_PutChar(sym);
	data++;
    sym = pgm_read_byte(data);
  }
}

//обработчик прерывания 
ISR(USART_UDRE_vect) 
{
  if (txCount > 0){              //если буфер не пустой
    UDR = usartTxBuf[txBufHead]; //записываем в UDR символ из буфера
    txCount--;                   //уменьшаем счетчик символов
    txBufHead++;                 //инкрементируем индекс головы буфера
    if (txBufHead == SIZE_BUF_TX) txBufHead = 0;
  } 
  else{
    UCSRB &= ~(1<<UDRIE);
  }
} 

//______________________________________________________________________________
//возвращает колличество символов находящихся в приемном буфере
uint8_t USART_GetRxCount(void)
{
  return rxCount;  
}

//"очищает" приемный буфер
void USART_FlushRxBuf(void)
{
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
}

//чтение буфера
char USART_GetChar(void)
{
  char sym;
  if (rxCount > 0){                     //если приемный буфер не пустой  
    sym = usartRxBuf[rxBufHead];        //прочитать из него символ    
    rxCount--;                          //уменьшить счетчик символов
    rxBufHead++;                        //инкрементировать индекс головы буфера  
    if (rxBufHead == SIZE_BUF_RX) rxBufHead = 0;
    return sym;                         //вернуть прочитанный символ
  }
  return 0;
}

volatile uint8_t block = 0;

//прерывание по завершению приема
ISR(USART_RXC_vect) 
{
    char data = UDR;
    if (rxCount < SIZE_BUF_RX){                    //если в буфере еще есть место   
      usartRxBuf[rxBufTail] = data;                //считать символ из UDR в буфер
      rxBufTail++;                                 //увеличить индекс хвоста приемного буфера 
      if (rxBufTail == SIZE_BUF_RX) rxBufTail = 0;  
      rxCount++;                                   //увеличить счетчик принятых символов
    }
} 
      


