#include "rc5_german.h"
#include <util/delay.h>
//Программа вывода символа
void putchar( char c )
{
while( (UCSRA & 1<<UDRE) == 0 );
UDR = c;
}
 
//Программа вывода строки
void puts( char *s )
{
while( *s )
putchar( *s++ );
}
void usart_init (){
	 UBRRL = bauddivider; //Устанавливаем значение baud rate
	 UBRRH = bauddivider >> 8;
	 UCSRA = 0;//не используем режим U2X
	 UCSRC = 1<<URSEL^1<<UCSZ1^1<<UCSZ0;//формат данных 8бит
	 UCSRB = 1<<RXEN^1<<TXEN;//Разрешаем прием и передачу
}
 
void t0_init (){
	TCCR0 = 1<<CS02; //Деление тактовой частоты на 256
	TIMSK = 1<<TOIE0;	//Разрешаем прерывание по таймеру
}
//Основная программа
int main( void )
{
uint i;
char s[30];//переменная строковая
 

t0_init ();
usart_init ();
 
sei();//Разрешаем глобально прерывания
puts( "RC5-Decoder:\n\r" );//Выводим строку приветствия

for(;;)//Главный цикл
{				
cli();//запрет прерывания
i = rc5_data;//Читаем два байта из прерывания
rc5_data = 0;

sei();//разрешаем прерывания

if( i )
{	
	cli();
	_delay_ms(200);
	int n = ((i & 0x3F)|(~i >> 7 & 0x40)); //Выделяем только код команды
	sei();
}
}
}