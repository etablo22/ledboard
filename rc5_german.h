#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
 
#define uchar unsigned char
#define uint uint16_t

#define DDRrc5		DDRC
#define	xRC5_IN		PINC//Задаем порт для датчика
#define	xRC5		PC0	//Задаем ногу порта для датчика
 
#define	XTAL		14.7456e6//Задаем частоту кварца
//#define	XTAL		8e6//Задаем частоту кварца
//#define	XTAL		11.0592e6
//#define	XTAL		7.3728e6
//#define XTAL		5e6
 
// #define  BAUD	19200//Задаем скорость соединения по USART
// //#define	BAUD	115200
//  
// #define bauddivider (uint)(XTAL / BAUD / 16 - 0.5)

void initRC5();
extern uint	rc5_data;//Задаем переменную для хранения полученного результата