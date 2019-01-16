/*
 * Ind_lib.h
 *
 * Created: 08.07.2015 11:48:28
 *  Author: trainee

 v1.5 - в ISR (TIMER2_OVF_vect) для случая без динамической индикации включение OE0 - OE3 по одному (было: case 1:{PORT_OE &=~ ((1<<OE0)|(1<<OE1));break;})
 - константа pwc определяет количество одновременно включаемых разрядов (включение линий OE выполнено через цикл)
 v.1.6 - константа pwc может быть определена в главном файле проекта (определена через #ifndef pwc #endif)
 v.1.7 -  в процедуре display_10code и display_10code_point для LED_TYPE==3 изменено расположение разрядов на стандартное
 v.1.8 - для всех типов плат включена процедура display_dynamic (переименована в display_send)
 v.1.9	- убрана константа DYNAMIC, всегда выполяется процедура display_send(uint8_t dig_num)
		- добавлен тип индикации LED_TYPE 5 для платы im-49.3 с квазидинамическим отображением цифр,
			 количество одновременно отображаемых цифр определяет переменная pwc
 v.2.0	- введена процедура set_PWM для установки значения pwc
		- исправлена процедура display_send - добавлен break; в структуру switch-case
 v.2.1	- в процедуру display_7code добавлена сортировка разрядов и отображение для LED_TYPE=5
		- по умолчанию (если не определен LED_TYPE) LED_TYPE = 5
 v.2.2	- для контроллера CDigit4x - введено обозначение порта #define MR PB4, подключенного к инверсному входу сброса ~MR сдвиговых регистров
		- в инициализацию void init595 () добавлена конфигурация DDRB |= (1<<4) для порта PB4
		- установка выхода PB4 в активное состояние логической единицы (PORTB4 = 1)

	- БАРДАК!
	- надо добавить для контроллера CU_4.3 (CONTROL_TYPE=0) отправку данных через процедуру dysplay_send для драйвера IM-49 (LED_TYPE=0, 2 и др.)
	
 v.3.0	- оставляем индикацию только для контроллера CDigit4x по типу LED_TYPE==5
*/ 


#ifndef IND_LIB_H_
	#define IND_LIB_H_

	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include "Stela_IRControl_.h"
	
	#define PORT_595	PORTD
	#define DDR_595		DDRD 		
	#define PORT_OE		PORTB
	#define DDR_OE 		DDRB
	#define MR 			PB4			//инверсный вход сброса ~MR сдвиговых регистров
	#define DATA  		PD5			//выход данных
	#define CLK   		PD6			//тактовый для сдвига данных
	#define LATCH   	PD7			//защелка регистра
	#define OE0 		PB0			// OE сдвигового регистра 1
	#define OE1 		PB1			// OE сдвигового регистра 2
	#define OE2 		PB2			// OE сдвигового регистра 3
	#define OE3 		PB3			// OE сдвигового регистра 4

//константа pwc для квазидинамической поразрядной индикации 
// количество одновременно включенных разрядов (при pwc=1 - аналог динамической индикации, pwc=4 - статической)
	uint8_t pwc;
	void set_PWC(uint8_t PWM_index);
	
	uint8_t OE_count;
	
	//сортировка разрядов табло: указываются номера разрядов по порядку включения
	uint8_t Dsort[4];	//массив, содержащий порядок включения разрядов табло
	void digit_sort(uint8_t NumDig3, uint8_t NumDig2, uint8_t NumDig1, uint8_t NumDig0);
	
	uint8_t DigitArr[4];
	void display_send(uint8_t dig_num);		//загрузка данных в регистры драйвера табло

	void init595 ();						//инициализация выводов для сдвигового регистра
	void CLK_PULSE ();						//импульс сдвига CLK регистра HC595
	void LATCH_PULSE ();					//импульс защелки LE регистра HC595
	void send_data8 (uint8_t data);			//запись 1 байта в регистр сдвига
	void send_data8_back (uint8_t data);	//запись 1 байта в регистр сдвига в обратном порядке
	
	//вывод на табло в представлении 7сегментного индикатора
	void display_7code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); //на вход байт в 7сег. коде, D0 правый разряд табло 
	
	//вывод на табло в десятичном представлении
	//вход - 4 десятичных числа - выход на табло в 7сегментном коде
	void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); 
	void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask);
	void display_dnum(int32_t num);
	void initPWM(); //инициализация режима ШИМ


#endif /* IND_LIB_H_ */