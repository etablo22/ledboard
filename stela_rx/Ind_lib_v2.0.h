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
*/ 


#ifndef IND_LIB_H_
	#define IND_LIB_H_

	#include <avr/io.h>
	#include <avr/interrupt.h>

	//LED_TYPE - тип табло индикации:
	// 0 -	пиксельные табло;
	// 1 -	табло "ПРОДВИЖЕНИЕ(prodv2011)"; 
	// 2 -	индикация на плате IM-51; 
	// 3 -	динамическая индикация на плате DLD_x;
	// 4 -	для платы IM-49 с обратным расположением сегментов, ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ (подходит для платы DLD);
	// 5 - для платы im-49.3 с квазидинамическим отображением цифр
	#define LED_TYPE 5
	#define CONTROLLER_TYPE 1 // 0 - блок управления стандарт CU_v.4.3; 1 - БУ на плате LED-контреллера CM_1.3|CM_v.2.0

	#if (CONTROLLER_TYPE==0)
		#define PORT_595	PORTB
		#define DDR_595		DDRB
		#define PORT_OE		PORTB
		#define DDR_OE		DDRB
		#define DATA		PB2         //выход данных
		#define LATCH		PB4			//защелка регистра
		
		#if ((LED_TYPE==0)||(LED_TYPE==4))		//табло IM-49 (пиксельные табло)
			#define CLK		PB3 		//тактовый для сдвига данных
			#define OE_595	PB5         // OE сдвигового регистра
		#elif (LED_TYPE==1)		//табло "ПРОДВИЖЕНИЕ(prodv2011)"
			#define CLK		PB5			//тактовый для сдвига данных
			#define OE_595	PB3			// OE сдвигового регистра
		#endif
		
	#elif (CONTROLLER_TYPE==1)
		#define PORT_595 PORTD
		#define DDR_595  DDRD 		
		#define PORT_OE  PORTB
		#define DDR_OE DDRB
		#define DATA  PD5          //выход данных
		#define CLK   PD6 		  //тактовый для сдвига гаддных
		#define LATCH   PD7		  //защелка регистра
		#define OE0 PB0         // OE сдвигового регистра
		#define OE1 PB1         // OE сдвигового регистра
		#define OE2 PB2         // OE сдвигового регистра
		#define OE3 PB3         // OE сдвигового регистра
	#endif

	#define MAXDIGNUMBER 13 // максимальный код символа в таблицах символов табло

//константа pwc для квазидинамической поразрядной индикации 
// количество одновременно включенных разрядов (при pwc=1 - аналог динамической индикации, pwc=4 - статической)
//платы IM-51 #if ((LED_TYPE==3)||(LED_TYPE==4)) //платы IM-49.3 #if (LED_TYPE==5)
//#define pwc	2			
	uint8_t pwc;
	void set_PWC(uint8_t PWM_index);
	
	uint8_t OE_count;
	uint8_t Dsort[4];
	void digit_sort(uint8_t dig3, uint8_t dig2, uint8_t dig1, uint8_t dig0); //сортировка разрядов табло
	
	uint8_t D[4];
	void display_send(uint8_t dig_num); //загрузка данных в регистры драйвера табло

	void init595 ();  //инициализация выводов для сдвигового регистра
	void CLK_PULSE ();
	void LATCH_PULSE ();
	void send_data8 (uint8_t data);
	void send_data8_back (uint8_t data);
	//void send_data16 (uint16_t data);
	void display_7code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); //на вход байт в 7сег. коде, D0 правый разряд табло 
	//функция вывода вход - 4 десятичных числа - выход на табло в 7сегментном коде или коде целого табло
	void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); 
	void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask);
	void display_dnum(int32_t num);
	void initPWM(); //инициализация режима ШИМ


#endif /* IND_LIB_H_ */