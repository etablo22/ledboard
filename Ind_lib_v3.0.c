/*
 * Ind_lib.c
 *
 * Created: 08.07.2015 12:00:41
 *  Author: trainee
 
 v1.5	- в ISR (TIMER2_OVF_vect) для случая без динамической индикации включение OE0 - OE3 по одному (было: case 1:{PORT_OE &=~ ((1<<OE0)|(1<<OE1));break;})
		- константа pwc определяет количество одновременно включаемых разрядов (включение линий OE выполнено через цикл) для платы IM_51
 v.1.8	- в процедурах display_10code... для всех плат, кроме prodv2011, одинаковое определение кода для загрузки в регистры табло	
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

#include "Ind_lib_v3.0.h"

//1 знак справа

//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
uint8_t ABCD_TABLE [MAXDIGNUMBER]= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};


//инициализация режима ШИМ
void initPWM()
{
	DDR_OE |= (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);  //Конфигурация портов ШИМ на выход
	PORT_OE &=~ (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);

#if ( (CPU_TYPE == ATMEGA328) || (CPU_TYPE == ATMEGA88) )
	//for Atmega88 and Atmega328
	TCCR2B=(1<<CS21)|(1<<CS20); //
	TCCR2A=(1<<WGM21)|(1<<WGM20); //
	TIMSK2 |= (1<<OCIE2A)|(1<<TOIE2);
	OCR2A=127;   //начальная значение яркости
#elif	(CPU_TYPE == ATMEGA8)		
	//for Atmega8A
	TCCR2=(1<<CS22)|(1<<CS20)|(1<<WGM21)|(1<<WGM20);
	TIMSK |= (1<<OCIE2)|(1<<TOIE2);
	OCR2=127;   //начальная значение яркости
#endif
		
}

//прерывания таймера Т2 по сравнению и переполнению для реализации ШИМ
//квазидинамическая индикация: поочередная загрузка разрядов (с учетом pwc), яркость устанавливается ШИМ по линии OE
//установка портов индикации в 0
#if ( (CPU_TYPE == ATMEGA328) || (CPU_TYPE == ATMEGA88) )
	ISR (TIMER2_COMPA_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
#elif	(CPU_TYPE == ATMEGA8)
	ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
#endif
//установка портов индикации в 1
ISR (TIMER2_OVF_vect){
	PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));
	OE_count++;
	if (OE_count<1) OE_count=4;
	if (OE_count>4) OE_count=1;
	display_send(OE_count);
}


//инициализация выводов для управления сдвиговым регистром
void init595 ()
{	
	DDR_595 |= (1<<DATA)|(1<<CLK)|(1<<LATCH);  		//Конфигурация порта на выход
	PORT_595 &=~ (1<<DATA)|(1<<CLK)|(1<<LATCH);		//установка начальных значений 0
	DDR_OE |= (1<<MR);								//конфигурация порта, управляющего инверсным входм ~MR
	PORT_OE |= (1<<MR);								//установка значения логической 1 порта, управляющего инверсным входм ~MR
}

//импульс сдвига CLK регистра HC595
void CLK_PULSE ()
{
	PORT_595|=_BV(CLK);	//Импульс на SCL
	asm("nop");
	PORT_595&=~_BV(CLK);
	asm("nop");
}

//импульс защелки LE регистра HC595
void LATCH_PULSE ()
{
	PORT_595|=_BV(LATCH);	//Импульс на Latch clock
	asm("nop");
	PORT_595&=~_BV(LATCH);
	asm("nop");
}
//запись 1 байта в регистр сдвига
void send_data8 (uint8_t data)
{
	unsigned char i;
	for (i=0;i<8;i++)
		{
			if ((data&0x80)!=0) PORT_595|=_BV(DATA);//Выставляем данные на PD0
			else PORT_595&=~_BV(DATA);
			CLK_PULSE();
			data=(data<<1);
		}
}
//запись 1 байта в регистр сдвига в обратном порядке
void send_data8_back (uint8_t data)
{
	unsigned char i;
	for (i=0;i<8;i++)
	{
		if ((data&0x01)!=0) PORT_595|=_BV(DATA);//Выставляем данные на PD0
		else PORT_595&=~_BV(DATA);
		CLK_PULSE();
		data=(data>>1);
	}
}

//вывод на табло в представлении 7сегментного индикатора
//на вход байт в 7сег. коде, D0 правый разряд табло 
void display_7code (uint8_t Dig3, uint8_t Dig2, uint8_t Dig1, uint8_t Dig0)
{
	DigitArr[Dsort[0]]=Dig0;
	DigitArr[Dsort[1]]=Dig1;
	DigitArr[Dsort[2]]=Dig2;
	DigitArr[Dsort[3]]=Dig3;
}

//вывод на табло в десятичном представлении
//на вход байт как десятичное число, D0 правый разряд табло 
void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0)
{
	//Вывод на табло в коде ABCD
	display_7code (ABCD_TABLE[D3], ABCD_TABLE[D2], ABCD_TABLE[D1], ABCD_TABLE[D0]);
// 	DigitArr[Dsort[0]]=ABCD_TABLE[D0];
// 	DigitArr[Dsort[1]]=ABCD_TABLE[D1];
// 	DigitArr[Dsort[2]]=ABCD_TABLE[D2];
// 	DigitArr[Dsort[3]]=ABCD_TABLE[D3];
}

//вывод на табло в десятичном представлении - с учетом точки: point - маска точек
//на вход байт как десятичное число, D0 правый разряд табло
void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask)
{
	//стандартное расположение разрядов
	DigitArr[Dsort[0]]=ABCD_TABLE[D0]|((pointmask<<7)&0x80);		//сложно обойтись без сдвига, т.к. с разными масками устанавливаются разные биты
	DigitArr[Dsort[1]]=ABCD_TABLE[D1]|((pointmask<<6)&0x80);
	DigitArr[Dsort[2]]=ABCD_TABLE[D2]|((pointmask<<5)&0x80);
	DigitArr[Dsort[3]]=ABCD_TABLE[D3]|((pointmask<<4)&0x80);
}

void DigitSorting (uint8_t DigitNumber, uint8_t DigitValue)
{
	DigitArr[Dsort[DigitNumber]] = DigitValue;
}

//вывод на табло десятичного числа от 0 до 9999
void display_dnum(int32_t num)
{
	display_10code ((uint8_t)num/1000,(uint8_t)num/100%10,(uint8_t)num/10%10,(uint8_t)num%10);
}

//индикация динамическая
void display_send(uint8_t dig_num)
{
	//Вывод на табло в коде ABCDEF
	//переменная pwc определяет количество одновременно включенных разрядов табло от 1 до 4
	//dig_num -	определяет номера выводимых разрядов
		switch (pwc) 
		{
// 			case 1:
// 				switch (dig_num){
// 					case 1:{send_data8(0x00); send_data8(0x00); send_data8(0x00); send_data8(D[3]); break;}
// 					case 2:{send_data8(0x00); send_data8(0x00); send_data8(D[2]); send_data8(0x00); break;}
// 					case 3:{send_data8(0x00); send_data8(D[1]); send_data8(0x00); send_data8(0x00); break;}
// 					case 4:{send_data8(D[0]); send_data8(0x00); send_data8(0x00); send_data8(0x00); break;}
// 				}
// 				break;
// 			case 2:
// 				switch (dig_num){
// 					case 1:{send_data8(0x00); send_data8(0x00); send_data8(D[2]); send_data8(D[3]); break;}
// 					case 2:{send_data8(0x00); send_data8(D[1]); send_data8(D[2]); send_data8(0x00); break;}
// 					case 3:{send_data8(D[0]); send_data8(D[1]); send_data8(0x00); send_data8(0x00); break;}
// 					case 4:{send_data8(D[0]); send_data8(0x00); send_data8(0x00); send_data8(D[3]); break;}
// 				}
// 				break;
// 			case 3:
// 				switch (dig_num){
// 					case 1:{send_data8(0x00); send_data8(D[1]); send_data8(D[2]); send_data8(D[3]); break;}
// 					case 2:{send_data8(D[0]); send_data8(D[1]); send_data8(D[2]); send_data8(0x00); break;}
// 					case 3:{send_data8(D[0]); send_data8(D[1]); send_data8(0x00); send_data8(D[3]); break;}
// 					case 4:{send_data8(D[0]); send_data8(0x00); send_data8(D[2]); send_data8(D[3]); break;}
// 				}
// 				break;
			case 4:
				send_data8(DigitArr[0]); send_data8(DigitArr[1]); send_data8(DigitArr[2]); send_data8(DigitArr[3]);
				break;
		}
	LATCH_PULSE();
}

//сортировка разрядов табло: указываются номера разрядов по порядку включения
void digit_sort(uint8_t NumDig3, uint8_t NumDig2, uint8_t NumDig1, uint8_t NumDig0){
	Dsort[0]=NumDig0;
	Dsort[1]=NumDig1;
	Dsort[2]=NumDig2;
	Dsort[3]=NumDig3;
} 

//установка значения pwc - количества одновременно включенных разрядов (при pwc=1 - аналог динамической индикации, pwc=4 - статической)
void set_PWC(uint8_t PWM_index){
	pwc=PWM_index;
}