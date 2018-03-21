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
*/

#include "Ind_lib_v2.2.h"

#ifndef LED_TYPE
#define LED_TYPE 5 //0- пиксельные табло IM-49.2; 1 - табло "ПРОДВИЖЕНИЕ"; 2-индикация на плате IM-51; 3-динамическая индикация; 4-IM-49
//0 - standard 7-segcode 4-digit indicator
//1 - "prodv2011"-table with non-standard coding
//2 - 7-segcode 4-digit indicator with backwards Dig3,Dig4
//3 - dynamic indication on 7-segcode 4-digit indicator
//4 - для платы IM-49 с обратным расположением сегментов, ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ (подходит для платы DLD)
//5 - для платы im-49.3 с квазидинамическим отображением цифр
#endif


//1 знак справа

//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
uint8_t ABCD_TABLE [MAXDIGNUMBER]= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};

//вместо таблицы CBA_TABLE используется функция send_data8_back();
//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
//uint8_t CBA_TABLE [MAXDIGNUMBER]= {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6, 0x02, 0x00, 0xC6};


//--------------------------------0-------1------2------3------4------5------6------7------8-------9---minus--null----^C--
uint16_t Dig0 [MAXDIGNUMBER] = {0x1EC0,0x1800,0x1740,0x1D40,0x1980,0x0DC0,0x0FC0,0x1840,0x1FC0,0x1DC0,0x0100,0x0000,0x11C0};

//--------------------------------0-------1------2------3------4------5------6------7------8-------9---minus--null----^C--
uint16_t Dig1 [MAXDIGNUMBER] = {0x202F,0x0028,0x2036,0x203C,0x0039,0x201D,0x201F,0x2028,0x203F,0x203D,0x0010,0x0000,0x2031};

uint16_t Dig2 [MAXDIGNUMBER] = {0xF401,0xC000,0xB801,0xE801,0xCC00,0x6C01,0x7C01,0xC001,0xFC01,0xEC01,0x0800,0x0000,0x8C01};
   
uint16_t Dig3 [MAXDIGNUMBER] = {0x0378,0x0140,0x03B0,0x03E0,0x01C8,0x02E8,0x02F8,0x0340,0x03F8,0x03E8,0x0080,0x0000,0x0388};


	//инициализация режима ШИМ
#if (CONTROLLER_TYPE==0) //CU_v.4.3
	void initPWM()
	{
		DDR_595 |= (1<<OE_595);  //Конфигурация порта ШИМ на выход
		PORT_595 &=~ (1<<OE_595);

		//for Atmega_88
		// 	TCCR2=(1<<CS21);//|(1<<CS20); //
		// 	TCCR2=(1<<WGM21)|(1<<WGM20); //
		
		//for Atmega8A
		TCCR2=(1<<CS22)|(1<<CS20)|(1<<WGM21)|(1<<WGM20);
		TIMSK |= (1<<OCIE2)|(1<<TOIE2);
		
		OCR2=127;   //начальная значение яркости
	}

	#if (LED_TYPE==4) //на одно табло
		//ШИМ - установка индикации в 0
		ISR (TIMER2_COMP_vect)
		{	send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(0x00);
			send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(0x00);
			LATCH_PULSE();} //на два табло
		//ШИМ - установка индикации в 1
		ISR (TIMER2_OVF_vect)
		{
			PORT_OE |= (1<<OE_595);
			display_send(OE_count);
			display_send(OE_count); //на два табло
//			ADCSTART++;
		}
	#else
		ISR (TIMER2_COMP_vect){PORT_OE &=~ (1<<OE_595);}
		ISR (TIMER2_OVF_vect){PORT_OE |= (1<<OE_595);}
	#endif

#elif (CONTROLLER_TYPE==1) //Controll module: CM_1.3; CM_v.2.0
	void initPWM()
	{
		DDR_OE |= (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);  //Конфигурация портов ШИМ на выход
		PORT_OE &=~ (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);

		//for Atmega_88
		// 	TCCR2=(1<<CS21);//|(1<<CS20); //
		// 	TCCR2=(1<<WGM21)|(1<<WGM20); //
		
		//for Atmega8A
		TCCR2=(1<<CS22)|(1<<CS20)|(1<<WGM21)|(1<<WGM20);
		TIMSK |= (1<<OCIE2)|(1<<TOIE2);
		
		OCR2=127;   //начальная значение яркости
	}

	#if ((LED_TYPE==3)||(LED_TYPE==4))
		//установка портов индикации в 0
		ISR (TIMER2_COMP_vect){PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//установка портов индикации в 1
		ISR (TIMER2_OVF_vect){
			switch (OE_count){
				case 1:{display_send(OE_count);PORT_OE |= ((1<<OE0));break;}
				case 2:{display_send(OE_count);PORT_OE |= ((1<<OE1));break;}
				case 3:{display_send(OE_count);PORT_OE |= ((1<<OE2));break;}
				case 4:{display_send(OE_count);PORT_OE |= ((1<<OE3));break;}
			}
			OE_count++;
			if (OE_count<1) OE_count=4;
			if (OE_count>4) OE_count=1;
		}
	#elif (LED_TYPE==2)
		//установка портов индикации в 0
		ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//установка портов индикации в 1
		ISR (TIMER2_OVF_vect){
			for (int jp=0;jp<pwc;jp++){			//pwc - определяет количество одновременно включенных разрядов
				switch (OE_count+jp){
					case 1:{PORT_OE &=~ ((1<<OE0));break;}
					case 2:{PORT_OE &=~ ((1<<OE1));break;}
					case 3:{PORT_OE &=~ ((1<<OE2));break;}
					case 4:{PORT_OE &=~ ((1<<OE3));break;}
					case 5:{PORT_OE &=~ ((1<<OE0));break;}
					case 6:{PORT_OE &=~ ((1<<OE1));break;}
					case 7:{PORT_OE &=~ ((1<<OE2));break;}
					case 8:{PORT_OE &=~ ((1<<OE3));break;}
				}
			}
			OE_count++;
			if (OE_count<1) OE_count=4;
			if (OE_count>4) OE_count=1;
		}
	#elif (LED_TYPE==5)	//квазидинамическая индикация: поочередная загрузка разрядов (с учетом pwc), яркость устанавливается ШИМ по линии OE
		//установка портов индикации в 0
		ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//установка портов индикации в 1
		ISR (TIMER2_OVF_vect){
			PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));
			OE_count++;
			if (OE_count<1) OE_count=4;
			if (OE_count>4) OE_count=1;
			display_send(OE_count);
		}
	#else	//
		//установка портов индикации в 0
		ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//установка портов индикации в 1
		ISR (TIMER2_OVF_vect){
			PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));
			OE_count++;
			if (OE_count==0) display_send(1);		//посылка данных в регистры 1 раз в 256 запусков прерывания
		}
	#endif
#endif


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

//запись 2х байт в регистр сдвига
void send_data16 (uint16_t data)
{
	unsigned char i;
	for (i=0;i<16;i++)
	{
		if ((data&0x8000)!=0) PORT_595|=_BV(DATA);//Выставляем данные на PD0
		else PORT_595&=~_BV(DATA);
		CLK_PULSE();
		data=(data<<1);
	}
}

//на вход байт в 7сег. коде, D0 правый разряд табло 
void display_7code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0)
{
	#if (LED_TYPE==0)
	{
		send_data8(D[Dsort[0]]);
		send_data8(D[Dsort[1]]);
		send_data8(D[Dsort[2]]);
		send_data8(D[Dsort[3]]);
	}
	#elif (LED_TYPE==1)
	{
	}
	#elif (LED_TYPE==2)
	{
		send_data8(D[Dsort[0]]);
		send_data8(D[Dsort[1]]);
		send_data8_back(D[Dsort[2]]);
		send_data8_back(D[Dsort[3]]);
	}
	//для динамической индикации
	#elif (LED_TYPE==3)
	{
		//определение разрядов платы Table Digit LED-driver
		D[0]=D0;
		D[1]=D1;
		D[2]=D2;
		D[3]=D3;
	}
	//для платы IM-49 с обратным расположением сегментов - динамическая индикация
	#elif (LED_TYPE==4)
	{
		send_data8_back(D[Dsort[0]]);
		send_data8_back(D[Dsort[1]]);
		send_data8_back(D[Dsort[2]]);
		send_data8_back(D[Dsort[3]]);
	}
	#else
	{
		D[Dsort[0]]=D0;
		D[Dsort[1]]=D1;
		D[Dsort[2]]=D2;
		D[Dsort[3]]=D3;
	}
#endif	
	LATCH_PULSE();
}

//вывод на табло в десятичном представлении
//на вход байт как десятичное число, D0 правый разряд табло 
void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0)
{
	//Вывод на цельное табло "prodv2011" в коде ЁПРСТ
	#if (LED_TYPE==1)
	{
		int16_t D01=0;
		int16_t D23=0;
		D01|=Dig0[D0]|Dig1[D1];
		D23|=Dig2[D2]|Dig3[D3];
		send_data16(D23);
		send_data16(D01);
		LATCH_PULSE();
	} 
	//Вывод на табло в коде ABCD
	#else
	{
		//стандартное расположение разрядов
		D[Dsort[0]]=ABCD_TABLE[D0];
		D[Dsort[1]]=ABCD_TABLE[D1];
		D[Dsort[2]]=ABCD_TABLE[D2];
		D[Dsort[3]]=ABCD_TABLE[D3];
		#if (LED_TYPE==2) 
			display_send(0);
		#endif
	}
	#endif	   	
}

//вывод на табло в десятичном представлении - с учетом точки: point - маска точек
//на вход байт как десятичное число, D0 правый разряд табло
void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask)
{
	//Вывод на цельное табло "prodv2011" в коде ЁПРСТ
	#if (LED_TYPE==1)
	{
		int16_t D01=0;
		int16_t D23=0;
		D01|=Dig0[D0]|Dig1[D1];
		D23|=Dig2[D2]|Dig3[D3];
		send_data16(D23);
		send_data16(D01);
		LATCH_PULSE();
	}
	//Вывод на табло в коде ABCD
	#else
	{
		//стандартное расположение разрядов
		D[Dsort[0]]=ABCD_TABLE[D0]|((pointmask<<7)&0x80);		//сложно обойтись без сдвига, т.к. с разными масками устанавливаются разные биты
		D[Dsort[1]]=ABCD_TABLE[D1]|((pointmask<<6)&0x80);
		D[Dsort[2]]=ABCD_TABLE[D2]|((pointmask<<5)&0x80);
		D[Dsort[3]]=ABCD_TABLE[D3]|((pointmask<<4)&0x80);
		#if (LED_TYPE==2)
			display_send(0);
		#endif
	}
	#endif
}


//вывод на табло десятичного числа от 0 до 9999
void display_dnum(int32_t num)
{
	uint8_t D_0=(num%10);
	uint8_t D_1=(num/10);
	uint8_t D_2=D_1/10;
	uint8_t D_3=D_2/10;
	D_1=D_1%10;
	D_2=D_2%10;
	display_10code (D_3,D_2,D_1,D_0);
//	display_10code ((uint8_t)num/1000,(uint8_t)num/100%10,(uint8_t)num/10%10,(uint8_t)num%10);
}

//индикация динамическая (тестовый вариант)
void display_send(uint8_t dig_num)
{
	//Вывод на пиксельное табло в коде ABCDEF
	#if (LED_TYPE==0)
	{
		send_data8(D[0]); send_data8(D[1]); send_data8(D[2]); send_data8(D[3]);
		LATCH_PULSE();
	}
	//Вывод на плату IM-51 в коде ABCD+DCBA
	#elif (LED_TYPE==2)
	{
		send_data8(D[0]); send_data8(D[1]); 
		send_data8_back(D[2]); send_data8_back(D[3]);
		LATCH_PULSE();
	}
	#elif (LED_TYPE==3)
//		send_data8_back(ABCD_TABLE[D[dig_num-1]]|0x80);
		send_data8_back(D[dig_num-1]);
	#elif (LED_TYPE==4)
//			send_data8_back(D[dig_num-1]);
		{send_data8_back(D[0]);send_data8_back(D[1]);send_data8_back(D[2]);send_data8_back(D[3]);}
// 			switch (dig_num){
// 				case 1:{send_data8(0x00);send_data8(0x00);send_data8_back(D[2]);send_data8_back(D[3]);break;}
// 				case 2:{send_data8(0x00);send_data8_back(D[1]);send_data8_back(D[2]);send_data8(0x00);break;}
// 				case 3:{send_data8_back(D[0]);send_data8_back(D[1]);send_data8(0x00);send_data8(0x00);break;}
// 				case 4:{send_data8_back(D[0]);send_data8(0x00);send_data8(0x00);send_data8_back(D[3]);break;}
// 			}
	#elif (LED_TYPE==5)
		switch (pwc) 
		{
			case 1:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(D[3]);break;}
					case 2:{send_data8(0x00);send_data8(0x00);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(0x00);send_data8(D[1]);send_data8(0x00);send_data8(0x00);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(0x00);send_data8(0x00);break;}
				}
				break;
			case 2:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(0x00);send_data8(D[2]);send_data8(D[3]);break;}
					case 2:{send_data8(0x00);send_data8(D[1]);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(D[0]);send_data8(D[1]);send_data8(0x00);send_data8(0x00);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(0x00);send_data8(D[3]);break;}
				}
				break;
			case 3:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(D[1]);send_data8(D[2]);send_data8(D[3]);break;}
					case 2:{send_data8(D[0]);send_data8(D[1]);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(D[0]);send_data8(D[1]);send_data8(0x00);send_data8(D[3]);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(D[2]);send_data8(D[3]);break;}
				}
				break;
			case 4:
				switch (dig_num){
					default: {send_data8(D[0]);send_data8(D[1]);send_data8(D[2]);send_data8(D[3]);break;}
				}
				break;
		}
	#endif
	LATCH_PULSE();
}

//сортировка разрядов табло
void digit_sort(uint8_t dig3, uint8_t dig2, uint8_t dig1, uint8_t dig0){
	Dsort[0]=dig0;
	Dsort[1]=dig1;
	Dsort[2]=dig2;
	Dsort[3]=dig3;
} 

//установка значения pwc - количества одновременно включенных разрядов (при pwc=1 - аналог динамической индикации, pwc=4 - статической)
void set_PWC(uint8_t PWM_index){
	pwc=PWM_index;
}