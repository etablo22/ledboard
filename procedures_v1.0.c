/*
 * procedures.h
 *
 * Created: 10.02.2016 06:57:41
 *  Author: bya
 */

#include "procedures_v1.0.h"

void COMMANDS (int8_t func){
	switch (func){
		case BRIGHT:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значени¤ ¤ркости
				BriData = UARTdata;
				OCR2=BriData;
				WRITEEEBRI = 1;
				BRITEST=0;						//отключение изменение ¤ркости в режиме тестировани¤
			}
			_LED1(0);
			break;
		}
		case RXTDATA:{
			_LED1(1);
			int8_t j=0;
			while ((j<4)||(USART_GetRxCount()))
			{
				if (ASCIIDATA) DigRX = USART_GetChar()-0x30;	//чтение цифры из буфера - данные в формате ASCII
				else DigRX = USART_GetChar();	//чтение цифры из буфера - данные в формате DEC/HEX
				if (_CORRECT(DigRX)) Digit[j] = DigRX; //запись в ќ«”
				j++;
			}
			//						if (j==4) {WRITEEEDIG=1;}
			WRITEEEDIG=1;
			//						DISPLAY=1;
			TIMER1CNT=1;//дл¤ оптимизации кода используетс¤ универсальна¤ переменна¤ TIMER1CNT
			_LED1(0);
			display_10code(Digit[3],Digit[2],Digit[1],Digit[0]);
			break;
		}
		case INDICMODE:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значени¤ ¤ркости
				ind_mode = UARTdata;
				cli();
				eeprom_write_byte(EEdata+2,ind_mode);
				sei();
				DISPLAY=1;//дл¤ оптимизации кода используетс¤ универсальна¤ переменна¤ TIMER1CNT
			}
			_LED1(0);
			break;
		}
		case TESTMODEON:{
			_LED1(1);
			mode=6;
			TIMER1CNT=0;
			cli();
			eeprom_write_byte(EEdata+3,mode);
			sei();
			_LED1(0);
			break;
		}
		case TESTMODEOFF:{
			_LED1(1);
			mode=0;
			cli();
			eeprom_write_byte(EEdata+3,mode);
			sei();
			_LED1(0);
			break;
		}
		case RESET:{
			_LED1(1);
			SOFTRESET=1;
			_LED1(0);
			break;
		}
	}
}