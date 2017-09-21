/*
 * Stela_4.c
 *
 * Created: 18.01.2016 9:11:19
 * Autor: BYA
 * controller for LED-indicator (AZS tablo)

 tab_control : версия для ведущего табло: установка яркости по измерению освещенности на общих табло

 20151109	: добавление обработчика нажатия на кнопку №0 (PC0 -> 0)
 20160316	: замена кнопки BTN0 на BTN1 из-за ошибки изготовления платы: не подведена дорожка к PC0
 			: исключаем ошибку случайного замыкания кнопки при сбросе питания (например, из-за снега или дождя)
 			: при долгом нажатии кнопки - выход из режима инициализации
 20160320	: в итоговой версии убрано изменение адреса табло по нажатию кнопки
 20160520	: производителем (PSElectro-Электроконнект) на платах восстановлена дорожка к PC0 МК
 20160929	: расположение разрядов согласно установке digit_sort(1,0,3,2) в _UPDATEDATA
 20161012	: исправления в библиотеке Ind_lib (v.2.0:режим поразрядной индикации)
			: добавлена процедура set_pwc - установка значения pwc

 */ 

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "ADC_lib.h"
#include "usart.h"

#include "Ind_lib_v2.0.h"

//команды протокола
#define BRIGHT		0xA0	//команда установить значение яркости = 160
#define BRIGHTDWN	0xA1	//команда уменьшить значение яркости = 161
#define BRIGHTUP	0xA2	//команда увеличить значение яркости = 162
#define RXNtab		0xA9	//команда установить количество табло
#define RXTDATAall	0xAA	//команда приема данных для всех табло 32Б (170)
#define RXTDATA		0xAB	//команда приема данных по адресу (171)
#define SETINDIC	0x96	//установить (показать) режим индикации
#define SETDSORT	0x97	//установить (показать) порядок сортировки разрядов табло
#define SETMODE		0x98	//установить (показать) режим работы
#define SETTADR		0x99	//установить (показать) адрес табло
#define RESET		0xFE	//программный сброс модуля

#define TESTMODEON 0x97

//адреса устройств сети
#define BROADCAST	0xFF	//адрес широковещательной передачи = 0
#define CUADR		0x64	//адрес блока управления (CUnit)
#define TADR0		0x64	//адрес нулевого табло (начало отсчета)
#define TADR		101		//адрес табло

//параметры
#define PORTLED		PORTD	//порт индикатора
#define DDRLED		DDRD	//порт индикатора
#define LED1		PD4		//порт индикатора
//#define LED2		PD5		//порт индикатора
#define DDR_RS485	DDRD	//порт интерфейса
#define PORT_RS485	PORTD	//порт интерфейса
#define PTXEN		PD2		//порт интерфейса
#define PRXDIS		PD3		//порт интерфейса
#define RXD			PD0		//вход UART - приемник
#define TXD			PD1		//выход UART - передатчик
#define MAXINDMODE	1		//количество режимов индикации

// пороги уровней освещенности
#define LUM1		5000
#define LUM2		30000

// пороги перехода времени суток
#define MORNING_HOUR	7
#define EVENING_HOUR	20
#define NIGHT_HOUR		23


int8_t j=0,it1=0,mode=0,ind_mode=0;
uint8_t dispcounter,OCRtest,TIMER1CNT;
uint8_t cnt_btn0,cnt_btn1;
int8_t TestCNT, INITIALIZATION=0;

uint8_t Digit[4]={1,2,3,4}, PointMask=0x0F;
uint8_t EEMEM EEDigit[4]={1,2,3,4}, EEDsort[5]={0,1,2,3,0x0F}, EEdata[5]={1,255,0,0,101};//, EEBri[1]={255}, EE_indmode[1]={0}, EEmode[1]={0};

// uint8_t EEMEM EEBriData[3] = {12,40,86}; //уровень яркости {ночной,средний, дневной}
// uint8_t BriMode,preBriMode, BriLevels[3]={12,40,86}, BriValues[12]={9,12,16,20,25,32,40,49,60,73,86,100}, BriStep;

//FYLS-5050LULW3C R150 pwc=2 - максимум 5А 60Вт (310LUX)
//FYLS-5050UR3C R300 pwc4 - максимум 6А 75Вт (170LUX)
// uint8_t EEMEM EEBriData[3] = {17,65,146}; //уровень яркости {ночной,средний, дневной}
// uint8_t BriMode,preBriMode, BriLevels[3]={17,65,146}, BriValues[12]={10,13,17,22,29,38,50,65,85,112,146,200}, BriStep;

//FYLS-5050UR3C R300 pwc4 - максимум 8А 100Вт (200LUX)
//FYLS-5050LULW3C R150 pwc=2 - максимум 6А 75Вт (350LUX)
//FYLS-5050LULW3C R300 pwc=4 - максимум _А _Вт (___LUX)
uint8_t EEMEM EEBriData[3] = {20,95,214}; //уровень яркости {ночной,средний, дневной}
uint8_t BriMode,preBriMode, BriLevels[3]={20,95,214}, BriValues[12]={20,30,40,50,62,77,95,118,146,181,214,250}, BriStep;


//EEdata - 4Б: данные для настройки табло
//{номер табло, яркость, режим индикации, режим работы,адрес табло} (номер и адрес табло - константы для надежности)

uint8_t getADR, UARTcommand, UARTdata, ADCch;
//флаги работы программы
int8_t CORRECT,TXBRIDATA,TXTAB,TXDATAEN,DISPLAY,SOFTRESET,CHBRI,BRITEST,TESTMODE;
uint8_t ADCENABLE,ADCSTART;
int8_t WREEN,WRITEEEDIG,WRITEEEBRI,WRITEEENTAB,READEEBRI,READEEDIG,READEEDSORT;
int8_t PRESSBTN0,PRESSBTN1,REPRESSBTN0,REPRESSBTN1;//,INITIALIZATION;
int8_t SETBRITIME;

static volatile uint8_t rxCount;
extern uint16_t v_ADC, adc_counter;

uint8_t hour=12, min=00, sec=00;

uint8_t _CONVERT_pult_code(uint8_t code)
{
	switch (code)
	{
		case 222:	{return 0;break;}
		case 24:	{return 1;break;}
		case 205:	{return 2;break;}
		case 93:	{return 3;break;}
		case 27:	{return 4;break;}
		case 87:	{return 5;break;}
		case 215:	{return 6;break;}
		case 28:	{return 7;break;}
		case 223:	{return 8;break;}
		case 95:	{return 9;break;}
		case 1:		{return 10;break;}
		default:	{return 11;break;}
	}
}

uint8_t _ascii2dec(uint8_t code)
{
	switch (code)
	{
		case 0x2D:		{return 10;break;}
		case 0x30:		{return 0;break;}
		case 0x31:		{return 1;break;}
		case 0x32:		{return 2;break;}
		case 0x33:		{return 3;break;}
		case 0x34:		{return 4;break;}
		case 0x35:		{return 5;break;}
		case 0x36:		{return 6;break;}
		case 0x37:		{return 7;break;}
		case 0x38:		{return 8;break;}
		case 0x39:		{return 9;break;}
		default:		{return 11;break;}
	}
}

uint8_t _ABCD2dec(uint8_t code)
{
	switch (code)
	{
		case 0x3F:		{return 0;break;}
		case 0x06:		{return 1;break;}
		case 0x5B:		{return 2;break;}
		case 0x4F:		{return 3;break;}
		case 0x66:		{return 4;break;}
		case 0x6D:		{return 5;break;}
		case 0x7D:		{return 6;break;}
		case 0x07:		{return 7;break;}
		case 0x7F:		{return 8;break;}
		case 0x6F:		{return 9;break;}
		case 0x40:		{return 10;break;}
		case 0x00:		{return 11;break;}
		case 0x63:		{return 12;break;}
		default:		{return 0x11;break;}
	}
}

//конвертер DEC  -> BCD
char _dec2bcd(char num)
{
	return ((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
int16_t _bcd2dec(int16_t num)
{
	return ((num/16 * 10) + (num % 16));
}

//проверка и корректировка данных для табло
uint8_t _CORRECT(uint8_t code, uint8_t digit, int8_t codetype)
{
	uint8_t tdata=code;
	code=digit;
	//codetype: тип кодировки
	//0: неопределен
	//1: десятичный (свой шрифт)
	//2: ASCII
	//3: ABCD - код 7-сегментного индикатора
	//4: Pult_code - кодировка пульта (Кузьмин П.В.)
	switch (codetype) {
		case 1: if ((tdata>=0)&&(tdata<=MAXDIGNUMBER)) code=tdata;break;	//корректное значение цифры
		case 2: code=_ascii2dec(tdata);break;								//преобразование из формата ASCII
		case 3: code=_ABCD2dec(tdata);break;								//преобразование из кода 7-сегм.
		case 4: code=_CONVERT_pult_code(tdata);break;						//преобразование из кода Pult_code
		default: {
			if ((tdata>=0)&&(tdata<=MAXDIGNUMBER)) code=tdata;				//корректное значение цифры
			else if ((tdata>0x29)&&(tdata<0x40)) code=_ascii2dec(tdata);			//преобразование из формата ASCII
			break;
		}
	}
	return code;										//некорректное значение заменяем на digit
}

//переключение индикатора LED1 на порту PORTLED
void _LED1(int8_t mLED1)
{
	switch (mLED1){
		case 0: {PORTLED|= _BV(LED1); break;}
		case 1: {PORTLED&=~_BV(LED1); break;}
		default:{break;}
	}
}

// void _LED2(int8_t mLED2)
// {
// 	switch (mLED2){
// 		case 0: {PORTLED &= ~_BV(LED2); break;}
// 		case 1: {PORTLED |=  _BV(LED2); break;}
// 		default:{break;}
// 	}
// }

//мигание индикатора LED1: num раз с задержкой delays5ms, которая указывается в мс
void _flash_LED1(uint8_t num, uint8_t delays5ms)
{
	delays5ms=delays5ms/5;	//задержка кратна 5мс
	_LED1(0);
	//в функции _delay_ms() из библиотеки <util/delay.h> в качестве аргумента не может быть указана переменная,
	//		указывается только константа: поэтому задержка по циклу
	for (int n=0;n<num;n++)
	{
		for (int dl=0;dl<delays5ms;dl++) _delay_ms(5);
		_LED1(1);
		for (int dl=0;dl<delays5ms;dl++) _delay_ms(5);
		_LED1(0);
	}
}

//определение номера шага по яркости
uint8_t _getBriStep(uint8_t level)
{
	uint8_t bs=0;
	while ( bs<(sizeof(BriValues)-1) && BriValues[bs]<level )
	{
		bs++;
	}
	return bs;
}

//управление интерфейсом UART-RS485
void _RS485 (int8_t type)
{
	switch(type){
		case 0: {
			// type=="INIT"	//инициализация UART-RS485
			DDR_RS485 |= (1<<PTXEN)|(1<<PRXDIS);  //Конфигурация управления интерфейсом UART
			UCSRB |= (1<<TXCIE);
			sei();

			//дополнительно
			MCUCR &=~(1<<PUD);	//включение внутренних подтягивающих резисторов на портах
			PORT_RS485 |= (1<<RXD);	//подтягивающий резистор на линии RXD для снижения помех
			break;
		}
		case 1:{
			//type=="RXEN"	//разрешение приема по RS485
			PORT_RS485 &=~ (1<<PRXDIS);
			break;
		}
		case 2:{
			//type=="TXEN"	//разрешение передачи по RS485
			PORT_RS485 |= (1<<PTXEN);
			break;
		}
		case 10:{
			//type=="RXDIS"	//запрещение приема
			PORT_RS485 |= (1<<PRXDIS);
			break;
		}
		case 20:{
			//type=="TXDIS"	//запрещение передачи
			PORT_RS485 &=~ (1<<PTXEN);
			break;
		}
		case 11:{
			//type=="RXEN-TXDIS"	//разрешение приема и запрет передачи
			PORT_RS485 &=~ (1<<PTXEN);
			PORT_RS485 &=~ (1<<PRXDIS);
			break;
		}
		case 22:{
			//type=="TXEN-RXDIS"	//разрешение передачи и запрет приема
			PORT_RS485 |= (1<<PTXEN);
			PORT_RS485 |= (1<<PRXDIS);
			break;
		}
	}
}

//по окончанию передачи отключение передатчика
ISR(USART_TXC_vect)
{
	_RS485(11);		//отключить передачу, включить прием
	WREEN |= 0x01;			//разрешить запись в ЕЕ
}

//настройка таймера для запуска АЦП
void initADC_T0()
{
	//таймер Т0
	TCCR0 |= (1<<CS01); //предделитель CLK/8
	TIMSK |= (1<<TOIE0); //прерывание для запуска АЦП
}


ISR(TIMER0_OVF_vect){
	//флаг запуска АЦП
	ADCSTART++;
}

//настройка таймера Т1 
//		Т1 используется для периодического обновления данных на табло
void initT1(uint8_t TCRB)
{
	TCCR1A=0;
	TCCR1B=TCRB;//(1<<CS12)|(1<<CS10);
	TIMSK |= (1<<TOIE1);
	TIMER1CNT=0;
}

//по переполнению Т1 
//		:инкремент переменной TIMER1CNT - счетчик периодов Т1
//		:обновляются данные на табло
//		:обновление данных в режиме тестирования табло
ISR (TIMER1_OVF_vect){
	TIMER1CNT++;
	DISPLAY=1;
}

//установка режима и яркости по времени или освещенности
void set_Bright(uint16_t val, uint8_t param)
{
	switch (param) {
		//установка режима яркости по номеру
		case 0:{
			BriMode = val;
			break;
		}
		//установка режима яркости по времени
		case 1: {
			if (CHBRI) {WRITEEEBRI=1;}
			else
			{
				if (val>=MORNING_HOUR&&val<EVENING_HOUR) {BriMode=0;}
				else if (val>=EVENING_HOUR&&val<NIGHT_HOUR) {BriMode=1;}
				else if ((val>=NIGHT_HOUR&&val<=0)||(val>0&&val<MORNING_HOUR))	{BriMode=2;}
				TXBRIDATA=1;
			}
			break;
		}
		//установка режима яркости по значению освещенности
		case 2:{
			if (CHBRI) {WRITEEEBRI=1;}
			else
			{
				if (val<(LUM1-500)) {BriMode=0;}
				else if ((val>LUM1+500)&&(val<(LUM2-1000))) {BriMode=1;}
				else if (val>(LUM2+1000)) {BriMode=2;}
				TXBRIDATA=1;
			}
			break;
		}
		//действия при изменении яркости по команде:
		//сброс измерения освещенности и настройка записи режимов яркости в ЕЕ
		case 3:{
			ADCENABLE=0;	//остановка автоматического изменения яркости, запуск через неск.секунд в модуле if(DISPLAY)
// 			ADCSTART=0;		//сброс старта АЦП для временного отключения автоматического изменения яркости
// 			adc_counter = 0; v_ADC=0;	//сброс ADC 
			WRITEEEBRI = 0;	//исключить лишнюю запись в ЕЕ -
			CHBRI = 1;		//разрешение WRITEEEBRI через неск.секунд в case 1 и case 2
			TXBRIDATA=1;	//отправка значения яркости на ведомые табло
			break;
		}
	}
	OCR2=BriLevels[BriMode];		//установка яркости по выбранному режиму
	BriStep=_getBriStep(BriLevels[BriMode]);
}

//команды
void COMMANDS (uint8_t func){
	switch (func){
		//установка яркости табло
		case BRIGHT:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значения яркости
				BriLevels[BriMode] = UARTdata;
				set_Bright(BriLevels[BriMode],3);
			}
			_LED1(0);
			break;
		}
		//Увеличить яркость на ед.градации 
		case BRIGHTUP:{
			_LED1(1);
			if (BriStep<(sizeof(BriValues)-1)) {
				BriStep++;
				BriLevels[BriMode]=BriValues[BriStep];
				set_Bright(BriLevels[BriMode],3);
			}
			_LED1(0);
			break;
		}
		case BRIGHTDWN:{
			_LED1(1);
			if (BriStep>0) {
				BriStep--;
				BriLevels[BriMode]=BriValues[BriStep];
				set_Bright(BriLevels[BriMode],3);
			}
			_LED1(0);
			break;
		}
		//прием данных для установки значений на табло
		case RXTDATA:{
			_LED1(1);
			j=0;
			while ((j<4)&&(USART_GetRxCount()))
			{
// 				if (ASCIIDATA) DigRX = USART_GetChar()-0x30;	//чтение цифры из буфера - данные в формате ASCII
// 				else DigRX = USART_GetChar();	//чтение цифры из буфера - данные в формате DEC/HEX
				UARTdata = USART_GetChar();	//чтение цифры из буфера
				Digit[j] = _CORRECT(UARTdata,Digit[j],0); //запись в ОЗУ скорректированного значения
				j++;
			}
			//						if (j==4) {WRITEEEDIG=1;}
			WRITEEEDIG=1;	//записать значение цены в ЕЕ
			DISPLAY=1;		//обновить инф. на табло
			_LED1(0);
// 			display_10code(Digit[3],Digit[2],Digit[1],Digit[0]);
// 			if ((TXDATAEN&0x01)&&(TXDATAEN&0x02)) {
// 				WREEN &=~ 0x01;		//исключить запись ЕЕ во время передачи данных
// 				_RS485(22);
// 				USART_PutChar((char) Ntab);
// 				USART_PutChar((char) TADR);
// 				USART_PutChar((char) RXTDATA);
// 				for (j=0;j<4;j++) USART_PutChar((char) Digit[j]);
// 				USART_PutChar((char) PointMask);
// 			}
			break;
		}
		//установка режима индикации - заблокировано
		//показать режим индикации
		case SETINDIC:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значения режима индикации
			}
			display_dnum(ind_mode);
			TCNT1=0; DISPLAY=0;					//задержать обновление табло
			_LED1(0);
			break;
		}
		//установка режима работы табло - заблокировано
		//показать номер режима работы
		case SETMODE:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значения режима работы
			}
			display_dnum(mode);					//показать номер режима
			TCNT1=0; DISPLAY=0;					//задержать обновление табло
			_LED1(0);
			break;
		}
		//отобразить номер табло (установка заблокирована)
		case SETTADR:{
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение адреса
			}
			display_dnum(TADR);					//показать адрес табло
			TCNT1=0; DISPLAY=0;					//задержать обновление табло
			break;
		}
		//программный сброс
		case RESET:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение оператора команды
				if (UARTdata==1) SOFTRESET=1;
			}
			_LED1(0);
			break;
		}
	}
}

void _SOFTRESET(void)
{
//	WDTCSR=0x00;
	
	DDRLED|=_BV(LED1);//|_BV(LED2);
	_flash_LED1(3,100);

	mode=0;ind_mode=0;

	cli();
	for (j=0;j<4;j++)
	{
		eeprom_write_byte(EEDigit+j,j+1);
	}
	for (j=0;j<4;j++)
	{
		Digit[j]=eeprom_read_byte(EEDigit+j);
	}
	sei();

	display_10code(Digit[0],Digit[1],Digit[2],Digit[3]);
}

//установка начальных значений переменных
void _UPDATEDATA(void)
{
	DDRLED|=_BV(LED1);//|_BV(LED2);

	//сортировка разрядов табло
	digit_sort(0,1,2,3);
	//установка режима поразрядной индикации
	set_PWC(2);
	
	j=0;it1=0;
	
	CHBRI=0;
	ADCENABLE=0;			//разрешить преобразования АЦП
	ADCch=7;
	TXDATAEN=3;
	WREEN=1;					//разрешить запись в ЕЕ
	WRITEEENTAB=0;
	WRITEEEDIG=0;
	WRITEEEBRI=0;
	CORRECT=1;
	READEEDIG=1;
	READEEBRI=1;
	SOFTRESET=0;
	
	BriMode=2;
	BriStep=_getBriStep(BriLevels[BriMode]);
	set_Bright(BriLevels[BriMode],3);
	
	BRITEST=1;
	init595();
	initPWM();
	initT1(0x05);
	initADC_T0();
	adc_init();
	USART_Init(USART_DOUBLED, 19200);
	USART_FlushTxBuf();

	display_dnum(TADR);
	
	TXDATAEN |= 0x02;	//разрешение TX глобально (2й бит)
//	TXDATAEN &=~ 0x02;	//запрещение TX глобально (2й бит)
	WREEN |= 0x02;		//разрешить запись яркости в ЕЕ

	_RS485(0); //инициализация интерфейса RS485
	_RS485(11);	 //RX_EN + TX_DIS

	_flash_LED1(3,50);
}




/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
int main(void)
{
	_UPDATEDATA();

    while(1)
    {
		//программный сброс
		if (SOFTRESET) {
			if (WDTCR&(1<<WDE)) while(1);
			else {_UPDATEDATA();_SOFTRESET();}
		}
			
		//обновление данных на табло по переполнению таймера Т1
		if (DISPLAY){
			_LED1(1);
			DISPLAY=0;
			display_10code_point(Digit[0],Digit[1],Digit[2],Digit[3],0x0F);	//прямое отображение
			ADCENABLE=1;
 			_delay_ms(1);
			_LED1(0);
		}
		
	
		//запуск измерений АЦП один раз в 256 периодов таймера
		if ((ADCSTART==255)&&ADCENABLE) 
		{
			ADCSTART=0;
			//суммирование результата измерения освещенности по номеру канала с датчиком
			v_ADC+=ADC_result(ADCch)/4;	//делитель на 4 для исключения риска переполнения
			adc_counter++; //набираем количество измерений для определения среднего значения

			//изменение яркости в соответствии с измеренной освещенностью
			if (adc_counter>200)
			{
					//отправить в UART измерения АЦП
// 					TXDATAEN=0;
// 					WREEN=0; //запретить запись в ЕЕ на время передачи данных
// 					_RS485(22);
// 					USART_PutChar((char) ((v_ADC&0xFF00)>>8));
// 					USART_PutChar((char) (v_ADC&0x00FF));
				set_Bright(v_ADC, 2);		//установить режим яркости по освещенности
				if (BriMode!=preBriMode)	//если режим яркости изменился
				{
					preBriMode=BriMode;
					TXDATAEN |= 0x01;
				}
//				else TXDATAEN &=~(0x01);	//запрещение TX при сохранении режима яркости
				adc_counter=0;
				v_ADC=0;
				_flash_LED1(3,20);
			}
		}

		//отправка широковещ-й команды установки яркости на табло
		if (TXBRIDATA&&(TXDATAEN&0x01)&&(TXDATAEN&0x02))		//фильтр с проверкой двух младших битов
		{
			TXBRIDATA=0;
			WREEN &=~ 0x01; //запретить запись в ЕЕ на время передачи данных
			_RS485(22);
			USART_PutChar(BROADCAST);
			USART_PutChar(BRIGHT);
			USART_PutChar(BriLevels[BriMode]);
		}

		//чтение значения яркости из ЕЕ в ОЗУ 
		if (READEEBRI){
			_LED1(1);
			READEEBRI=0;
			cli();
			//считываем значения трех уровней яркости из ЕЕ
			BriLevels[0]=eeprom_read_byte(EEBriData); //ночная яркость
			BriLevels[1]=eeprom_read_byte(EEBriData+1); //яркость утром и вечером
			BriLevels[2]=eeprom_read_byte(EEBriData+2); //дневная яркость
			sei();
			set_Bright(1,0);
			_LED1(0);
		}

		//чтение данных табло в ОЗУ из ЕЕ
		if (READEEDIG){
			_LED1(1);
			READEEDIG=0;
			cli();
			for (j=0;j<4;j++)
			{
				Digit[j]=eeprom_read_byte(EEDigit+j);
			}
			sei();
			_LED1(0);
		}

		//запись данных табло в ЕЕ
		if (WRITEEEDIG&&(WREEN&0x01)){
			_LED1(1);
			WRITEEEDIG=0;
			cli();
			for (j=0;j<4;j++)
			{
				eeprom_write_byte(EEDigit+j,Digit[j]);
			}
			sei();
			_LED1(0);
		}
		
		//запись значения яркости в ЕЕ
		if (WRITEEEBRI&&(WREEN&0x03)){
			_LED1(1);
			WRITEEEBRI=0;
			CHBRI=0;
			cli();
			eeprom_write_byte(EEBriData+BriMode,BriLevels[BriMode]); //сохранение яркости только для текущего режима
// 			eeprom_write_byte(EEBriData+1,BriLevels[1]);
// 			eeprom_write_byte(EEBriData+2,BriLevels[2]);
			sei();
			_LED1(0);
		}

		if (USART_GetRxCount()){	//проверка наличия данных в буфере приема UART
			_LED1(1);
			CORRECT=1;
			getADR = USART_GetChar();	//чтение байта 1: адрес
			if (getADR!=BROADCAST) {	//если не широковещательный
				if (getADR==TADR) {		//сравнение с адресом контроллера
	  				_LED1(1);			//выполнение при совпадении адреса
 	 				if (USART_GetRxCount()) UARTcommand = USART_GetChar();	//чтение адресной команды
					else {
						_delay_ms(200);		//дополнительная задержка для загрузки байта
						if (USART_GetRxCount()) UARTcommand = USART_GetChar();
						else CORRECT=0;
					}
 					_LED1(0);
				}
				else CORRECT=0;
			}
			else if (USART_GetRxCount()){	//если широковещательный адрес
				UARTcommand = USART_GetChar();	//чтение широковещательной команды
				}
			else {
				_delay_ms(200);			//дополнительная задержка для загрузки байта
				if (USART_GetRxCount()) UARTcommand = USART_GetChar();
				else CORRECT=0;
			}
			
			if (CORRECT){
				COMMANDS (UARTcommand);		//если данные корректны, то выполнить команду
 			}
  			_LED1(0);
		}
	}
				
}