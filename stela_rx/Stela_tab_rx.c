/*
 * Stela_4.c
 *
 * Created: 18.01.2016 9:11:19
 * Autor: BYA
 * controller for LED-indicator (AZS tablo)
 20151109	: добавление обработчика нажатия на кнопку №0 (PC0 -> 0)
 20160316	: замена кнопки BTN0 на BTN1 из-за ошибки изготовления платы: не подведена дорожка к PC0
 			: исключаем ошибку случайного замыкания кнопки при сбросе питания (например, из-за снега или дождя)
 			: при долгом нажатии кнопки - выход из режима инициализации
tab_rx		: версия для общего табло: только прием данных

20161012	: исправления в библиотеке Ind_lib (v.2.0:режим поразрядной индикации)
			: добавлена процедура set_pwc - установка значения pwc
			
20181303	: добавлена процедура установки номера табло с кнопки
			: параметр поразрядной индикации всегда устанавливается 4 (в _UPDATEDATA() -> pwc(4) )
	
	!!! в дальнейшем исключить pwc и оставить статическую (одновременно все 4 разряда) индикацию

 */ 

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

//#include "ADC_lib.h"
#include "usart.h"
//#include "procedures_v1.0.h"
//#include "ds18b20.h"

#include "Ind_lib_v2.2.h"

//команды протокола
#define BRIGHT		0xA0	//команда установить значение яркости = 160
#define BRIGHTDWN	0xA1	//команда уменьшить значение яркости = 161
#define BRIGHTUP	0xA2	//команда увеличить значение яркости = 162
#define RXNtab		0xA9	//команда установить количество табло
#define RXTDATAall	0xAA	//команда приема данных для всех табло 32Б (170)
#define RXTDATA		0xAB	//команда приема данных по адресу (171)
#define RXWRITEEE   0xAC    //команда на запись в ее установленной цены
#define SETINDIC	0x96	//установить (показать) режим индикации
#define SETDSORT	0x97	//установить (показать) порядок сортировки разрядов табло
#define SETMODE		0x98	//установить (показать) режим работы
#define SETTADR		0x99	//установить (показать) адрес табло
#define RESET		0xFE	//программный сброс модуля

//адреса устройств сети
#define BROADCAST	0xFF	//адрес широковещательной передачи = 0
#define CUADR		0x64	//адрес блока управления (CUnit)
#define TADR0		0x64	//адрес нулевого табло (начало отсчета)


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

uint8_t TADR = 101;		//адрес табло
const uint8_t MAXNTAB = 12;
uint8_t Ntab = 1, INITtab = 0, qtTab = 0;
uint8_t EEMEM EETab[3] = {1, 101, 5};		//Ntab - номер табло, TADR - адрес табло, qtTab - количество табло

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
uint8_t EEMEM EEBriData[3] = {5,95,214}; //уровень яркости {ночной,средний, дневной}
uint8_t BriMode,preBriMode, BriLevels[3]={5,95,214}, BriValues[12]={5,30,40,50,62,77,95,118,146,181,214,250}, BriStep;

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
//		case 0x20:		{return 11;break;}		//пробел - пустой
//		case 0x2A:		{return 12;break;}		//звездочка - знак градуса 
//		case 0x2B:		{return 11;break;}		//плюс - 
//		case 0x2C:		{return 13;break;}		//запятая - нижнее подчеркивание
		case 0x2D:		{return 10;break;}		//минус - дефис
//		case 0x2E:		{return 14;break;}		//точка - пустой с точкой
//		case 0x2F:		{return 11;break;}		//слэш - 
		case 0x30:		{return 0;break;}		//
		case 0x31:		{return 1;break;}		//
		case 0x32:		{return 2;break;}		//
		case 0x33:		{return 3;break;}		//
		case 0x34:		{return 4;break;}		//
		case 0x35:		{return 5;break;}		//
		case 0x36:		{return 6;break;}		//
		case 0x37:		{return 7;break;}		//
		case 0x38:		{return 8;break;}		//
		case 0x39:		{return 9;break;}		//
		default:		{return 11;break;}		//пустой - null
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
	//4: Pult_code - кодировка пудьта (Кузьмин П.В.)
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




//команды
void COMMANDS (uint8_t func){
	switch (func){
		//установка яркости табло
		case BRIGHT:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//чтение значения яркости
				OCR2=UARTdata;		//установка яркости
				BriStep=_getBriStep(UARTdata);
			}
			_LED1(0);
			break;
		}
		//Увеличить яркость на ед.градации 
		case BRIGHTUP:{
			_LED1(1);
			if (BriStep<(sizeof(BriValues)-1)) {
				BriStep++;
				OCR2=BriValues[BriStep];	//установка яркости 
			}
			_LED1(0);
			break;
		}
		case BRIGHTDWN:{
			_LED1(1);
			if (BriStep>0) {
				BriStep--;
				OCR2=BriValues[BriStep];	//установка яркости
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
				UARTdata = USART_GetChar();	//чтение цифры из буфера
				Digit[j] = _CORRECT(UARTdata,Digit[j],0); //запись в ОЗУ скорректированного значения
				j++;
			}
			DISPLAY=1;		//обновить инф. на табло
			_LED1(0);
			break;
		}
		case RXWRITEEE: {
			WRITEEEDIG = 1;
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

//настройка параметров обработки нажатия кнопок на портах PC0 и PC1
void initBTN()
{
	//таймер Т0 для считывания нажатия кнопки
	TCCR0 |= (1<<CS01); //предделитель CLK/8
	TIMSK |= (1<<TOIE0); //прерывание для проверки нажатия
	DDRC &= ~((1<<PC0)|(1<<PC1)); //порты в режим входа
	PORTC &= ~((1<<PC0)|(1<<PC1));
	PRESSBTN0 = 0;
	PRESSBTN1 = 0;
	REPRESSBTN0 = 1;
	REPRESSBTN1 = 1;
}

ISR(TIMER0_OVF_vect){
	//проверка нажатия кнопки PC0
	if (PINC&0x01) { //если PINC0=1, то нет нажатия
		cnt_btn0=0;
		PRESSBTN0 = 0; //кнопка отпущена
	}
	else {			//иначе - кнопка нажата
		cnt_btn0++;  //счетчик задержки нажатия для исключения дребезга
		if (cnt_btn0>100)	PRESSBTN0 = 1;	//кнопка нажата
	}
	
	//проверка нажатия кнопки PC1
	if (PINC&0x02) { //если PINC1=1, то нет нажатия
		PRESSBTN1 = 0;	//кнопка отпущена
		cnt_btn1=0;
	}
	else {			//иначе - кнопка нажата
		cnt_btn1++;  //счетчик задержки нажатия для исключения дребезга
		if (cnt_btn1>100)	PRESSBTN1 = 1;
	}
}

//установка начальных значений переменных
void _UPDATEDATA(void)
{
	DDRLED|=_BV(LED1);//|_BV(LED2);
	
	Ntab = eeprom_read_byte(EETab);
	TADR = eeprom_read_byte(EETab + 1);		//значение адреса табло
	qtTab = eeprom_read_byte(EETab + 2);		//значение количества табло
	
	//сортировка разрядов табло
	digit_sort(2,3,0,1);	//IM-49
	// digit_sort(3,2,1,0);	//DLD_0.6
	//установка режима поразрядной индикации
	set_PWC(4);
	
	j = 0; it1 = 0;
	
	CHBRI=0;
	WREEN=1;					//разрешить запись в ЕЕ
	WRITEEENTAB=0;
	WRITEEEDIG=0;
	CORRECT=1;
	READEEDIG=1;
	READEEBRI=1;
	SOFTRESET=0;
	
	BriMode=2;
	BriStep=_getBriStep(BriLevels[BriMode]);
	OCR2=BriValues[BriStep];	//установка яркости
	
	init595();
	initPWM();
	initT1(0x05);
	initBTN();
	USART_Init(USART_DOUBLED, 19200);
	USART_FlushTxBuf();

	display_dnum(TADR);
	
	WREEN = 1;		//разрешить запись в ЕЕ

	_RS485(0); //инициализация интерфейса RS485
	_RS485(11);	 //RX_EN + TX_DIS

	_flash_LED1(3,50);
}




/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
int main(void)
{
	_UPDATEDATA();
	setBoardAddr(); //установка адреса табло перед главным циклом

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
 			_delay_ms(1);
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

void setBoardAddr() {
	
initT1(0x00);	//остановка таймера для исключения ложного срабатывания
TIMER1CNT=0;
REPRESSBTN1=1;
while (PRESSBTN1) //пока кнопка BTN1 не будет отпущена
{
	//вход в режим инициализации - нажатие кнопки в течение 3 сек
	//исключаем ошибку случайного замыкания кнопки при сбросе питания (например, из-за снега или дождя)
	//при нажатии кнопки более 15ти сек - выход из режима инициализации
	if (REPRESSBTN1)		//однократное выполнение при длительном нажатии
	{
		_LED1(1);
		initT1(0x05);		//запускаем таймер длительности нажатия
		REPRESSBTN1 = 0;
	}
	if (TIMER1CNT == 1)		//подтверждение запуска инициализации при удержании кнопки примерно 1 сек
	{
		TIMER1CNT = 2;
		INITIALIZATION = 3;	//установка параметра для инициализации табло
		display_7code(0x08,0x08,0x08,0x08);
		_flash_LED1(1, 200);
	}
	if (TIMER1CNT > 4)		//слишком длительное нажатие
	{
		INITIALIZATION = 0;	//пропуcтить инициализацию при превышении времени удержания кнопки
		TIMER1CNT = 0;
		_flash_LED1(5, 50);
		break;				//выход из цикла и запуск работы табло
	}
	_delay_ms(10); //без этой задержки не выходит из цикла при отпускании кнопки PRESSBTN, возможно из-за оптимизации кода компилятором
}
REPRESSBTN1=1;
_LED1(0);

//выполнить инициализацию - определение номера табло Ntab
while(INITIALIZATION)
{
	//при нажатии BTN1 инкремент Ntab
	if (PRESSBTN1)
	{
		if (REPRESSBTN1)
		{
			_flash_LED1(1,20);
			REPRESSBTN1=0;
			WRITEEENTAB=1;
			if (Ntab<12)
			{
				Ntab++;	//максимальный номер табло 12
			}
			else
			{
				Ntab=0;
			}
			display_dnum(Ntab);
			INITIALIZATION=2;		//сброс выхода из инициализации
			TIMER1CNT=0; TCNT1=0;		//сброс счетчика Т1
		}
	}
	else REPRESSBTN1=1;
	
	//индикация номера табло и отсчет завершения инициализации
	if (TIMER1CNT)
	{
		if (Ntab) _flash_LED1(Ntab,200);
		else _flash_LED1(2,30);
		INITIALIZATION--;			//завершение инициализации по обнулении параметра INITIALIZATION
		TIMER1CNT=0; TCNT1=0;		//сброс счетчика Т1
		TADR = 100 + Ntab;
		display_dnum(TADR);
	}
}
//при завершении инициализации запись в ЕЕ номера табло
if (WRITEEENTAB) {
	TADR = 100 + Ntab;
	WRITEEENTAB=0;
	cli();
	eeprom_write_byte(EETab,Ntab);
	eeprom_write_byte(EETab+1,TADR);
	sei();
}

_UPDATEDATA();
_delay_ms(500);		//отображение адреса табло 500мс
}