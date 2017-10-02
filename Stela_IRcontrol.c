/*
* Stela_IRcontrol.c
*
* Created: 14.09.2017 22:33:24
*  Author: bya

2017		: добавлена пауза 200мс между полученными пакетами данных по RX
: (проблема: после длительной работы при получении нескольких байт данных теряется последний, возможно из-за задержек записи в буфер)
: (: происходит зависание на табло часов - не изменяется час и температура)
: (: возможно по этой же причине некоторые табло-ценники не меняют яркость)

20170914	:	добавлены в процедуру COMMANDS команды кнопочного управления - заготовка для IR-управления

20170916    :   надо проверить рефактор по этой функции  -  uint8_t USART_handle(void)  -  поубирал много лишнего, отрефакторил

20170919	:	изменил в Ind_lib_v2.1.h значение параметра LED_TYPE 5
значение DISPLAY = TabloUpdatePeriod
переименование DISPLAY в TabloUpdateTime
для тестирования Digit[0]++; в процедуре обновления табло
добавление #define TabloUpdatePeriod
CountDigitButtonClick - счетчик нажатия цифровых кнопок ИК-пульта от 1 до 4; при первом нажатии CountDigitButtonClick==0;
смещение курсора при вводе цены при нажатии цифровых кнопок: CountDigitButtonClick++;
ввод значений в массив DigTmp[CountDigitButtonClick]=RC5DIG_num
код в процедуре case RC5DIG0: {...} надо вынести в функцию


*/


#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "rc5_german.h"
#include "ADC_lib.h"
#include "usart.h"
#include "Ind_lib_v2.1.h"

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
#define RXPRICE		0xAC	//без сохранения в ЕЕ для прямого редактирования цены (кнопочный пульт, ИК-пульт)

//адреса устройств сети
#define BROADCAST	0xFF	//адрес широковещательной передачи = 0
#define TADR0		0x64	//адрес нулевого табло (начало отсчета)

//параметры
#define PORTLED		PORTD	//порт индикатора
#define DDRLED		DDRD	//порт индикатора
#define LED1		PD4		//порт индикатора
#define DDR_RS485	DDRD	//порт интерфейса
#define PORT_RS485	PORTD	//порт интерфейса
#define PTXEN		PD2		//порт интерфейса
#define RXD			PD0		//вход UART - приемник
#define TXD			PD1		//выход UART - передатчик
#define MAXINDMODE	1		//количество режимов индикации
#define MAXNTAB		12		//максимальный номер табло, устанавливаемый кнопкой

// пороги уровней освещенности
uint8_t const ADCLUXCH = 7; //сидит на 7 порту
uint8_t const LUM1 = 100; //уровни освещенности
uint8_t const LUM2 = 120;
uint8_t const LUM3 = 580;
uint8_t const LUM4 = 620;
#define MIDDLE_BRIGHT 3 //индекс для массива значений яркости табло (среднее)

//общие константы
#define TabloUpdatePeriod		1000 //1 ед. = 4,4мс при TCCRB = 0x01 в Initialize : initT1(0x01)
//1 ед. = 35мс при TCCRB = 0x02 в Initialize : initT1(0x02)
//1 ед. = 1мс при TCCRB = 0x0A; OCR1A = 1843; в Initialize : initT1(0x0A)

// пороги перехода времени суток
#define MORNING_HOUR	7
#define EVENING_HOUR	20
#define NIGHT_HOUR		23

// команды прямого редактирования
#define RC5POWER		12
#define RC5MUTE			13
#define RC5OK			59
#define RC5CHUP			32
#define RC5CHDWN		33
#define RC5VOLUP		16
#define RC5VOLDWN		17
#define RC5TEXT			42
#define RC5DIG0			0
#define RC5DIG1			1
#define RC5DIG2			2
#define RC5DIG3			3
#define RC5DIG4			4
#define RC5DIG5			5
#define RC5DIG6			6
#define RC5DIG7			7
#define RC5DIG8			8
#define RC5DIG9			9
#define RC5RED			55
#define RC5GREEN		54
#define RC5YELLOW		50
#define RC5BLUE			52
#define RC5SUBTITLE		44
#define RC5TVR			30
#define RC5INFO			10
#define RC5EXIT			56
#define RC5MENU			63


uint8_t const DASHCODE = 10;
uint8_t const ADCLUXCOUNTER = 100; //количество измерений
uint16_t j = 0, it1 = 0;
unsigned long cntT1 = 0, TabloUpdateTime, ADCSTART, TxRxBufCleanPeriod = 3600000;
unsigned long doTimer = 0;
uint32_t const ONEMIN = 60000;
unsigned long const ONEDAY = 86400000;

uint8_t dispcounter, OCRtest, cntT0;
uint8_t cnt_btn0, cnt_btn1;
int8_t TestCNT, INITtab = 0;

uint8_t Ntab,TADR, qtTab, tabUP = 1;		//tabUP - изменяемый флаг направления перехода между табло при редактировании

uint8_t EEMEM EETab[3] = {1, 101, 5};		//Ntab - номер табло, TADR - адрес табло, qtTab - количество табло

uint8_t isSettingsMode = 0; //флаг что мы находимся в режиме настроек
uint8_t isSettingsModeOver = 0; //флаг выхода из режима настроек
uint8_t CountDigitButtonClick = 0; //флаг первичного нажатия на клавишу цифры на ИК пульте (для обнуления текущего выбранного табло)

uint8_t Digit[4]={5, 6, 7, 8}, PointMask=0x0F, DigTmp[4], nDig;
uint8_t EEMEM EEDigit[4] = {0, 0, 0, 0},
EEDsort[5] = {0, 1, 2, 3, 0x0F},
EEdata[5] = {1, 255, 0, 0, 101};

uint8_t EEMEM EEBriData[3] = {5, 95, 250}; //уровень яркости {ночной,средний, дневной}

uint8_t BriMode,
preBriMode,
BriLevels[3] = {20, 95, 214},
BriValues[12] = {5, 30, 40, 50, 62, 77, 95, 118, 146, 181, 214, 250},
BriStep;

//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
uint8_t ABCD_T [MAXDIGNUMBER]= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};


//EEdata - 4Б: данные для настройки табло
//{номер табло, яркость, режим индикации, режим работы,адрес табло} (номер и адрес табло - константы для надежности)

uint8_t getADR, UARTcommand, UARTdata;
//флаги работы программы
int8_t TXBRIDATA, TXTAB, TXDATAEN, SOFTRESET, CHBRI, SWMODE;
uint8_t ADCENABLE, EDITBRI, EDITDIG;
int8_t WREEN, WRITEEEDIG, WRITEEEBRI, WRITEEENTAB, READEEBRI, READEEDIG, READEEDSORT;
int8_t PRESSBTN0, PRESSBTN1, REPRESSBTN0, REPRESSBTN1;
int8_t RCONTROL, Rfunc, mode=0, premode;
//Rfunc-номер команды (кнопки) поступившей с ИК-пульта
//mode-режим работы- 0:стандарт; 1:временное отображение данных; 2:режим редактирования

//static volatile uint8_t rxCount;
extern uint16_t v_ADC, adc_counter;

uint8_t hour=12, min=0, sec=0;

uint8_t _CONVERT_pult_code(uint8_t code)
{
	switch (code)
	{
		case 222:	return 0; break;
		case 24:	return 1; break;
		case 205:	return 2; break;
		case 93:	return 3; break;
		case 27:	return 4; break;
		case 87:	return 5; break;
		case 215:	return 6; break;
		case 28:	return 7; break;
		case 223:	return 8; break;
		case 95:	return 9; break;
		case 1:		return 10; break;
		default:	return 11; break;
	}
}

uint8_t _ascii2dec(uint8_t code)
{
	switch (code)
	{
		case 0x2D:		return 10; break;
		case 0x30:		return 0; break;
		case 0x31:		return 1; break;
		case 0x32:		return 2; break;
		case 0x33:		return 3; break;
		case 0x34:		return 4; break;
		case 0x35:		return 5; break;
		case 0x36:		return 6; break;
		case 0x37:		return 7; break;
		case 0x38:		return 8; break;
		case 0x39:		return 9; break;
		default:		return 11; break;
	}
}

uint8_t _ABCD2dec(uint8_t code)
{
	switch (code)
	{
		case 0x3F:		return 0; break;
		case 0x06:		return 1; break;
		case 0x5B:		return 2; break;
		case 0x4F:		return 3; break;
		case 0x66:		return 4; break;
		case 0x6D:		return 5; break;
		case 0x7D:		return 6; break;
		case 0x07:		return 7; break;
		case 0x7F:		return 8; break;
		case 0x6F:		return 9; break;
		case 0x40:		return 10; break;
		case 0x00:		return 11; break;
		case 0x63:		return 12; break;
		default:		return 0x11; break;
	}
}

//конвертер DEC  -> BCD
char _dec2bcd(char num)
{
	return ((num / 10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
int16_t _bcd2dec(int16_t num)
{
	return ((num / 16 * 10) + (num % 16));
}

//проверка и корректировка данных для табло
uint8_t _CORRECT(uint8_t code, uint8_t digit, int8_t codetype)
{
	uint8_t tdata = code;
	code = digit;
	//codetype: тип кодировки
	//0: неопределен
	//1: десятичный (свой шрифт)
	//2: ASCII
	//3: ABCD - код 7-сегментного индикатора
	//4: Pult_code - кодировка пульта (Кузьмин П.В.)
	switch (codetype) {
		case 1: {
			if ((tdata >= 0) && (tdata <= MAXDIGNUMBER)) code=tdata;
			break;	//корректное значение цифры
		}
		case 2:	{
			code = _ascii2dec(tdata);
			break;
		}								//преобразование из формата ASCII
		case 3: {
			code = _ABCD2dec(tdata);
			break;
		}								//преобразование из кода 7-сегм.
		case 4: {
			code = _CONVERT_pult_code(tdata);
			break;
		}						//преобразование из кода Pult_code
		default: {
			if ((tdata >= 0) && (tdata <= MAXDIGNUMBER)) code=tdata;				//корректное значение цифры
			else if ((tdata > 0x29) && (tdata < 0x40)) code = _ascii2dec(tdata);			//преобразование из формата ASCII
			break;
		}
	}
	return code;										//некорректное значение заменяем на digit
}

//переключение индикатора LED1 на порту PORTLED
void _LED1(int8_t mLED1)
{
	switch (mLED1) {
		case 0: {
			PORTLED |= _BV(LED1);
			break;
		}
		case 1: {
			PORTLED &= ~_BV(LED1);
			break;
		}
		default: break;
	}
}

//мигание индикатора LED1: num раз с задержкой delays5ms, которая указывается в мс
void _flash_LED1(uint8_t num, uint16_t delays5ms)
{
	delays5ms = delays5ms / 5;	//задержка кратна 5мс
	_LED1(0);
	//в функции _delay_ms() из библиотеки <util/delay.h> в качестве аргумента не может быть указана переменная,
	//		указывается только константа: поэтому задержка по циклу
	for (int n = 0; n < num; n++) {
		for (int dl = 0; dl < delays5ms; dl++) _delay_ms(5);
		_LED1(1);
		for (int dl = 0; dl < delays5ms; dl++) _delay_ms(5);
		_LED1(0);
	}
}

//определение номера шага по яркости
uint8_t _getBriStep(uint8_t level)
{
	uint8_t bs = 0;
	while (bs < (sizeof(BriValues) - 1) && BriValues[bs] < level) {
		bs++;
	}
	return bs;
}

//управление интерфейсом UART-RS485
void _RS485(int8_t type)
{
	switch(type) {
		case 0: {
			// type=="INIT"	//инициализация UART-RS485
			DDR_RS485 |= (1 << PTXEN);  //Конфигурация управления интерфейсом UART & RS485
			UCSRB |= (1 << TXCIE);
			sei();

			//дополнительно
			MCUCR &= ~(1 << PUD);	//включение внутренних подтягивающих резисторов на портах
			PORT_RS485 |= (1 << RXD);	//подтягивающий резистор на линии RXD для снижения помех
			break;
		}
		case 1: {
			//type=="RXEN-TXDIS"	//разрешение приема и запрет передачи
			PORT_RS485 &=~ (1<<PTXEN);
			break;
		}
		case 2:{
			//type=="TXEN-RXDIS"	//разрешение передачи и запрет приема
			PORT_RS485 |= (1 << PTXEN);
			break;
		}
	}
}

//отправка 4 байта на табло по адресу TXADR
void TxDATA(uint8_t TXADR, uint8_t Command, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3)
{
	WREEN &= ~0x01; //запретить запись в ЕЕ на время передачи данных
	_RS485(2);
	USART_PutChar(TXADR);
	USART_PutChar(Command);
	USART_PutChar(Data0);
	USART_PutChar(Data1);
	USART_PutChar(Data2);
	USART_PutChar(Data3);
}


//по окончанию передачи отключение передатчика
ISR(USART_TXC_vect)
{
	_RS485(1);		//отключить передачу, включить прием
	WREEN |= 0x01;			//разрешить запись в ЕЕ
}

//настройка параметров обработки нажатия кнопок на портах PC0 и PC1
void initBTN()
{
	// 	//таймер Т0 для считывания нажатия кнопки
	DDRC &= ~((1 << PC0) | (1 << PC1)); //порты в режим входа
	PORTC &= ~((1 << PC0) | (1 << PC1));
	PRESSBTN0 = 0;
	PRESSBTN1 = 0;
	REPRESSBTN0 = 1;
	REPRESSBTN1 = 1;
}

//настройка таймера Т1
//		Т1 используется для периодического обновления данных на табло
void initT1(uint8_t TCRB, uint16_t _ocr1A)
{
	TCCR1A = 0;
	TCCR1B = TCRB;
	TIMSK |= (1 << OCIE1A);
	OCR1A = _ocr1A;
	cntT1 = 0;
}

//прерывание по сравнению Т1 (регистра OCR1A)
//	:опрос нажатия кнопок
//	:инкремент переменной long cntT1 - счетчик периодов таймера Т1 (настраивается на 1 мс)
//		cntT1 используется при:
//			:обновляении данных на табло
//			:обновлении данных в режиме тестирования табло
//			:запуске измерений АЦП
ISR (TIMER1_COMPA_vect)
{
	cntT1++;
	//проверка нажатия кнопки PC0
	if (PINC & 0x01) { //если PINC0=1, то нет нажатия
		cnt_btn0 = 0;
		PRESSBTN0 = 0; //кнопка отпущена
	}
	else {			//иначе - кнопка нажата
		cnt_btn0++;  //счетчик задержки нажатия для исключения дребезга
		if (cnt_btn0 > 20) PRESSBTN0 = 1;	//кнопка нажата
	}

	//проверка нажатия кнопки PC1
	if (PINC & 0x02) { //если PINC1=1, то нет нажатия
		PRESSBTN1 = 0;	//кнопка отпущена
		cnt_btn1=0;
	}
	else {			//иначе - кнопка нажата
		cnt_btn1++;  //счетчик задержки нажатия для исключения дребезга
		if (cnt_btn1 > 20) PRESSBTN1 = 1;
	}
}

//установка режима и яркости по времени или освещенности
void set_Bright(uint16_t val, uint8_t param)
{
	switch (param) {
		//установка режима яркости по номеру
		case 0: {
			BriMode = val;
			break;
		}
		//установка режима яркости по времени
		case 1: {
			if (CHBRI) {
				WRITEEEBRI = 1;
			}
			else {
				if (val >= MORNING_HOUR && val < EVENING_HOUR) {
					BriMode = 0;
				}
				else if (val >= EVENING_HOUR && val < NIGHT_HOUR) {
					BriMode = 1;
				}
				else if ((val >= NIGHT_HOUR && val <= 0) || (val > 0 && val < MORNING_HOUR)) {
					BriMode = 2;
				}
				TXBRIDATA = 1;
			}
			break;
		}
		//установка режима яркости по значению освещенности
		case 2: {
			if (CHBRI) {
				WRITEEEBRI = 1;
			}
			else {
				if (val < (LUM1)) {
					BriMode = 0;
				}
				else if ((val > LUM2) && (val < (LUM3))) {
					BriMode = 1;
				}
				else if (val > (LUM4)) {
					BriMode = 2;
				}
				TXBRIDATA = 1;
			}
			break;
		}
		//действия при изменении яркости по команде:
		//сброс измерения освещенности и настройка записи режимов яркости в ЕЕ
		case 3: {
			ADCSTART=cntT1 + 30000;	//остановка автоматического изменения яркости, запуск через неск.секунд в модуле if(DISPLAY)
			WRITEEEBRI = 0;	//исключить лишнюю запись в ЕЕ -
			CHBRI = 1;		//разрешение WRITEEEBRI через неск.секунд в case 1 и case 2
			TXBRIDATA = 1;	//отправка значения яркости на ведомые табло
			break;
		}
		//установить яркость = val на всех табло
		case 4: {
			ADCSTART=cntT1 + ONEMIN;	//остановка автоматического изменения яркости, запуск через минуту
			BriLevels[BriMode] = val;				//
			TxDATA(BROADCAST, BRIGHT, BriLevels[BriMode], BROADCAST, BRIGHT, BriLevels[BriMode]);	//установить яркость = val на всех табло
			break;
		}
		case 5: {
			ADCSTART=cntT1 + ONEMIN;	//остановка автоматического изменения яркости, запуск через минуту
			BriLevels[BriMode] = val;				//
			break;
		}
	}
	
	OCR2 = BriLevels[BriMode];		//установка яркости по выбранному режиму
	BriStep = _getBriStep(BriLevels[BriMode]);
}

//Передача данных по проводному пульту (интерфейс 485)
void COMMANDS(uint8_t func) {
	switch (func) {
		//установка яркости табло
		case BRIGHT: {
			_LED1(1);
			if (USART_GetRxCount() == 0) _delay_ms(200);
			if (USART_GetRxCount())	{
				UARTdata = USART_GetChar();		//чтение значения яркости
				BriLevels[BriMode] = UARTdata;
				set_Bright(BriLevels[BriMode], 3);
			}
			_LED1(0);
			break;
		}
		//Увеличить яркость на ед.градации
		case BRIGHTUP: {
			_LED1(1);
			if (BriStep < (sizeof(BriValues) - 1)) {
				BriStep++;
				BriLevels[BriMode] = BriValues[BriStep];
				set_Bright(BriLevels[BriMode], 3);
			}
			_LED1(0);
			break;
		}
		case BRIGHTDWN: {
			_LED1(1);
			if (BriStep > 0) {
				BriStep--;
				BriLevels[BriMode] = BriValues[BriStep];
				set_Bright(BriLevels[BriMode], 3);
			}
			_LED1(0);
			break;
		}
		//прием данных для установки значений на табло
		case RXTDATA: {
			_LED1(1);
			j = 0;
			
			if (USART_GetRxCount() == 0) _delay_ms(200);
			
			while ((j < 4) && (USART_GetRxCount()))
			{
				UARTdata = USART_GetChar();	//чтение цифры из буфера
				Digit[j] = _CORRECT(UARTdata, Digit[j], 0); //запись в ОЗУ скорректированного значения
				j++;
				
				if (USART_GetRxCount() == 0) _delay_ms(200);
			}

			WRITEEEDIG = 1;	//записать значение цены в ЕЕ
			TabloUpdateTime = 1;	 	//обновить инф. на табло
			_LED1(0);
			break;
		}
		//установка режима работы табло - заблокировано
		//показать номер режима работы
		case SETMODE: {
			_LED1(1);
			if (USART_GetRxCount() == 0) _delay_ms(200);
			
			if (USART_GetRxCount())	{
				UARTdata = USART_GetChar();		//чтение значения режима работы
			}
			display_dnum(mode);					//показать номер режима
			TCNT1 = 0;
			TabloUpdateTime = 0;					//задержать обновление табло
			_LED1(0);
			break;
		}
		//отобразить номер табло (установка заблокирована)
		case SETTADR: {
			if (USART_GetRxCount() == 0) _delay_ms(200);
			
			if (USART_GetRxCount())	{
				UARTdata = USART_GetChar();		//чтение адреса
			}
			display_dnum(TADR);					//показать адрес табло
			TCNT1 = 0;
			TabloUpdateTime = 0;					//задержать обновление табло
			break;
		}
		//программный сброс
		case RESET: {
			_LED1(1);
			
			if (USART_GetRxCount() == 0) _delay_ms(200);
			
			if (USART_GetRxCount())	{
				UARTdata = USART_GetChar();		//чтение оператора команды
				
				if (UARTdata == 1) SOFTRESET = 1;
			}
			_LED1(0);
			break;
		}
	}
}

uint8_t USART_handle(void)
{
	uint8_t CORRECT=1;
	getADR = USART_GetChar();	//чтение байта 1: адрес
	if (getADR!=BROADCAST) {	//если не широковещательный
		if (getADR==TADR) {		//сравнение с адресом контроллера
			//выполнение при совпадении адреса
			if (USART_GetRxCount()) UARTcommand = USART_GetChar();	//чтение адресной команды
			else {
				_delay_ms(200);		//дополнительная задержка для загрузки байта
				if (USART_GetRxCount()) UARTcommand = USART_GetChar();
				else CORRECT=0;
			}
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
	return CORRECT;
}

void ADCluxmeter(uint8_t luxchannel)
{
	//суммирование результата измерения освещенности по номеру канала с датчиком
	v_ADC += ADC_result(luxchannel) / ADCLUXCOUNTER;	//10-битный АЦП, вычисляем среднее значение
	adc_counter++; //набираем количество измерений для определения среднего значения
	
	//изменение яркости в соответствии с измеренной освещенностью
	if (adc_counter > ADCLUXCOUNTER)		//с периодичностью 10 сек
	{	
		#ifdef DEBUG
		{
			PrintStringWithValToSerial("LOW v_ADC = ", (uint8_t) v_ADC);
			PrintStringWithValToSerial("HIGH v_ADC = ", (uint8_t) (v_ADC>>8));
			display_dnum(v_ADC);
			TabloUpdateTime = cntT1 + 5000;
		}
		#endif
		
		set_Bright(v_ADC, 2);		//установить режим яркости по освещенности
		if (BriMode != preBriMode)	//если режим яркости изменился
		{
			preBriMode = BriMode;
			TXDATAEN |= 0x01;
		}
		//				else TXDATAEN &=~(0x01);	//запрещение TX при сохранении режима яркости
		adc_counter = 0;
		v_ADC = 0;
		_flash_LED1(3, 20);
	}
}

void _SOFTRESET(void)
{
	//	WDTCSR=0x00;
	
	DDRLED |= _BV(LED1);//|_BV(LED2);
	_flash_LED1(3,100);

	mode=0;

	cli();
	
	for (j = 0; j < 4; j++)
	{
		eeprom_write_byte(EEDigit + j, j + 1);
	}
	for (j = 0; j < 4; j++)
	{
		Digit[j] = eeprom_read_byte(EEDigit + j);
	}
	
	sei();
	display_10code(Digit[0], Digit[1], Digit[2], Digit[3]);
}


//установка начальных значений переменных
void Initialize(void)
{
	cli();
	Ntab = eeprom_read_byte(EETab);		//значение номера табло
	TADR = eeprom_read_byte(EETab + 1);		//значение адреса табло
	qtTab = eeprom_read_byte(EETab + 2);		//значение количества табло

	DDRLED |= (1 << LED1);//|_BV(LED2);
	
	digit_sort(0, 1, 2, 3); //сортировка разрядов табло
	set_PWC(2); //установка режима поразрядной индикации
	
	j = 0;
	it1 = 0;
	
	TabloUpdateTime = TabloUpdatePeriod;
	cntT1 = 0;
	CHBRI = 0;
	ADCENABLE = 1;			//разрешить преобразования АЦП
	TXDATAEN = 3;
	WREEN = 1;					//разрешить запись в ЕЕ
	WRITEEENTAB = 0;
	WRITEEEDIG = 0;
	WRITEEEBRI = 0;
	READEEDIG = 1;
	READEEBRI = 1;
	SOFTRESET = 0;
	ADCSTART = 0;
	adc_counter = 0;
	
	BriMode = 2;
	BriStep = _getBriStep(BriLevels[BriMode]);
	set_Bright(BriLevels[BriMode], 3);
	
	initRC5();
	initBTN();			//обработка в T1_OVF
	init595();
	initPWM();
	initT1(0x0A, 1843);		//16-битный, clk/2, режим СТС, OCR1A = 1843, период  1 мс
	adc_init();
	USART_Init(USART_DOUBLED, 19200);
	USART_FlushTxBuf();

	display_dnum(TADR);
	
	TXDATAEN |= 0x02;	//разрешение TX глобально (2й бит)
	//	TXDATAEN &=~ 0x02;	//запрещение TX глобально (2й бит)
	WREEN |= 0x02;		//разрешить запись яркости в ЕЕ

	_RS485(0); //инициализация интерфейса RS485
	_RS485(1);	 //RX_EN + TX_DIS

	_flash_LED1(3, 50);
}



/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
int main(void)
{
	Initialize();

	// 	initT1(0x00);	//остановка таймера для исключения ложного срабатывания
	// 	cntT1=0;
	// 	REPRESSBTN1=1;
	// 	while (PRESSBTN1) //пока кнопка BTN1 не будет отпущена
	// 	{
	// 		//вход в режим инициализации - нажатие кнопки в течение 3 сек
	// 		//исключаем ошибку случайного замыкания кнопки при сбросе питания (например, из-за снега или дождя)
	// 		//при нажатии кнопки более 15ти сек - выход из режима инициализации
	// 		if (REPRESSBTN1)		//однократное выполнение при длительном нажатии
	// 		{
	// 			_LED1(1);
	// 			initT1(0x05);		//запускаем таймер длительности нажатия
	// 			REPRESSBTN1=0;
	// 		}
	// 		if (cntT1==1)		//подтверждение запуска инициализации при удержании кнопки примерно 1 сек
	// 		{
	// 			cntT1=2;
	// 			INITtab=3;	//установка параметра для инициализации табло
	// 			display_7code(0x08,0x08,0x08,0x08);
	// 			_flash_LED1(1,200);
	// 		}
	// 		if (cntT1>4)		//слишком длительное нажатие (например при замыкании из-за влаги)
	// 		{
	// 			INITtab=0;	//пропуcтить инициализацию при превышении времени удержания кнопки
	// 			cntT1=0;
	// 			_flash_LED1(5,50);
	// 			break;				//выход из цикла и запуск работы табло
	// 		}
	// 		_delay_ms(10); //без этой задержки не выходит из цикла при отпускании кнопки PRESSBTN, возможно из-за оптимизации кода компилятором
	// 	}
	// 	REPRESSBTN1=1;
	// 	_LED1(0);
	//
	// 	//выполнить инициализацию - определение номера табло Ntab
	// 	while(INITtab)
	// 	{
	// 		//при нажатии BTN1 инкремент Ntab
	// 		if (PRESSBTN1)
	// 		{
	// 			if (REPRESSBTN1)
	// 			{
	// 				_flash_LED1(1,20);
	// 				REPRESSBTN1=0;
	// 				WRITEEENTAB=1;
	// 				if (Ntab < MAXNTAB)
	// 				{
	// 					Ntab++;	//максимальный номер табло MAXNTAB (12)
	// 				}
	// 				else
	// 				{
	// 					Ntab=0;
	// 				}
	// 				display_dnum(Ntab);
	// 				INITtab=2;		//сброс выхода из инициализации
	// 				cntT1=0; TCNT1=0;		//сброс счетчика Т1
	// 			}
	// 		}
	// 		else REPRESSBTN1=1;
	//
	// 		//индикация номера табло и отсчет завершения инициализации
	// 		if (cntT1)
	// 		{
	// 			if (Ntab) _flash_LED1(Ntab,200);
	// 			else _flash_LED1(2,30);
	// 			INITtab--;			//завершение инициализации по обнулении параметра INITIALIZATION
	// //			cntT1=0; TCNT1=0;		//сброс счетчика Т1
	// 			TADR = TADR0 + Ntab;
	// 			display_dnum(TADR);
	// 		}
	// 	}
	// 	//при завершении инициализации запись в ЕЕ номера табло
	// 	if (WRITEEENTAB) {
	// 		TADR = TADR0 + Ntab;
	// 		WRITEEENTAB=0;
	// 		cli();
	// 		eeprom_write_byte(EETab,Ntab);
	// 		eeprom_write_byte(EETab+1,TADR);
	// 		sei();
	// 	}
	//
	// 	_UPDATEDATA();
	// 	_delay_ms(500);		//отображение адреса табло 500мс

	while(1)
	{
		//программный сброс
		if (SOFTRESET) {
			if (WDTCR & (1 << WDE)) {
				while(1);
			}
			else {
				Initialize();
				_SOFTRESET();
			}
		}
		
		if (cntT1 > TxRxBufCleanPeriod) {
			USART_FlushTxBuf();
			USART_FlushRxBuf();
			TxRxBufCleanPeriod = cntT1 + ONEDAY;
		}
		
		//обновление данных на табло по достижению счетчика cntT1 + TabloUpdatePeriod
		if (cntT1 > TabloUpdateTime && isSettingsMode == 0) {
			_LED1(1);
			TabloUpdateTime = cntT1 + TabloUpdatePeriod;
			if (Digit[3] == 10) {
				Digit[3] = 0;
				Digit[2]++;
				
			}
			if (Digit[2] == 6) {
				Digit[2] = 0;
				Digit[1]++;
			}
			if (Digit[1] == 10) {
				Digit[1] = 0;
				Digit[0]++;
			}
			if (Digit[0] == 6) {
				Digit[0] = 0;
			}
			
			display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3]++, 0x0F);	//прямое отображение
			_LED1(0);
		}
		
		//запуск измерений АЦП
		if ((cntT1 > ADCSTART) && ADCENABLE)
		{
			//PrintStringToSerial("INTO ADC FINE!");
			ADCSTART=cntT1 + 100;	//примерно с периодом 0.1 сек
			ADCluxmeter(ADCLUXCH);
		}

		//чтение значения яркости из ЕЕ в ОЗУ
		if (READEEBRI) {
			_LED1(1);
			READEEBRI = 0;
			cli();
			//считываем значения трех уровней яркости из ЕЕ
			BriLevels[0] = eeprom_read_byte(EEBriData); //ночная яркость
			BriLevels[1] = eeprom_read_byte(EEBriData + 1); //яркость утром и вечером
			BriLevels[2] = eeprom_read_byte(EEBriData + 2); //дневная яркость
			sei();
			set_Bright(1, 0);
			TXBRIDATA = 1;
			_LED1(0);
		}
		
		//отправка широковещ-й команды установки яркости на табло
		//фильтр с проверкой двух младших битов
		if (TXBRIDATA && (TXDATAEN & 0x01) && (TXDATAEN & 0x02)) {
			//если глобально разрешена передача по RS485
			TXBRIDATA = 0;
			TxDATA(BROADCAST, BRIGHT, BriLevels[BriMode], BROADCAST, BRIGHT, BriLevels[BriMode]);
		}

		//запись данных на всех табло в ЕЕ
		if (WRITEEEDIG && (WREEN & 0x01)) {
			_LED1(1);
			WRITEEEDIG = 0;
			EepromWritePrice(BROADCAST - TADR0); //широковещательная отправка на все табло для сохранения в ее
			_LED1(0);
		}
		
		//чтение данных табло в ОЗУ из ЕЕ
		if (READEEDIG) {
			_LED1(1);
			READEEDIG = 0;
			cli();
			for (j = 0; j < 4; j++)
			Digit[j] = eeprom_read_byte(EEDigit + j);
			sei();
			_LED1(0);
		}
		
		//запись значения яркости в ЕЕ
		if (WRITEEEBRI && (WREEN & 0x03)) {
			_LED1(1);
			WRITEEEBRI = 0;
			CHBRI = 0;
			cli();
			eeprom_write_byte(EEBriData + BriMode, BriLevels[BriMode]); //сохранение яркости только для текущего режима
			sei();
			_LED1(0);
		}

		if (USART_GetRxCount()) {	//проверка наличия данных в буфере приема UART
			_LED1(1);
			if (USART_handle()) {
				COMMANDS(UARTcommand);		//если данные корректны, то выполнить команду
			}
			_LED1(0);
		}//UART

		//обработчик команд, поступающих по ИК-
		if (rc5_data)
		{
			j = rc5_data;
			Rfunc = ((j & 0x3F)|(~j >> 7 & 0x40)); //Выделяем только код команды
			//		USART_SendStr("---rc5_data OK");
			//		USART_SendStr("------ TRY RCommand");
			RCommand(Rfunc, isSettingsMode);
			//если мы верно ввели количество табло в настройках то у нас isSettingsMode сохранится = 1 и мы сохраняемся в еепром + подтверждаем верный ввод еще одним морганием
			if (isSettingsMode == 1 && isSettingsModeOver == 1) {
				PrintStringToSerial(" if isSettingsMode == 1 && isSettingsModeOver == 1 OK ");
				eeprom_write_byte(EETab + 2, qtTab);
				isSettingsMode = 0;
				isSettingsModeOver = 0;
				//PrintStringToSerial(" TRY DoBlinkingAllTabs() ");
				display_7code(0x39, ABCD_T[2], ABCD_T[10], ABCD_T[qtTab]); //вывели на экран новое значение количества табло
				DoBlinking(1); //поморгали ведущим табло в течении 3 секунд
			}
			rc5_data=0;
		}
		
	}//while(1)

}//main()

//Задаем количество табло по нажатию кнопки с ИК пульта
void SetCountTabs(uint8_t func)
{
	switch (func) {
		case RC5DIG1:
		case RC5DIG2:
		case RC5DIG3:
		case RC5DIG4:
		case RC5DIG5:
		case RC5DIG6:
		case RC5DIG7:
		case RC5DIG8:
		case RC5DIG9: {
			qtTab = func;
			PrintStringWithValToSerial(" ---SetCountTabs() OK    qtTab = ", qtTab);
			isSettingsModeOver = 1; //поднимаем флаг что ввод окончен
			break;
		}
		default: {
			isSettingsMode = 0; //если неверно ввели то выходим из режима настроек
			break;
		}
	}
	_delay_ms(300);
}

//Задаем значения на выбранном табло в режиме редактирования или же выбираем следующее табло при повторном нажатии на Power или Ok
void SetSettingsFromIrControl(uint8_t func)
{
	PrintStringWithValToSerial(" ---SetSettingsFromIrControl() OK    KEY CODE = ", func);

	switch (func) {
		//нажатие кнопки Power повторно
		case RC5POWER: {
			PowerButtonClickProgMode();
			break;
		}
		case RC5OK: {
			OkButtonClickProgMode();
			break;
		}
		case RC5EXIT: {
			ExitButtonClickProgMode();
			break;
		}
		case RC5DIG0:
		case RC5DIG1:
		case RC5DIG2:
		case RC5DIG3:
		case RC5DIG4:
		case RC5DIG5:
		case RC5DIG6:
		case RC5DIG7:
		case RC5DIG8:
		case RC5DIG9: {
			DigitButtonClickProgMode(func);
			break;
		}
		default: {
			PrintStringToSerial("PROGRAMMING MODE WRONG CODE");
			break;
		}
	}
}

void PowerButtonClickProgMode()
{
	if (CountDigitButtonClick > 0) {
		//PrintStringToSerial(" ------TRY EepromWritePrice()");
		EepromWritePrice(Ntab); //сохранили цену табло в еепром если редактировали
	}
	
	Ntab++;
	
	//проверка на превышение Ntab > qtTab
	if (Ntab > qtTab) {
		//PrintStringToSerial("Ntab > qtTab OK");
		Ntab = 1; //тогда переключаемся снова на первое табло
	}
	PrintStringToSerial("DO BRIGHT all tabs OK");
	set_Bright(BriValues[MIDDLE_BRIGHT], 4); //яркость всех табло
	//PrintStringToSerial("DO Blinking curr tab OK");
	DoBlinking(Ntab); //начинаем моргать текущим табло
	CountDigitButtonClick = 0;
}

void OkButtonClickProgMode()
{
	_flash_LED1(1, 30);
	if (CountDigitButtonClick > 0) {
		//PrintStringToSerial(" ------TRY EepromWritePrice()");
		EepromWritePrice(Ntab); //сохранили цену табло в еепром  если редактировали
	}
	
	Ntab--;
	
	//проверка на Ntab < 1
	if (Ntab < 1) {
		Ntab = qtTab; //тогда присваиваем индекс последнего табло чтобы переключиться на него
	}
	
	set_Bright(BriValues[MIDDLE_BRIGHT], 4); //яркость всех табло
	DoBlinking(Ntab); //начинаем моргать текущим табло
	CountDigitButtonClick = 0;
}

void ExitButtonClickProgMode()
{
	PrintStringToSerial("EXIT EXIT EXIT");
	Ntab = 1;
	isSettingsMode = 0;
	ADCENABLE = 1;
	READEEBRI = 1; //читаем яркость из ЕЕ
	
	if (CountDigitButtonClick > 0) {
		//PrintStringToSerial(" ------TRY EepromWritePrice()");
		EepromWritePrice(Ntab); //сохранили цену табло в еепром  если редактировали
	}
}

void DigitButtonClickProgMode(uint8_t buttonCode)
{
	//при первом нажатии CountDigitButtonClick == 0
	//выполняется сброс данных на редактируемом табло и установка первой цифры
	//переход к редактированию следующей цифры: CountDigitButtonClick++
	//далее CountDigitButtonClick меняется циклично от 1 до 4
	//ввод значений в массив DigTmp[CountDigitButtonClick-1]=RC5DIG0;
	if (CountDigitButtonClick > 0) {
		CountDigitButtonClick++;
		if (CountDigitButtonClick > 4) CountDigitButtonClick = 1;
		DigTmp[CountDigitButtonClick - 1] = buttonCode;
	}
	else {
		DigTmp[CountDigitButtonClick] = buttonCode;
		DigTmp[CountDigitButtonClick + 1] = DASHCODE;
		DigTmp[CountDigitButtonClick + 2] = DASHCODE;
		DigTmp[CountDigitButtonClick + 3] = DASHCODE;
		CountDigitButtonClick = 1;
	}
	//если редактируемое табло - текущее табло, то изменяем значения
	if (TADR == (TADR0 + Ntab)) {
		Digit[0] = DigTmp[0];
		Digit[1] = DigTmp[1];
		Digit[2] = DigTmp[2];
		Digit[3] = DigTmp[3];
		display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3], 0x0F);	//прямое отображение
	}
	//отправка новой цены на редактируемое табло
	TxDATA(TADR0 + Ntab, RXTDATA, DigTmp[0], DigTmp[1], DigTmp[2], DigTmp[3]);
	_delay_ms(300);
}

//обработчик нажатия кнопки на ИК пульте
void IrControlButtonClick(uint8_t func)
{
	//PrintStringWithValToSerial("---IrControlButtonClick() OK   ButtonCode = ", func);
	
	switch (func) {
		//нажатие кнопки Power
		case RC5POWER:
		case RC5OK: {
			_flash_LED1(1, 30);
			display_7code(0x39, ABCD_T[1], 0, 0);//вывели на экран команду
			PrintStringToSerial("Button POWER OR OK CLICK OK");
			//PrintStringToSerial("TRY ProgrammingModeButtonClick()");
			isSettingsMode = 2; //если нажали Power или OK то взводим флаг что мы в режиме редактирования текущего табло
			ProgrammingModeButtonClick(1); //передали номер нижнего табло по нажатию ок
			break;
		}
		//если нажата кнопка меню то переходим в режим настроек
		case RC5MENU: {
			_flash_LED1(1, 30);
			display_7code(0x39, ABCD_T[2], ABCD_T[10], ABCD_T[qtTab]); //вывели на экран команду и количество табло
			isSettingsMode = 1; // подняли флаг что мы в режиме настроек но ввод еще не закончен, потому как когда нажали меню мы сбрякаемся из этой функи сразу в проверку окончания ввода
			isSettingsModeOver = 0; // флаг что ввод еще не окончен
			DoBlinking(1); //поморгали ведущим табло в течении 3 секунд
			break;
		}
		//нажатие кнопки Vol+
		case RC5VOLUP: {
			COMMANDS(BRIGHTUP);
			break;
		}
		//нажатие кнопки Vol-
		case RC5VOLDWN: {
			COMMANDS(BRIGHTDWN);
			break;
		}
		default: {
			PrintStringWithValToSerial("WRONG BUTTON CLICK!");
			_flash_LED1(4, 30);
			break;
		}
	}
}

//передача данных с ИК пульта
void RCommand (uint8_t func, uint8_t _isSettingsMode) {
	_flash_LED1(1, 30); //Моргнцть один раз что команда принята с пульта
	//если мы в режиме настроек то ждем ввод количества табло и выходим из этого режима
	if (_isSettingsMode == 1) {
		//PrintStringToSerial("if (_isSettingsMode == 1) OK");
		SetCountTabs(func); //Задаем количество табло по нажатию кнопки с ИК пульта
	}
	else if (_isSettingsMode == 0) {
		//PrintStringToSerial(" isSettingsMode = 0 OK");
		//обработчик нажатия кнопки на ИК пульте
		IrControlButtonClick(func);
	}
	else if (_isSettingsMode == 2) {
		//PrintStringToSerial("  isSettingsMode = 2 OK");
		//Задаем значения на выбранном табло в режиме настроек или же выбираем следующее табло при повторном нажатии на Power
		SetSettingsFromIrControl(func);
	}
}

void DoBlinking(uint8_t _nTab) {
	for (uint8_t i = 0; i < 3; i++) {
		if (_nTab == 1) set_Bright(BriValues[0], 5); //яркость текущего табло
		
		TxDATA(TADR0 + _nTab, BRIGHT, BriValues[0], TADR0 + _nTab, BRIGHT, BriValues[0]);
		_delay_ms(200);
		
		if (_nTab == 1) set_Bright(BriValues[11], 5); //яркость текущего табло
		
		TxDATA(TADR0 + _nTab, BRIGHT, BriValues[11], TADR0 + _nTab, BRIGHT, BriValues[11]);
		_delay_ms(200);
	}
}

void DoBlinkingAllTabs() {
	for (uint8_t i = 0; i < 3; i++) {
		set_Bright(BriValues[0], 4); //яркость всех табло
		_delay_ms(200);
		set_Bright(BriValues[11], 4); //яркость всех табло
		_delay_ms(200);
	}
}

//Функция обработки нажатия кнопки режима программирования
void ProgrammingModeButtonClick(uint8_t _nTab) {
	set_Bright(BriValues[MIDDLE_BRIGHT], 4);  //яркость всех табло
	DoBlinking(_nTab); //начинаем моргать текущим табло (самым первым по индексу Ntab, выше = 1)
	PrintStringToSerial("------Set DoBlinking(Ntab) OK   Ntab = ");
	
	// 	doTimer = cntT1 + ONEMIN;		//плюс одна минута на редактирование
	//
	// 	while (cntT1 < doTimer) {
	// 		if (rc5_data)
	// 		{
	// 			doTimer = cntT1 + ONEMIN;		//плюс одна минута на редактирование
	// 			j = rc5_data;
	// 			Rfunc = ((j & 0x3F) | (~j >> 7 & 0x40)); //Выделяем только код команды
	// 			PrintStringToSerial(" INTO WHILE  if (rc5_data) OK OK OK");
	// 			RCommand(Rfunc, isSettingsMode);//парсим команду с ИК
	// 			rc5_data = 0;
	// 		}
	// 		_delay_ms(10);	//иначе while не сработает
	// 	}
	// 	isSettingsMode = 0;
	// 	PrintStringToSerial("................OUT FROM WHILE.................");
}

void PrintStringWithValToSerial(char* string, uint8_t val)
{
	_RS485(2);
	USART_SendStr(string);
	USART_PutChar(val);
	USART_SendStr("\r\n");
}

void PrintStringToSerial(char* string)
{
	_RS485(2);
	USART_SendStr(string);
	USART_SendStr("\r\n");
}

void EepromWritePrice(uint8_t _nTab)
{
	_flash_LED1(1, 30);
	if (_nTab == 1) {
		cli();
		for (j = 0; j < 4; j++)
		eeprom_write_byte(EEDigit + j, Digit[j]);
		sei();
	}
	//команда RX на сохранение в ЕЕ на редактируемом табло
	TxDATA(TADR0 + _nTab, RXWRITEEE, 0x01, TADR0 + _nTab, RXWRITEEE, 0x01);
}