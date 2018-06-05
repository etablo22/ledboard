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
#include <avr/wdt.h>

#include "rc5_german.h"
#include "ADC_lib.h"
#include "usart.h"
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
#define RXPRICE		0xAC	//без сохранения в ЕЕ для прямого редактирования цены (кнопочный пульт, ИК-пульт)

//адреса устройств сети
uint8_t const BROADCAST	=   0xFF;//адрес широковещательной передачи = 0
uint8_t const TADR0 =    	0x64;	//адрес нулевого табло (начало отсчета)

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
uint16_t const LUM1 = 100; //уровни освещенности
uint16_t const LUM2 = 120;
uint16_t const LUM3 = 580;
uint16_t const LUM4 = 620;
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
#define RC5TVFORMAT		41

const uint8_t DEFAULTMODE = 0, //режим работы платы по умолчанию
			  SETTABCOUNTMODE = 1, //режим установки количества табло на стеле
			  SETPRISEMODE = 2, //режим установки цены на табло
			  INFOMODEADDR = 3, //режим установки работы платы слэйв-мастер
			  INFOMODEDEVSTATE = 4, //режим установки работы платы слэйв-мастер
			  SETPINCODEMODE = 5, //режим проверки ввода пин кода для установки слэйв-мастер
			  SETMASTERSLAVEMODE = 6, //режим установки работы платы в состояние слэйв или масте
			  INFOMODESETADDR = 7; //режим установки работы платы в состояние слэйв или масте
uint8_t subMenuItem = 0; //временная переменная для хранения пункта меню сервисной настройки
uint8_t isMasterDevice = 0; //флаг для платы - мастер или слэйв
uint8_t maskDegVal = 0x0F; //значение маски для включения разрядов
const uint8_t SHIFTYELLOBUTTON = 1; //значение на которое сдвигаем разряд маски для включения точки
const uint8_t SHIFTBLUEBUTTON = 0;//значение на которое сдвигаем разряд маски для включения ЭКТО
const uint8_t SHIFTREDBUTTON = 3;//значение на которое сдвигаем разряд маски для включения СНЕЖИНКА
const uint8_t SHIFTGREENBUTTON = 2;//значение на которое сдвигаем разряд маски для включения ЕВРО

uint8_t const DASHCODE = 10; //индекс символа минус из массива ABCD_T для функции display_10code_point
uint8_t const SYMB_SPACEINDX = 11; //индекс символа пробел из массива ABCD_T для функции display_10code_point
uint8_t const ADCLUXCOUNTER = 100; //количество измерений для датчика освещенности
uint16_t irControlData = 0; //2 байта команда с пульта
const uint8_t WAITTIME = 2; //граница длительности нажатой кнопки кратный 1 секунде
uint32_t irLongPressTimer = 0; //таймер длительного нажатия
uint8_t irLongPressCounter = 0; //счетчик длительности нажатия кнопки
uint8_t irRepeatPressCounter = 0; //счетчик повторов пакетов с ИК пульта
uint32_t whileLoopCounter = 0; //счетчик глобального цикла

uint32_t cntBlinkTimer = 0; //таймер цикла для бесконечной моргалки
uint8_t isBlinked = 0; //флаг что табло уже моргнуло (для бесконечной моргалки)
uint32_t cntT1 = 0, cntTabloUpdate, cntADCSTART, cntExitProgMode, TxRxBufCleanPeriod = 3600000;		//cntT1 - глобальный таймер-счетчик с периодом 1 мс
uint32_t const MILLIS_500 = 500, ONESEC = 1000, ONEMIN = 60000, ONEDAY = 86400000;
uint32_t const cntMaxPeriod = 3800000000;		//максимальное значение таймер-счетчика с запасом примерно 10 суток
uint8_t cnt_btn0, cnt_btn1;
int8_t INITtab = 0;

uint8_t editNtab, TADR, qtTab, tabUP = 1;		//tabUP - изменяемый флаг направления перехода между табло при редактировании
												//editNtab - номер редактируемого табло,

uint8_t stelaTabPosition = 0; //позиция текущего табло на стеле
uint8_t EEMEM EETab[3] = {100, 5, 0};		
//TADR - адрес табло,
//qtTab - количество табло,
//isMasterDevice - плата управления мастер или слэйв

uint8_t isSettingsMode = 0; //флаг что мы находимся в режиме настроек
uint8_t countDigitButtonClick = 0; //флаг первичного нажатия на клавишу цифры на ИК пульте (для обнуления текущего выбранного табло)

uint8_t Digit[4]={5, 6, 7, 8}, PointMask=0x0F, DigTmp[4];
uint8_t EEMEM EEDigit[4] = {0, 0, 0, 0},
EEDevPinCode[4] = {0, 0, 0, 0}; //пин код для платы (мак адрес)

uint8_t EEMEM EEBriData[3] = {5, 95, 250}; //уровень яркости {ночной,средний, дневной}

uint8_t BriMode,
preBriMode,
BriLevels[3] = {5, 95, 214},
BriValues[12] = {5, 30, 40, 50, 62, 77, 95, 118, 146, 181, 214, 250},
BriStep = 0;

//-------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
uint8_t ABCD_T[MAXDIGNUMBER]= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};
#define SYMB_C	0x39 //символ С (аббривиатура команда)
#define SYMB_I	0x30 //символ С (аббривиатура команда)
#define SYMB_n	0x54 //символ С (аббривиатура команда)
#define SYMB_F	0x71 //символ С (аббривиатура команда)
#define SYMB_o	0x5c //символ С (аббривиатура команда)
#define SYMB_P	0x73 //символ С (аббривиатура команда)
#define SYMB_i	0x10 //символ С (аббривиатура команда)
#define SYMB_E	0x79 //символ С (аббривиатура команда)
#define SYMB_L	0x38 //символ С (аббривиатура команда)

uint8_t getADR, UARTcommand, UARTdata;
//флаги работы программы
int8_t TXBRIDATA, TXDATAEN, SOFTRESET, CHBRI;
uint8_t ADCENABLE;
int8_t WREEN, WRITEEEDIG, WRITEEEBRI, WRITEEENTAB, READEEBRI, READEEDIG;
int8_t PRESSBTN0, PRESSBTN1, REPRESSBTN0, REPRESSBTN1;
int8_t Rfunc; //Rfunc-номер команды (кнопки) поступившей с ИК-пульта

//static volatile uint8_t rxCount;
extern uint16_t v_ADC, adc_counter;

// uint8_t _CONVERT_pult_code(uint8_t code)
// {
// 	switch (code)
// 	{
// 		case 222:	return 0; break;
// 		case 24:	return 1; break;
// 		case 205:	return 2; break;
// 		case 93:	return 3; break;
// 		case 27:	return 4; break;
// 		case 87:	return 5; break;
// 		case 215:	return 6; break;
// 		case 28:	return 7; break;
// 		case 223:	return 8; break;
// 		case 95:	return 9; break;
// 		case 1:		return 10; break;
// 		default:	return 11; break;
// 	}
// }

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
//
// uint8_t _ABCD2dec(uint8_t code)
// {
// 	switch (code)
// 	{
// 		case 0x3F:		return 0; break;
// 		case 0x06:		return 1; break;
// 		case 0x5B:		return 2; break;
// 		case 0x4F:		return 3; break;
// 		case 0x66:		return 4; break;
// 		case 0x6D:		return 5; break;
// 		case 0x7D:		return 6; break;
// 		case 0x07:		return 7; break;
// 		case 0x7F:		return 8; break;
// 		case 0x6F:		return 9; break;
// 		case 0x40:		return 10; break;
// 		case 0x00:		return 11; break;
// 		case 0x63:		return 12; break;
// 		default:		return 0x11; break;
// 	}
// }

//конвертер DEC  -> BCD
// char _dec2bcd(char num)
// {
// 	return ((num / 10 * 16) + (num % 10));
// }
//
// // Convert Binary Coded Decimal (BCD) to Decimal
// int16_t _bcd2dec(int16_t num)
// {
// 	return ((num / 16 * 10) + (num % 16));
// }

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
		// 		case 1: {
		// 			if ((tdata >= 0) && (tdata <= MAXDIGNUMBER)) code=tdata;
		// 			break;	//корректное значение цифры
		// 		}
		// 		case 2:	{
		// 			code = _ascii2dec(tdata);
		// 			break;
		// 		}								//преобразование из формата ASCII
		// 		case 3: {
		// 			code = _ABCD2dec(tdata);
		// 			break;
		// 		}								//преобразование из кода 7-сегм.
		// 		case 4: {
		// 			code = _CONVERT_pult_code(tdata);
		// 			break;
		// 		}						//преобразование из кода Pult_code
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
			cntADCSTART=cntT1 + ONEMIN;	//остановка автоматического изменения яркости, запуск через одну минуту
			WRITEEEBRI = 0;	//исключить лишнюю запись в ЕЕ -
			CHBRI = 1;		//разрешение WRITEEEBRI через неск.секунд в case 1 и case 2
			TXBRIDATA = 1;	//отправка значения яркости на ведомые табло
			break;
		}
		//установить яркость = val на всех табло
		case 4: {
			cntADCSTART = cntT1 + ONEMIN;	//остановка автоматического изменения яркости, запуск через минуту
			BriLevels[BriMode] = val;				//
			TxDATA(BROADCAST, BRIGHT, BriLevels[BriMode], BROADCAST, BRIGHT, BriLevels[BriMode]);	//установить яркость = val на всех табло
			break;
		}
		case 5: {
			cntADCSTART=cntT1 + ONEMIN;	//остановка автоматического изменения яркости, запуск через минуту
			BriLevels[BriMode] = val;				//
			break;
		}
	}
	
	OCR2 = BriLevels[BriMode];		//установка яркости по выбранному режиму
	BriStep = _getBriStep(BriLevels[BriMode]);
}

//Передача данных по интерфейсу 485
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
			uint8_t j = 0;
			
			if (USART_GetRxCount() == 0) _delay_ms(200);
			
			while ((j < 4) && (USART_GetRxCount()))
			{
				UARTdata = USART_GetChar();	//чтение цифры из буфера
				Digit[j] = _CORRECT(UARTdata, Digit[j], 0); //запись в ОЗУ скорректированного значения
				j++;
				
				if (USART_GetRxCount() == 0) _delay_ms(200);
			}

			//WRITEEEDIG = 1;	//записать значение цены в ЕЕ
			cntTabloUpdate = 1;	 	//обновить инф. на табло
			_LED1(0);
			break;
		}
		case RXWRITEEE: {
			WRITEEEDIG = 1;
			break;
		}
		//установка режима работы табло - заблокировано
		//показать номер режима работы
		// 		case SETMODE: {
		// 			_LED1(1);
		// 			if (USART_GetRxCount() == 0) _delay_ms(200);
		//
		// 			if (USART_GetRxCount())	{
		// 				UARTdata = USART_GetChar();		//чтение значения режима работы
		// 			}
		// 			display_dnum(mode);					//показать номер режима
		// 			TCNT1 = 0;
		// 			cntTabloUpdate = 0;					//задержать обновление табло
		// 			_LED1(0);
		// 			break;
		// 		}
		// 		//отобразить номер табло (установка заблокирована)
		// 		case SETTADR: {
		// 			if (USART_GetRxCount() == 0) _delay_ms(200);
		//
		// 			if (USART_GetRxCount())	{
		// 				UARTdata = USART_GetChar();		//чтение адреса
		// 			}
		// 			display_dnum(TADR);					//показать адрес табло
		// 			TCNT1 = 0;
		// 			cntTabloUpdate = 0;					//задержать обновление табло
		// 			break;
		// 		}
		// 		//программный сброс
		// 		case RESET: {
		// 			_LED1(1);
		//
		// 			if (USART_GetRxCount() == 0) _delay_ms(200);
		//
		// 			if (USART_GetRxCount())	{
		// 				UARTdata = USART_GetChar();		//чтение оператора команды
		//
		// 				if (UARTdata == 1) SOFTRESET = 1;
		// 			}
		// 			_LED1(0);
		// 			break;
		// 		}
	}
}

uint8_t USART_handle(void)
{
	uint8_t CORRECT=1;
	getADR = USART_GetChar();	//чтение байта 1: адрес
	if (getADR != BROADCAST) {	//если не широковещательный
		if (getADR == TADR) {		//сравнение с адресом контроллера
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
		//отображение значения яркости для тестов---------------------
		//#ifdef DEBUG
		//{
		//display_dnum(v_ADC); //отображение значения яркости
		//cntTabloUpdate = cntT1 + 1000; //отображаем значение яркости 1 секунду
		//}
		//#endif
		//отображение значения яркости для тестов---------------------
		
		set_Bright(v_ADC, 2);		//установить режим яркости по освещенности
		if (BriMode != preBriMode)	//если режим яркости изменился
		{
			preBriMode = BriMode;
			TXDATAEN |= 0x01;
		}
		adc_counter = 0;
		v_ADC = 0;
		_flash_LED1(3, 20);
	}
}

// void _SOFTRESET(void)
// {
// 	//	WDTCSR=0x00;
//
// 	DDRLED |= _BV(LED1);//|_BV(LED2);
// 	_flash_LED1(3,100);
//
// 	mode=0;
//
// 	cli();
//
// 	for (j = 0; j < 4; j++)
// 	{
// 		eeprom_write_byte(EEDigit + j, j + 1);
// 	}
// 	for (j = 0; j < 4; j++)
// 	{
// 		Digit[j] = eeprom_read_byte(EEDigit + j);
// 	}
//
// 	sei();
// 	display_10code(Digit[0], Digit[1], Digit[2], Digit[3]);
// }


//установка начальных значений переменных
void Initialize(void)
{
	cli();
	editNtab = 1;		//значение номера табло
	TADR = eeprom_read_byte(EETab);		//значение адреса табло
	qtTab = eeprom_read_byte(EETab + 1);		//значение количества табло
	isMasterDevice = eeprom_read_byte(EETab + 2); //является ли плата мастер или слэйв
	stelaTabPosition = TADR - TADR0; //номер табло на стеле по порядку

	DDRLED |= (1 << LED1);
	
	digit_sort(2, 3, 0, 1); //сортировка разрядов табло (для удобства подключения шлейфов на плате)
	set_PWC(4); //установка режима поразрядной индикации
	
	cntTabloUpdate = TabloUpdatePeriod;
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
	cntADCSTART = 0;
	adc_counter = 0;
	cntExitProgMode = cntT1 + ONEMIN;
	
	BriMode = 2;
	BriStep = _getBriStep(BriLevels[BriMode]);
	set_Bright(BriLevels[BriMode], 3);
	
	initRC5();
	initBTN();			//обработка в T1_OVF
	init595();
	initPWM();
	initT1(0b00001010, 1843);		//16-битный, clk/2, режим СТС, OCR1A = 1843, период  1 мс
	adc_init();
	USART_Init(USART_DOUBLED, 19200);
	USART_FlushTxBuf();

	//display_dnum(TADR);
	display_7code(SYMB_E, SYMB_L, SYMB_P, SYMB_I); //вывели на экран режим мастер-слэйв
	
	TXDATAEN |= 0x02;	//разрешение TX глобально (2й бит)
	//	TXDATAEN &=~ 0x02;	//запрещение TX глобально (2й бит)
	WREEN |= 0x02;		//разрешить запись яркости в ЕЕ

	_RS485(0); //инициализация интерфейса RS485
	_RS485(1);	 //RX_EN + TX_DIS

	_flash_LED1(3, 50);
	_delay_ms(2000);
}

/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
int main(void)
{
	Initialize();
	setBoardAddr(); //установка адреса табло перед главным циклом

	while(1)
	{
		wdt_enable(WDTO_2S); //вочдог таймер на 2 секунды, если зависли то хардресет (не забываем про фьюзы)
		//программный сброс
		// 		if (SOFTRESET) {
		// 			if (WDTCR & (1 << WDE)) {
		// 				while(1);
		// 			}
		// 			else {
		// 				Initialize();
		// 				_SOFTRESET();
		// 			}
		// 		}
		
		if (isMasterDevice) 
		{
			//запуск измерений АЦП
			if ((cntT1 > cntADCSTART) && ADCENABLE)
			{
				//PrintStringToSerial("INTO ADC FINE!");
				cntADCSTART=cntT1 + 20;	//примерно с периодом 0.02 сек
				ADCluxmeter(ADCLUXCH); //измерение и отображение значения яркости
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
		}
		
		//проверка переполнения переменных, использующих таймер-счетчик
		if (cntT1 > cntMaxPeriod)
		{
			cntADCSTART -= cntT1;				//корректировка таймера
			cntTabloUpdate -= cntT1;		//
			cntExitProgMode -= cntT1;		//
			cntBlinkTimer -=  cntT1;
			whileLoopCounter -= cntT1;
			cntT1 = 0;
		}
		
		//чистка буфера каждые 24 часа
		if (cntT1 > TxRxBufCleanPeriod) {
			USART_FlushTxBuf();
			USART_FlushRxBuf();
			TxRxBufCleanPeriod = cntT1 + ONEDAY;
		}
		
		//обновление данных на табло по достижению счетчика cntT1 + TabloUpdatePeriod
		if (cntT1 > cntTabloUpdate && isSettingsMode == DEFAULTMODE) {
			_LED1(1);
			cntTabloUpdate = cntT1 + TabloUpdatePeriod;
			
			//Реализация таймера для тестов-----------------
			//if (Digit[3] == 10) {
			//Digit[3] = 0;
			//Digit[2]++;
			//
			//}
			//if (Digit[2] >= 6) {
			//Digit[2] = 0;
			//Digit[1]++;
			//}
			//if (Digit[1] == 10) {
			//Digit[1] = 0;
			//Digit[0]++;
			//}
			//if (Digit[0] >= 6) {
			//Digit[0] = 0;
			//}
			//display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3]++, maskDegVal);	//прямое отображение
			//Реализация таймера для тестов-----------------
			
			display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3], maskDegVal);	//прямое отображение
			_LED1(0);
		}
		
		//запись данных на текущем табло в ЕЕ если по 485 пришла команда на изменение цены
		if (WRITEEEDIG && (WREEN & 0x01)) {
			_LED1(1);
			WRITEEEDIG = 0;
			cli();
			for (uint8_t i = 0; i < 4; i++)
			eeprom_write_byte(EEDigit + i, Digit[i]);
			sei();
			_LED1(0);
		}
		
		//чтение данных табло в ОЗУ из ЕЕ
		if (READEEDIG) {
			_LED1(1);
			READEEDIG = 0;
			cli();
			for (uint8_t i = 0; i < 4; i++)
			Digit[i] = eeprom_read_byte(EEDigit + i);
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

		//таймер выхода из режима isSettingsMode == 1 или isSettingsMode == 2 и т.д
		if (isSettingsMode && (cntT1 > cntExitProgMode))
		{
			ExitButtonClickProgMode();		//запуск процедуры выхода из режима программирования
		}
		
		//бесконечная моргалка для табло (период MILLIS_500)
		if ((isSettingsMode == SETMASTERSLAVEMODE || 
			 isSettingsMode == SETPINCODEMODE ||
			 isSettingsMode == INFOMODESETADDR ||
			 (isSettingsMode == SETPRISEMODE && TADR == (TADR0 + editNtab))) && (cntT1 > cntBlinkTimer + MILLIS_500)) {
			if (isBlinked == 0) {
				set_Bright(BriValues[3], 5); //яркость текущего табло
				isBlinked = 1;
			}
			else {
				set_Bright(BriValues[11], 5); //яркость текущего табло
				isBlinked = 0;
			}
			cntBlinkTimer = cntT1;
		}
						
/*Алгоритм приема команд с ИК пульта;
Задачи отследить: - короткое нажатие
- длительное нажатие
- паузу между нажатиями
- повторное нажатие

Короткое нажатие выполняется сразу же если не было длительного нажатия
Длительным нажатием считается удержание более 1 секунды
При длительном нажатии сначала проверяется условие на короткое нажатие
Сброс пойманной команды с ИК пульта выполняется при отсутствии сигнала в течении 150 мс, т.к период пакета ИК пульта 115 мс по даташиту;
*/
if (rc5_data)
{
	//запомнили команду с пульта
	irControlData = rc5_data;
	//если таймер начал отсчет (запускается сразу после короткого нажатия)
	if (irLongPressTimer > 0) {
		//отсчитываем по 1 сек
		if (cntT1 > irLongPressTimer + ONESEC) {
			//считаем количество итераций по 1 секунде таймера
			irLongPressCounter++;
			//обрабатываем каждую секунду удерживаемую кнопку
			irManage();
		}
	}
	else {
		//при длительном нажатии самый первый раз попадем сюда
		//парсим код и запускаем таймер длительного нажатия кнопки
		irManage();
	}
	//пока нажата кнопка летят новые пакеты, старый обнуляем
	rc5_data=0;
	//пока нажата кнопка не зависим от 150мс и накидываем общий таймер
	whileLoopCounter = cntT1;
}
else {
	//если отпустили кнопку то каждые 150 мс будем обнуляться
	if (cntT1 > whileLoopCounter + 150) {
		//сбросили сохраненный сигнал ИК
		irControlData = 0;
		whileLoopCounter = cntT1;
		//сбросили количество секунд длительного нажатия
		irLongPressCounter = 0;
		//сбросили таймер длительного нажатия
		irLongPressTimer = 0;
	}
}
	}//while(1)
}//main()

void irManage() {
	Rfunc = ((irControlData & 0x3F)|(~irControlData >> 7 & 0x40)); //Выделяем только код команды
	
	//если плата в режиме мастера то ловим пульт и обрабатываем команды под мастера
	if (isMasterDevice)	{
		RCommand(Rfunc, isSettingsMode);
	}
	//настройка редима работы платы устанавливается пультом и на слэйве и на мастере
	setDeviceType(Rfunc); //Устанавливаем режим работы платы (мастер или слэйв)
	irLongPressTimer = cntT1;
}

//НАстройка режима работы платы с пульта (мастер или слэйв)
void setDeviceType(uint8_t irCode) {
	switch(irCode) {
		case RC5INFO:
			ADCENABLE = 0; //Зарпещаем измерение датчика освещенности пока находимся в режиме конфигурирования
			cntExitProgMode = cntT1 + ONEMIN; //обновили счетчик выхода по таймеру из всех режимов пульта
			infoModeManage(); //меню настройки контроллера - адрес и состояние слэйв-мастер
			break;
		case RC5MENU:
			deviceTypeManage(); //обрабатываем нажатие меню
			break;
		case RC5DIG0:
		case RC5DIG1:
		case RC5DIG2:
		case RC5DIG3:
		case RC5DIG4:
		case RC5DIG5:
		case RC5DIG6:
		case RC5DIG7:
		case RC5DIG8:
		case RC5DIG9:
			slaveDigButtonManage(irCode); //обрабатываем нажатие ввода пинкода и конфигурируем режим слэйв
			break;
		case RC5TEXT: {
			//когда отловили именно нажатие кнопки текс на пульте (код 42) то посылаем в функцию именно индекс этого символа в массиве ABCD_T
			//потому что в массив Digit[] записываются именно индексы соответствующие массиву ABCD_T, а далее вызывается функция display_10code_point
			//которая отображает уже символы из массива ABCD_T по индексу
			slaveDigButtonManage(DASHCODE);
			break;
		}
		case RC5TVFORMAT: {
			slaveDigButtonManage(SYMB_SPACEINDX);//вывели на экран команду С1 - что мы в режиме программирования);
			break;
		}
		case RC5EXIT:
			ExitButtonClickProgMode();		//запуск процедуры выхода из режима программирования
			break;
	}
}

//меню настройки контроллера - адрес и состояние слэйв-мастер
void infoModeManage() {
	if (isSettingsMode == DEFAULTMODE || isSettingsMode == INFOMODEDEVSTATE) {
		isSettingsMode = INFOMODEADDR; // перешли в режим ожидания ввода следующей команды
		display_7code(0, ABCD_T[10], ABCD_T[stelaTabPosition], ABCD_T[10]); //вывели на экран номер табло по порядку в виде -1-
	}
	else if (isSettingsMode == INFOMODEADDR) {
		isSettingsMode = INFOMODEDEVSTATE; // перешли в режим ожидания ввода следующей команды
		display_7code(SYMB_P, 0, 0, ABCD_T[isMasterDevice]); //вывели на экран режим мастер-слэйв
	}
	else {
		subMenuItem = 0;
		ExitButtonClickProgMode();
	}
	subMenuItem = isSettingsMode;//сохранили подпункт сервисного меню
}

//обрабатываем нажатие меню в режиме конфигурирования платы под слэйв-мастер
void deviceTypeManage() {
	if (isSettingsMode == INFOMODEDEVSTATE || isSettingsMode == INFOMODEADDR) { //если была нажата кнопка инфо и мы в режиме инфо
		for (uint8_t i = 0; i < 4; i++) {
			DigTmp[i] = eeprom_read_byte(EEDevPinCode + i); //заполняем пинкод во временный массив
		}
		cntExitProgMode = cntT1 + ONEMIN; //обновили счетчик выхода по таймеру из всех режимов пульта
		display_7code(ABCD_T[DigTmp[0]], ABCD_T[DigTmp[1]], ABCD_T[DigTmp[2]], ABCD_T[DigTmp[3]]); //вывели пин код на экран
		isSettingsMode = SETPINCODEMODE; //перешли в режим ввода пинкода
	}
}

//проверка правильности ввода пин кода для конфигурирования платы под слэйв-мастер
//конфигурирование в режиме слэйв
void slaveDigButtonManage(uint8_t bCode) {
	cntExitProgMode = cntT1 + ONEMIN; //обновили счетчик выхода по таймеру из всех режимов пульта
	if (isSettingsMode == SETPINCODEMODE) {
		//проверка на правильность ввода пин кода, если цифра неверная то уходим в стандартный режим
		if (DigTmp[countDigitButtonClick] == bCode) {
			DigTmp[countDigitButtonClick] = SYMB_SPACEINDX; //если правильно ввели цифру то зануляем ее чтобы убрать с экрана
			set_Bright(BriValues[11], 5); //яркость текущего табло подтверждение ввода символа
			display_7code(ABCD_T[DigTmp[0]], ABCD_T[DigTmp[1]], ABCD_T[DigTmp[2]], ABCD_T[DigTmp[3]]); //убираем с экрана правильно введенные цифры
			} else {
			ExitButtonClickProgMode();
			return;
		}
		countDigitButtonClick++;
		if (countDigitButtonClick > 3) { //если закончили ввод всех 4х цифр пин кода
			if (subMenuItem == INFOMODEDEVSTATE) { //смотрим пункт меню в котором находимся - если хотим настраивать мастер-слэйв
				isSettingsMode = SETMASTERSLAVEMODE; //переходим в режим настройки платы под слэйв-мастер
				display_7code(SYMB_P, 0, 0, ABCD_T[isMasterDevice]); //вывели на экран текущий режим мастер-слэйв
			}
			else if (subMenuItem == INFOMODEADDR) { //если хотим настраивать порядок на стеле для текущей платы
				isSettingsMode = INFOMODESETADDR;
				display_7code(0, ABCD_T[10], ABCD_T[stelaTabPosition], ABCD_T[10]); //моргаем параметром который хотим сменить - порядок на стеле
			}
			return;
		}
	}
	else if (isSettingsMode == SETMASTERSLAVEMODE && bCode < 2) { //тут отрабатываем нажатия на кнопки 0-1 для смены режима слэйв-мастер bCode < 2 - только цифры 0-1
		isMasterDevice = bCode; //присвоили нажатую кнопку 0-1 режиму платы
		display_7code(SYMB_P, 0, 0, ABCD_T[isMasterDevice]); //вывели на экран режим мастер-слэйв
	}
	else if (isSettingsMode == INFOMODESETADDR) {
		stelaTabPosition = bCode;
		display_7code(0, ABCD_T[10], ABCD_T[stelaTabPosition], ABCD_T[10]); //вывели параметр который сменили
	}
	else if (isSettingsMode == INFOMODEADDR) { //если мы в режиме инфо и сработал признак длительного нажатия на цифру
		 if (irLongPressCounter >= WAITTIME) {
			 display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3], maskDegVal);	//прямое отображение
			 isSettingsMode = SETPRISEMODE;
			 editNtab = stelaTabPosition; //для редактирование слэйва именно текущего табло надо его выбрать
		 }
		 else if (bCode != stelaTabPosition) ExitButtonClickProgMode();
	}	
	else if (isSettingsMode == SETPRISEMODE && !isMasterDevice) { //если в режиме установки цены и мы в слэйве
		DigitButtonClickProgMode(bCode);
	}
	else if (isSettingsMode != SETPRISEMODE && 
			 isSettingsMode != SETTABCOUNTMODE && 
			 isSettingsMode != INFOMODEADDR &&
			 isSettingsMode != DEFAULTMODE) { //исключаем ложный выход в режимах мастера
		ExitButtonClickProgMode(); //если на пульте нажали кнопку отличную от последовательности входа в режим выбора состояния платы то выходим (info-menu-pin-slavemaster)
	}
}

//Алгоритм установки адреса платы кнопкой на самой плате - для настройки порядка табло на стеле
void setBoardAddr() {
	//initT1(0x00, 0);	//остановка таймера для исключения ложного срабатывания
	cntT1 = 0;
	REPRESSBTN1 = 1;
	
	while (PRESSBTN1) //пока кнопка BTN1 не будет отпущена
	{
		//вход в режим инициализации - нажатие кнопки в течение 3 сек
		//исключаем ошибку случайного замыкания кнопки при сбросе питания (например, из-за снега или дождя)
		//при нажатии кнопки более 15ти сек - выход из режима инициализации
		if (REPRESSBTN1)		//однократное выполнение при длительном нажатии
		{
			_LED1(1);
			//initT1(0x05, 0);		//запускаем таймер длительности нажатия
			REPRESSBTN1 = 0;
		}
		if (cntT1 > ONESEC && INITtab < 3)		//подтверждение запуска инициализации при удержании кнопки примерно 1 сек
		{
			//cntT1++; //чтобы не зашлю сюда повторно в цикле
			INITtab = 3;	//установка параметра для инициализации табло
			display_7code(0x08, 0x08, 0x08, 0x08);
			_flash_LED1(3, 200);
		}
		if (cntT1 > ONESEC * 10)		//слишком длительное нажатие (например при замыкании из-за влаги)
		{
			INITtab = 0;	//пропуcтить инициализацию при превышении времени удержания кнопки
			cntT1 = 0;
			_flash_LED1(10, 100);
			break;				//выход из цикла и запуск работы табло
		}
		_delay_ms(10); //без этой задержки не выходит из цикла при отпускании кнопки PRESSBTN, возможно из-за оптимизации кода компилятором
	}
	REPRESSBTN1=1;
	_LED1(0);
	cntT1 = 0;
	//выполнить инициализацию - определение номера табло Ntab
	while(INITtab)
	{
		//при нажатии BTN1 инкремент Ntab
		if (PRESSBTN1)
		{
			if (REPRESSBTN1)
			{
				_flash_LED1(1, 20);
				REPRESSBTN1 = 0;
				WRITEEENTAB = 1;
				if (editNtab < MAXNTAB)
				{
					editNtab++;	//максимальный номер табло MAXNTAB (12)
				}
				else
				{
					editNtab = 0;
				}
				display_dnum(editNtab);
				INITtab = 3;		//сброс выхода из инициализации
				cntT1 = 0; TCNT1 = 0;		//сброс счетчика Т1
			}
		}
		else REPRESSBTN1=1;
		
		//индикация номера табло и отсчет завершения инициализации
		if (cntT1 >= ONESEC * 3)
		{
			if (editNtab) _flash_LED1(editNtab, 200);
			else _flash_LED1(2, 30);
			INITtab--;			//завершение инициализации по обнулении параметра INITIALIZATION
			cntT1 = 0;          //сброс счетчика Т1
			TADR = TADR0 + editNtab;
			display_dnum(TADR);
		}
	}
	//при завершении инициализации запись в ЕЕ номера табло
	if (WRITEEENTAB) {
		TADR = TADR0 + editNtab;
		WRITEEENTAB=0;
		cli();
		eeprom_write_byte(EETab, TADR);
		sei();
	}
	
	Initialize();
	_delay_ms(500);		//отображение адреса табло 500мс
}

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
			ExitButtonClickProgMode(); //если неверно ввели то выходим из режима настроек
			break;
		}
		default: {
			ExitButtonClickProgMode(); //если неверно ввели то выходим из режима настроек
			break;
		}
	}
	_delay_ms(300);
}

//Задаем значения на выбранном табло в режиме редактирования или же выбираем следующее табло при повторном нажатии на Power или Ok
void SetSettingsFromIrControl(uint8_t func)
{
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
		case RC5TEXT: {
			//когда отловили именно нажатие кнопки текс на пульте (код 42) то посылаем в функцию именно индекс этого символа в массиве ABCD_T
			//потому что в массив Digit[] записываются именно индексы соответствующие массиву ABCD_T, а далее вызывается функция display_10code_point
			//которая отображает уже символы из массива ABCD_T по индексу
			DigitButtonClickProgMode(DASHCODE);
			break;
		}
		case RC5TVFORMAT: {
			DigitButtonClickProgMode(SYMB_SPACEINDX);//вывели на экран команду С1 - что мы в режиме программирования);
			break;
		}
		
		case RC5RED: {
			ColorButtonsClick(SHIFTREDBUTTON);
			break;
		}
		case RC5GREEN: {
			ColorButtonsClick(SHIFTGREENBUTTON);
			break;
		}
		case RC5BLUE: {
			ColorButtonsClick(SHIFTBLUEBUTTON);
			break;
		}
		case RC5YELLOW: {
			ColorButtonsClick(SHIFTYELLOBUTTON);
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
			break;
		}
	}
}

void ColorButtonsClick(uint8_t shiftVal)
{
	cntExitProgMode = cntT1 + ONEMIN;
	if ((maskDegVal & (1 << shiftVal)) != 0) {
		//BITCLEAR(maskDegVal, shiftVal);
		maskDegVal &= ~(1 << shiftVal);
	}
	else {
		//BITSET(maskDegVal, shiftVal);
		maskDegVal |= (1 << shiftVal);
	}
	display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3], maskDegVal);	//прямое отображение
}

void PowerButtonClickProgMode()
{
	if (countDigitButtonClick > 0) {
		EepromWritePrice(editNtab); //сохранили цену табло в еепром если редактировали
	}
	
	editNtab++;
	
	//проверка на превышение Ntab > qtTab
	if (editNtab > qtTab) {
		editNtab = 1; //тогда переключаемся снова на первое табло
	}
	set_Bright(BriValues[MIDDLE_BRIGHT], 4); //яркость всех табло
	DoBlinking(editNtab); //начинаем моргать текущим табло
	countDigitButtonClick = 0;
}

void OkButtonClickProgMode()
{
	if (countDigitButtonClick > 0) {
		EepromWritePrice(editNtab); //сохранили цену табло в еепром  если редактировали
	}
	
	editNtab--;
	
	//проверка на Ntab < 1
	if (editNtab < 1) {
		editNtab = qtTab; //тогда присваиваем индекс последнего табло чтобы переключиться на него
	}
	
	set_Bright(BriValues[MIDDLE_BRIGHT], 4); //яркость всех табло
	DoBlinking(editNtab); //начинаем моргать текущим табло
	countDigitButtonClick = 0;
}

void ExitButtonClickProgMode()
{
	//если мы верно ввели количество табло в настройках то у нас isSettingsMode сохранится = 1 и мы сохраняемся в еепром + подтверждаем верный ввод еще одним морганием
	if (isSettingsMode == SETTABCOUNTMODE) {
		eeprom_write_byte(EETab + 1, qtTab);
		display_7code(0x39, ABCD_T[2], ABCD_T[10], ABCD_T[qtTab]); //вывели на экран новое значение количества табло
		DoBlinking(1); //поморгали ведущим табло в течении 3 секунд
	}
	if (isSettingsMode == SETPRISEMODE && countDigitButtonClick > 0) { //если режим для ввода цен
		EepromWritePrice(editNtab); //сохранили цену табло в еепром  если редактировали
	}
	if (isSettingsMode == SETMASTERSLAVEMODE) {//если режим установки мастера-слэйа для платы
		eeprom_write_byte(EETab + 2, isMasterDevice);
	}
	if (isSettingsMode == INFOMODESETADDR) {//если режим смены адреса табло (позиции на стеле)
		eeprom_write_byte(EETab, stelaTabPosition + TADR0);
		TADR = stelaTabPosition + TADR0;
	}
	isSettingsMode = DEFAULTMODE;
	ADCENABLE = 1;
	READEEBRI = 1; //читаем яркость из ЕЕ
	countDigitButtonClick = 0;
}

void DigitButtonClickProgMode(uint8_t buttonCode)
{
	//при первом нажатии CountDigitButtonClick == 0
	//выполняется сброс данных на редактируемом табло и установка первой цифры
	//переход к редактированию следующей цифры: CountDigitButtonClick++
	//далее CountDigitButtonClick меняется циклично от 1 до 4
	//ввод значений в массив DigTmp[CountDigitButtonClick-1]=RC5DIG0;
	if (countDigitButtonClick > 0) {
		countDigitButtonClick++;
		if (countDigitButtonClick > 4) countDigitButtonClick = 1;
		DigTmp[countDigitButtonClick - 1] = buttonCode;
	}
	else {
		DigTmp[countDigitButtonClick] = buttonCode;
		DigTmp[countDigitButtonClick + 1] = DASHCODE;
		DigTmp[countDigitButtonClick + 2] = DASHCODE;
		DigTmp[countDigitButtonClick + 3] = DASHCODE;
		countDigitButtonClick = 1;
	}
	//если редактируемое табло - текущее табло, то изменяем значения
	if (TADR == (TADR0 + editNtab)) {
		Digit[0] = DigTmp[0];
		Digit[1] = DigTmp[1];
		Digit[2] = DigTmp[2];
		Digit[3] = DigTmp[3];
		display_10code_point(Digit[0], Digit[1], Digit[2], Digit[3], maskDegVal);	//прямое отображение
	}
	//отправка новой цены на редактируемое табло
	TxDATA(TADR0 + editNtab, RXTDATA, DigTmp[0], DigTmp[1], DigTmp[2], DigTmp[3]);
	_delay_ms(300);
}

//обработчик нажатия кнопки на ИК пульте ВПЕРВЫЕ
void IrControlButtonClick(uint8_t func)
{
	switch (func) {
		//нажатие кнопки Power
		case RC5POWER:
		case RC5OK: {
			editNtab = 1; //сброс номера табло
			isSettingsMode = SETPRISEMODE; //если нажали Power или OK то взводим флаг что мы в режиме редактирования текущего табло
			cntExitProgMode = cntT1 + ONEMIN;
			countDigitButtonClick = 0; //сброс нажатий на цифры
			ADCENABLE = 0; //Зарпещаем измерение датчика освещенности пока находимся в режиме программирования
			set_Bright(BriValues[MIDDLE_BRIGHT], 4);  //яркость всех табло
			DoBlinking(editNtab); //начинаем моргать текущим табло (самым первым по индексу Ntab, выше = 1)
			break;
		}
		//если нажата кнопка меню то переходим в режим настроек
		case RC5MENU: {
			display_7code(SYMB_C, ABCD_T[2], ABCD_T[10], ABCD_T[qtTab]); //вывели на экран команду и количество табло С2 - 5 что мы в режиме настроек
			isSettingsMode = SETTABCOUNTMODE; // подняли флаг что мы в режиме настроек но ввод еще не закончен, потому как когда нажали меню мы сбрякаемся из этой функи сразу в проверку окончания ввода
			ADCENABLE = 0; //Зарпещаем измерение датчика освещенности пока находимся в режиме конфигурирования
			DoBlinking(stelaTabPosition); //поморгали ведущим табло в течении 3 секунд
			cntExitProgMode = cntT1 + ONEMIN;
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
			_flash_LED1(4, 30);
			break;
		}
	}
}

//передача данных с ИК пульта
void RCommand (uint8_t func, uint8_t _isSettingsMode) {
	_flash_LED1(1, 30); //Моргнцть один раз что команда принята с пульта
	//если мы в режиме настроек то ждем ввод количества табло и выходим из этого режима
	if (_isSettingsMode == SETTABCOUNTMODE) {
		SetCountTabs(func); //Задаем количество табло по нажатию кнопки с ИК пульта
	}
	else if (_isSettingsMode == DEFAULTMODE) {
		//обработчик нажатия кнопки на ИК пульте
		IrControlButtonClick(func);
	}
	else if (_isSettingsMode == SETPRISEMODE) {
		//Задаем значения на выбранном табло в режиме настроек или же выбираем следующее табло при повторном нажатии на Power
		cntExitProgMode = cntT1 + ONEMIN;
		SetSettingsFromIrControl(func);
	}
}

void DoBlinking(uint8_t _nTab) {
	for (uint8_t i = 0; i < 3; i++) {
		wdt_enable(WDTO_2S); //вочдог таймер на 2 секунды, если зависли то хардресет (не забываем про фьюзы)
		if (TADR == (TADR0 + _nTab)) set_Bright(BriValues[0], 5); //яркость текущего табло
		
		TxDATA(TADR0 + _nTab, BRIGHT, BriValues[0], TADR0 + _nTab, BRIGHT, BriValues[0]);
		_delay_ms(200);
		
		if (TADR == (TADR0 + _nTab)) set_Bright(BriValues[11], 5); //яркость текущего табло
		
		TxDATA(TADR0 + _nTab, BRIGHT, BriValues[11], TADR0 + _nTab, BRIGHT, BriValues[11]);
		_delay_ms(200);
	}
}

void EepromWritePrice(uint8_t _nTab)
{
	if (TADR == (TADR0 + _nTab)) {
		cli();
		for (uint8_t j = 0; j < 4; j++)
		eeprom_write_byte(EEDigit + j, Digit[j]);
		sei();
	}
	//команда RX на сохранение в ЕЕ на редактируемом табло
	TxDATA(TADR0 + _nTab, RXWRITEEE, 0x01, TADR0 + _nTab, RXWRITEEE, 0x01);
}