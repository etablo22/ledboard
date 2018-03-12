/*
 * Stela_4.c
 *
 * Created: 18.01.2016 9:11:19
 * Autor: BYA
 * controller for LED-indicator (AZS tablo)
 20151109	: ���������� ����������� ������� �� ������ �0 (PC0 -> 0)
 20160316	: ������ ������ BTN0 �� BTN1 ��-�� ������ ������������ �����: �� ��������� ������� � PC0
 			: ��������� ������ ���������� ��������� ������ ��� ������ ������� (��������, ��-�� ����� ��� �����)
 			: ��� ������ ������� ������ - ����� �� ������ �������������
tab_rx		: ������ ��� ������ �����: ������ ����� ������

20161012	: ����������� � ���������� Ind_lib (v.2.0:����� ����������� ���������)
			: ��������� ��������� set_pwc - ��������� �������� pwc

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

#include "Ind_lib_v2.0.h"

//������� ���������
#define BRIGHT		0xA0	//������� ���������� �������� ������� = 160
#define BRIGHTDWN	0xA1	//������� ��������� �������� ������� = 161
#define BRIGHTUP	0xA2	//������� ��������� �������� ������� = 162
#define RXNtab		0xA9	//������� ���������� ���������� �����
#define RXTDATAall	0xAA	//������� ������ ������ ��� ���� ����� 32� (170)
#define RXTDATA		0xAB	//������� ������ ������ �� ������ (171)
#define RXWRITEEE   0xAC    //������� �� ������ � �� ������������� ����
#define SETINDIC	0x96	//���������� (��������) ����� ���������
#define SETDSORT	0x97	//���������� (��������) ������� ���������� �������� �����
#define SETMODE		0x98	//���������� (��������) ����� ������
#define SETTADR		0x99	//���������� (��������) ����� �����
#define RESET		0xFE	//����������� ����� ������

//������ ��������� ����
#define BROADCAST	0xFF	//����� ����������������� �������� = 0
#define CUADR		0x64	//����� ����� ���������� (CUnit)
#define TADR0		0x64	//����� �������� ����� (������ �������)


//���������
#define PORTLED		PORTD	//���� ����������
#define DDRLED		DDRD	//���� ����������
#define LED1		PD4		//���� ����������
//#define LED2		PD5		//���� ����������
#define DDR_RS485	DDRD	//���� ����������
#define PORT_RS485	PORTD	//���� ����������
#define PTXEN		PD2		//���� ����������
#define PRXDIS		PD3		//���� ����������
#define RXD			PD0		//���� UART - ��������
#define TXD			PD1		//����� UART - ����������
#define MAXINDMODE	1		//���������� ������� ���������

uint8_t TADR = 101;		//����� �����
const uint8_t MAXNTAB = 12;
uint8_t Ntab = 1, INITtab = 0, qtTab = 0;
uint8_t EEMEM EETab[3] = {1, 101, 5};		//Ntab - ����� �����, TADR - ����� �����, qtTab - ���������� �����

int8_t j=0,it1=0,mode=0,ind_mode=0;
uint8_t dispcounter,OCRtest,TIMER1CNT;
uint8_t cnt_btn0,cnt_btn1;
int8_t TestCNT, INITIALIZATION=0;

uint8_t Digit[4]={1,2,3,4}, PointMask=0x0F;
uint8_t EEMEM EEDigit[4]={1,2,3,4}, EEDsort[5]={0,1,2,3,0x0F}, EEdata[5]={1,255,0,0,101};//, EEBri[1]={255}, EE_indmode[1]={0}, EEmode[1]={0};

// uint8_t EEMEM EEBriData[3] = {12,40,86}; //������� ������� {������,�������, �������}
// uint8_t BriMode,preBriMode, BriLevels[3]={12,40,86}, BriValues[12]={9,12,16,20,25,32,40,49,60,73,86,100}, BriStep;

//FYLS-5050LULW3C R150 pwc=2 - �������� 5� 60�� (310LUX)
//FYLS-5050UR3C R300 pwc4 - �������� 6� 75�� (170LUX)
// uint8_t EEMEM EEBriData[3] = {17,65,146}; //������� ������� {������,�������, �������}
// uint8_t BriMode,preBriMode, BriLevels[3]={17,65,146}, BriValues[12]={10,13,17,22,29,38,50,65,85,112,146,200}, BriStep;

//FYLS-5050UR3C R300 pwc4 - �������� 8� 100�� (200LUX)
//FYLS-5050LULW3C R150 pwc=2 - �������� 6� 75�� (350LUX)
uint8_t EEMEM EEBriData[3] = {5,95,214}; //������� ������� {������,�������, �������}
uint8_t BriMode,preBriMode, BriLevels[3]={5,95,214}, BriValues[12]={5,30,40,50,62,77,95,118,146,181,214,250}, BriStep;

//EEdata - 4�: ������ ��� ��������� �����
//{����� �����, �������, ����� ���������, ����� ������,����� �����} (����� � ����� ����� - ��������� ��� ����������)

uint8_t getADR, UARTcommand, UARTdata, ADCch;
//����� ������ ���������
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
//		case 0x20:		{return 11;break;}		//������ - ������
//		case 0x2A:		{return 12;break;}		//��������� - ���� ������� 
//		case 0x2B:		{return 11;break;}		//���� - 
//		case 0x2C:		{return 13;break;}		//������� - ������ �������������
		case 0x2D:		{return 10;break;}		//����� - �����
//		case 0x2E:		{return 14;break;}		//����� - ������ � ������
//		case 0x2F:		{return 11;break;}		//���� - 
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
		default:		{return 11;break;}		//������ - null
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

//��������� DEC  -> BCD
char _dec2bcd(char num)
{
	return ((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
int16_t _bcd2dec(int16_t num)
{
	return ((num/16 * 10) + (num % 16));
}

//�������� � ������������� ������ ��� �����
uint8_t _CORRECT(uint8_t code, uint8_t digit, int8_t codetype)
{
	uint8_t tdata=code;
	code=digit;
	//codetype: ��� ���������
	//0: �����������
	//1: ���������� (���� �����)
	//2: ASCII
	//3: ABCD - ��� 7-����������� ����������
	//4: Pult_code - ��������� ������ (������� �.�.)
	switch (codetype) {
		case 1: if ((tdata>=0)&&(tdata<=MAXDIGNUMBER)) code=tdata;break;	//���������� �������� �����
		case 2: code=_ascii2dec(tdata);break;								//�������������� �� ������� ASCII
		case 3: code=_ABCD2dec(tdata);break;								//�������������� �� ���� 7-����.
		case 4: code=_CONVERT_pult_code(tdata);break;						//�������������� �� ���� Pult_code
		default: {
			if ((tdata>=0)&&(tdata<=MAXDIGNUMBER)) code=tdata;				//���������� �������� �����
			else if ((tdata>0x29)&&(tdata<0x40)) code=_ascii2dec(tdata);			//�������������� �� ������� ASCII
			break;
		}
	}
	return code;										//������������ �������� �������� �� digit
}

//������������ ���������� LED1 �� ����� PORTLED
void _LED1(int8_t mLED1)
{
	switch (mLED1){
		case 0: {PORTLED|= _BV(LED1); break;}
		case 1: {PORTLED&=~_BV(LED1); break;}
		default:{break;}
	}
}

//������� ���������� LED1: num ��� � ��������� delays5ms, ������� ����������� � ��
void _flash_LED1(uint8_t num, uint8_t delays5ms)
{
	delays5ms=delays5ms/5;	//�������� ������ 5��
	_LED1(0);
	//� ������� _delay_ms() �� ���������� <util/delay.h> � �������� ��������� �� ����� ���� ������� ����������,
	//		����������� ������ ���������: ������� �������� �� �����
	for (int n=0;n<num;n++)
	{
		for (int dl=0;dl<delays5ms;dl++) _delay_ms(5);
		_LED1(1);
		for (int dl=0;dl<delays5ms;dl++) _delay_ms(5);
		_LED1(0);
	}
}

//����������� ������ ���� �� �������
uint8_t _getBriStep(uint8_t level)
{
	uint8_t bs=0;
	while ( bs<(sizeof(BriValues)-1) && BriValues[bs]<level )
	{
		bs++;
	}
	return bs;
}

//���������� ����������� UART-RS485
void _RS485 (int8_t type)
{
	switch(type){
		case 0: {
			// type=="INIT"	//������������� UART-RS485
			DDR_RS485 |= (1<<PTXEN)|(1<<PRXDIS);  //������������ ���������� ����������� UART
			UCSRB |= (1<<TXCIE);
			sei();

			//�������������
			MCUCR &=~(1<<PUD);	//��������� ���������� ������������� ���������� �� ������
			PORT_RS485 |= (1<<RXD);	//������������� �������� �� ����� RXD ��� �������� �����
			break;
		}
		case 1:{
			//type=="RXEN"	//���������� ������ �� RS485
			PORT_RS485 &=~ (1<<PRXDIS);
			break;
		}
		case 2:{
			//type=="TXEN"	//���������� �������� �� RS485
			PORT_RS485 |= (1<<PTXEN);
			break;
		}
		case 10:{
			//type=="RXDIS"	//���������� ������
			PORT_RS485 |= (1<<PRXDIS);
			break;
		}
		case 20:{
			//type=="TXDIS"	//���������� ��������
			PORT_RS485 &=~ (1<<PTXEN);
			break;
		}
		case 11:{
			//type=="RXEN-TXDIS"	//���������� ������ � ������ ��������
			PORT_RS485 &=~ (1<<PTXEN);
			PORT_RS485 &=~ (1<<PRXDIS);
			break;
		}
		case 22:{
			//type=="TXEN-RXDIS"	//���������� �������� � ������ ������
			PORT_RS485 |= (1<<PTXEN);
			PORT_RS485 |= (1<<PRXDIS);
			break;
		}
	}
}

//�� ��������� �������� ���������� �����������
ISR(USART_TXC_vect)
{
	_RS485(11);		//��������� ��������, �������� �����
	WREEN |= 0x01;			//��������� ������ � ��
}

//��������� ������� �1 
//		�1 ������������ ��� �������������� ���������� ������ �� �����
void initT1(uint8_t TCRB)
{
	TCCR1A=0;
	TCCR1B=TCRB;//(1<<CS12)|(1<<CS10);
	TIMSK |= (1<<TOIE1);
	TIMER1CNT=0;
}

//�� ������������ �1 
//		:��������� ���������� TIMER1CNT - ������� �������� �1
//		:����������� ������ �� �����
//		:���������� ������ � ������ ������������ �����
ISR (TIMER1_OVF_vect){
	TIMER1CNT++;
	DISPLAY=1;
}




//�������
void COMMANDS (uint8_t func){
	switch (func){
		//��������� ������� �����
		case BRIGHT:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//������ �������� �������
				OCR2=UARTdata;		//��������� �������
				BriStep=_getBriStep(UARTdata);
			}
			_LED1(0);
			break;
		}
		//��������� ������� �� ��.�������� 
		case BRIGHTUP:{
			_LED1(1);
			if (BriStep<(sizeof(BriValues)-1)) {
				BriStep++;
				OCR2=BriValues[BriStep];	//��������� ������� 
			}
			_LED1(0);
			break;
		}
		case BRIGHTDWN:{
			_LED1(1);
			if (BriStep>0) {
				BriStep--;
				OCR2=BriValues[BriStep];	//��������� �������
			}
			_LED1(0);
			break;
		}
		//����� ������ ��� ��������� �������� �� �����
		case RXTDATA:{
			_LED1(1);
			j=0;
			while ((j<4)&&(USART_GetRxCount()))
			{
				UARTdata = USART_GetChar();	//������ ����� �� ������
				Digit[j] = _CORRECT(UARTdata,Digit[j],0); //������ � ��� ������������������ ��������
				j++;
			}
			DISPLAY=1;		//�������� ���. �� �����
			_LED1(0);
			break;
		}
		case RXWRITEEE: {
			WRITEEEDIG = 1;
			break;
		}
		//��������� ������ ��������� - �������������
		//�������� ����� ���������
		case SETINDIC:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//������ �������� ������ ���������
			}
			display_dnum(ind_mode);
			TCNT1=0; DISPLAY=0;					//��������� ���������� �����
			_LED1(0);
			break;
		}
		//��������� ������ ������ ����� - �������������
		//�������� ����� ������ ������
		case SETMODE:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//������ �������� ������ ������
			}
			display_dnum(mode);					//�������� ����� ������
			TCNT1=0; DISPLAY=0;					//��������� ���������� �����
			_LED1(0);
			break;
		}
		//���������� ����� ����� (��������� �������������)
		case SETTADR:{
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//������ ������
			}
			display_dnum(TADR);					//�������� ����� �����
			TCNT1=0; DISPLAY=0;					//��������� ���������� �����
			break;
		}
		//����������� �����
		case RESET:{
			_LED1(1);
			if (USART_GetRxCount())
			{
				UARTdata = USART_GetChar();		//������ ��������� �������
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

//��������� ���������� ��������� ������� ������ �� ������ PC0 � PC1
void initBTN()
{
	//������ �0 ��� ���������� ������� ������
	TCCR0 |= (1<<CS01); //������������ CLK/8
	TIMSK |= (1<<TOIE0); //���������� ��� �������� �������
	DDRC &= ~((1<<PC0)|(1<<PC1)); //����� � ����� �����
	PORTC &= ~((1<<PC0)|(1<<PC1));
	PRESSBTN0 = 0;
	PRESSBTN1 = 0;
	REPRESSBTN0 = 1;
	REPRESSBTN1 = 1;
}

ISR(TIMER0_OVF_vect){
	//�������� ������� ������ PC0
	if (PINC&0x01) { //���� PINC0=1, �� ��� �������
		cnt_btn0=0;
		PRESSBTN0 = 0; //������ ��������
	}
	else {			//����� - ������ ������
		cnt_btn0++;  //������� �������� ������� ��� ���������� ��������
		if (cnt_btn0>100)	PRESSBTN0 = 1;	//������ ������
	}
	
	//�������� ������� ������ PC1
	if (PINC&0x02) { //���� PINC1=1, �� ��� �������
		PRESSBTN1 = 0;	//������ ��������
		cnt_btn1=0;
	}
	else {			//����� - ������ ������
		cnt_btn1++;  //������� �������� ������� ��� ���������� ��������
		if (cnt_btn1>100)	PRESSBTN1 = 1;
	}
}

//��������� ��������� �������� ����������
void _UPDATEDATA(void)
{
	DDRLED|=_BV(LED1);//|_BV(LED2);
	
	Ntab = eeprom_read_byte(EETab);
	TADR = eeprom_read_byte(EETab + 1);		//�������� ������ �����
	qtTab = eeprom_read_byte(EETab + 2);		//�������� ���������� �����
	
	//���������� �������� �����
	digit_sort(2,3,0,1);	//IM-49
	// digit_sort(3,2,1,0);	//DLD_0.6
	//��������� ������ ����������� ���������
	set_PWC(3);
	
	j = 0; it1 = 0;
	
	CHBRI=0;
	WREEN=1;					//��������� ������ � ��
	WRITEEENTAB=0;
	WRITEEEDIG=0;
	CORRECT=1;
	READEEDIG=1;
	READEEBRI=1;
	SOFTRESET=0;
	
	BriMode=2;
	BriStep=_getBriStep(BriLevels[BriMode]);
	OCR2=BriValues[BriStep];	//��������� �������
	
	init595();
	initPWM();
	initT1(0x05);
	initBTN();
	USART_Init(USART_DOUBLED, 19200);
	USART_FlushTxBuf();

	display_dnum(TADR);
	
	WREEN = 1;		//��������� ������ � ��

	_RS485(0); //������������� ���������� RS485
	_RS485(11);	 //RX_EN + TX_DIS

	_flash_LED1(3,50);
}




/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = */
int main(void)
{
	_UPDATEDATA();
	setBoardAddr(); //��������� ������ ����� ����� ������� ������

    while(1)
    {
		//����������� �����
		if (SOFTRESET) {
			if (WDTCR&(1<<WDE)) while(1);
			else {_UPDATEDATA();_SOFTRESET();}
		}
			
		//���������� ������ �� ����� �� ������������ ������� �1
		if (DISPLAY){
			_LED1(1);
			DISPLAY=0;
			display_10code_point(Digit[0],Digit[1],Digit[2],Digit[3],0x0F);	//������ ����������� 
 			_delay_ms(1);
			_LED1(0);
		}
		
	
		//������ ������ ����� � ��� �� ��
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

		//������ ������ ����� � ��
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
		
		if (USART_GetRxCount()){	//�������� ������� ������ � ������ ������ UART
			_LED1(1);
			CORRECT=1;
			getADR = USART_GetChar();	//������ ����� 1: �����
			if (getADR!=BROADCAST) {	//���� �� �����������������
				if (getADR==TADR) {		//��������� � ������� �����������
	  				_LED1(1);			//���������� ��� ���������� ������
 	 				if (USART_GetRxCount()) UARTcommand = USART_GetChar();	//������ �������� �������
					else {
						_delay_ms(200);		//�������������� �������� ��� �������� �����
						if (USART_GetRxCount()) UARTcommand = USART_GetChar();
						else CORRECT=0;
					}
 					_LED1(0);
				}
				else CORRECT=0;
			}
			else if (USART_GetRxCount()){	//���� ����������������� �����
				UARTcommand = USART_GetChar();	//������ ����������������� �������
				}
			else {
				_delay_ms(200);			//�������������� �������� ��� �������� �����
				if (USART_GetRxCount()) UARTcommand = USART_GetChar();
				else CORRECT=0;
			}
			
			if (CORRECT){
				COMMANDS (UARTcommand);		//���� ������ ���������, �� ��������� �������
 			}
  			_LED1(0);
		}
	}
				
}

void setBoardAddr() {
	
initT1(0x00);	//��������� ������� ��� ���������� ������� ������������
TIMER1CNT=0;
REPRESSBTN1=1;
while (PRESSBTN1) //���� ������ BTN1 �� ����� ��������
{
	//���� � ����� ������������� - ������� ������ � ������� 3 ���
	//��������� ������ ���������� ��������� ������ ��� ������ ������� (��������, ��-�� ����� ��� �����)
	//��� ������� ������ ����� 15�� ��� - ����� �� ������ �������������
	if (REPRESSBTN1)		//����������� ���������� ��� ���������� �������
	{
		_LED1(1);
		initT1(0x05);		//��������� ������ ������������ �������
		REPRESSBTN1 = 0;
	}
	if (TIMER1CNT == 1)		//������������� ������� ������������� ��� ��������� ������ �������� 1 ���
	{
		TIMER1CNT = 2;
		INITIALIZATION = 3;	//��������� ��������� ��� ������������� �����
		display_7code(0x08,0x08,0x08,0x08);
		_flash_LED1(1, 200);
	}
	if (TIMER1CNT > 4)		//������� ���������� �������
	{
		INITIALIZATION = 0;	//�����c���� ������������� ��� ���������� ������� ��������� ������
		TIMER1CNT = 0;
		_flash_LED1(5, 50);
		break;				//����� �� ����� � ������ ������ �����
	}
	_delay_ms(10); //��� ���� �������� �� ������� �� ����� ��� ���������� ������ PRESSBTN, �������� ��-�� ����������� ���� ������������
}
REPRESSBTN1=1;
_LED1(0);

//��������� ������������� - ����������� ������ ����� Ntab
while(INITIALIZATION)
{
	//��� ������� BTN1 ��������� Ntab
	if (PRESSBTN1)
	{
		if (REPRESSBTN1)
		{
			_flash_LED1(1,20);
			REPRESSBTN1=0;
			WRITEEENTAB=1;
			if (Ntab<12)
			{
				Ntab++;	//������������ ����� ����� 12
			}
			else
			{
				Ntab=0;
			}
			display_dnum(Ntab);
			INITIALIZATION=2;		//����� ������ �� �������������
			TIMER1CNT=0; TCNT1=0;		//����� �������� �1
		}
	}
	else REPRESSBTN1=1;
	
	//��������� ������ ����� � ������ ���������� �������������
	if (TIMER1CNT)
	{
		if (Ntab) _flash_LED1(Ntab,200);
		else _flash_LED1(2,30);
		INITIALIZATION--;			//���������� ������������� �� ��������� ��������� INITIALIZATION
		TIMER1CNT=0; TCNT1=0;		//����� �������� �1
		TADR = 100 + Ntab;
		display_dnum(TADR);
	}
}
//��� ���������� ������������� ������ � �� ������ �����
if (WRITEEENTAB) {
	TADR = 100 + Ntab;
	WRITEEENTAB=0;
	cli();
	eeprom_write_byte(EETab,Ntab);
	eeprom_write_byte(EETab+1,TADR);
	sei();
}

_UPDATEDATA();
_delay_ms(500);		//����������� ������ ����� 500��
}