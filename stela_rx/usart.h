//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega16
//
//  Compiler....: GCC
//
//  Description.: ������� USART/UART � ��������� �������
//
//  Data........: 
//
//***************************************************************************
#ifndef USART_H
#define USART_H

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#define USART_NORMAL  0
#define USART_DOUBLED 1

#define SIZE_BUF_RX 128       //������ ������ ��������� ������� - <255
#define SIZE_BUF_TX 128

//****************************************************************************
void USART_Init(uint8_t regime, uint16_t baudRate); //������������� usart`a
uint8_t USART_GetTxCount(void); //����� ����� �������� ����������� ������
void USART_FlushTxBuf(void); //�������� ���������� �����
void USART_PutChar(char sym); //�������� ������ � �����
void USART_SendStr(char * data); //������� ������ �� ��� �� usart`�
void USART_SendStrFl(char const * data); //������� ������ �� ����� �� usart`�
uint8_t USART_GetRxCount(void); //����� ����� �������� � �������� ������
void USART_FlushRxBuf(void); //�������� �������� �����
char USART_GetChar(void); //��������� �������� ����� usart`a 
void USART_GetBuf(uint8_t num, char *buf); //����������� �������� ����� usart`a 
#endif //USART_H