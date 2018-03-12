/*
 * procedures.h
 *
 * Created: 10.02.2016 06:57:41
 *  Author: bya
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#ifndef PROCEDURES_
#define PROCEDURES_

//������� ���������
#define BRIGHT		0xA0	//������� ���������� �������� ������� = 160
#define RXNtab		0xA9	//������� ���������� ���������� �����
#define RXTDATA		0xAA	//������� ������ ������ = 170
#define TXTDATA		0xAB	//������� �������� ������ = 171
#define INDICMODE	0x96	//���������� ������� ���������
#define TESTMODEON	0x97	//���������� ������� ������
#define TESTMODEOFF	0x98	//��������� ����� ������������
#define RESET		0xFE	//����������� ����� ������

//������ ��������� ����
#define BROADCAST	0xFF	//����� ����������������� �������� = 0
#define CUADR		0x64	//����� ����� ���������� (CUnit)
#define TADR0		0x64	//����� �������� ����� (������ �������)

void COMMANDS (int8_t func);

#endif /* ADC_LIB_H_ */