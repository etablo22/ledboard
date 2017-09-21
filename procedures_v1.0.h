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

//команды протокола
#define BRIGHT		0xA0	//команда установить значение яркости = 160
#define RXNtab		0xA9	//команда установить количество табло
#define RXTDATA		0xAA	//команда приема данных = 170
#define TXTDATA		0xAB	//команда отправки данных = 171
#define INDICMODE	0x96	//управление режимом индикации
#define TESTMODEON	0x97	//управление режимом работы
#define TESTMODEOFF	0x98	//выключить режим тестирования
#define RESET		0xFE	//программный сброс модуля

//адреса устройств сети
#define BROADCAST	0xFF	//адрес широковещательной передачи = 0
#define CUADR		0x64	//адрес блока управления (CUnit)
#define TADR0		0x64	//адрес нулевого табло (начало отсчета)

void COMMANDS (int8_t func);

#endif /* ADC_LIB_H_ */