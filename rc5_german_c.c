#include "rc5_german.h"
 
#define RC5TIME 	1.778e-3		// 1.778msec Длительность одного бита
#define PULSE_MIN	(uchar)(XTAL / 512 * RC5TIME * 0.4 + 0.5)
#define PULSE_1_2	(uchar)(XTAL / 512 * RC5TIME * 0.8 + 0.5)
#define PULSE_MAX	(uchar)(XTAL / 512 * RC5TIME * 1.2 + 0.5)
 
uchar	rc5_bit;//значение бита
uchar	rc5_time;//Подсчет количества бит
uint	rc5_tmp;//Переменная для хранения временных значений
uint	rc5_data;//Переменная для результата

void initRC5()
{
	DDRrc5 &=~ _BV(xRC5);
	TCCR0 = 1 << CS02; //Деление тактовой частоты на 256
	TIMSK = 1 << TOIE0;	//Разрешаем прерывание по таймеру
}
 
//Обработчик прерывания по таймеру
ISR	(TIMER0_OVF_vect)
{
	uint tmp = rc5_tmp; // for faster access
	TCNT0 = -2; // 2 * 256 = 512 cycle
	if( ++rc5_time > PULSE_MAX ){	// count pulse time
		if( !(tmp & 0x4000) && tmp & 0x2000 )//Только если приняли 14 бит
			rc5_data = tmp;
		tmp = 0;
	}
	if( (rc5_bit ^ xRC5_IN) & 1<<xRC5 ){		// change detect
		rc5_bit = ~rc5_bit;				// 0x00 -> 0xFF -> 0x00
		if( rc5_time < PULSE_MIN )			// to short
			tmp = 0;
		if( !tmp || rc5_time > PULSE_1_2 ){		// start or long pulse time
			if( !(tmp & 0x4000) )			// not to many bits
				tmp <<= 1;				// shift
			if( !(rc5_bit & 1<<xRC5) )		// inverted bit
				tmp |= 1;				// insert new bit
			rc5_time = 0;				// count next pulse time
		}
	}
	rc5_tmp = tmp;
}
