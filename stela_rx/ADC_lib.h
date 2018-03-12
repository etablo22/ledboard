/*
 * ADC_lib.h
 *
 * Created: 27.07.2015 17:02:03
 *  Author: trainee
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#ifndef ADC_LIB_H_
#define ADC_LIB_H_

#define ADC_LIGHT_IN   7    //номер канала для датчика освещенности
#define ADC_BUTTON_IN  5     //номер канала для адресного переключателя=делителя 

static uint16_t v_ADC=0, adc_counter=0;//переменная результата измерения АЦП

void adc_init (void);
void ADC_stop(void);
uint16_t ADC_result(uint8_t adc_input);

#endif /* ADC_LIB_H_ */