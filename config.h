//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: 
//
//  Compiler....: 
//
//  Description.: глобальный конфигурационный файл
//
//  Data........:  
//
//***************************************************************************
#ifndef CONFIG_H
#define CONFIG_H

#ifndef F_CPU 
//   #define F_CPU 14745600UL
   #define F_CPU 16000000UL
#endif

#ifndef CPU_TYPE
	#define ATMEGA8		8
	#define	ATMEGA88	88
	#define ATMEGA328	328
	#define CPU_TYPE	ATMEGA328
#endif


#endif //CONFIG_H