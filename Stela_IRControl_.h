#ifndef STELA_IRCONTROL_H_
#define STELA_IRCONTROL_H_

#define GLASHATAY	0		//решение под цифры глашатай
#define MAXDIGNUMBER 13 // максимальный код символа в таблицах символов табло

//------   -------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
#if GLASHATAY
static int8_t ABCD_T[MAXDIGNUMBER] = {0x3F, 0x06, 0x6B, 0x4F, 0x56, 0x5D, 0x7D, 0x07, 0x7F, 0x5F, 0x40, 0x00, 0x63};
#else
static uint8_t ABCD_T[MAXDIGNUMBER] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};
#endif

#define SYMB_C			0x39 //символ С (аббривиатура команда)
#define SYMB_I			0x30 //символ С (аббривиатура команда)

#ifdef GLASHATAY
#define SYMB_n 	0x54 //символ С (аббривиатура команда)
#else
#define SYMB_n	0x64 //символ С (аббривиатура команда)
#endif

#define SYMB_F			0x71 //символ С (аббривиатура команда)

#ifdef GLASHATAY
#define SYMB_o			0x5c //символ С (аббривиатура команда)
#else
#define SYMB_o_GLASH	0x6c //символ С (аббривиатура команда)
#endif

#define SYMB_P			0x73 //символ С (аббривиатура команда)

#ifdef GLASHATAY
#define SYMB_i			0x10 //символ С (аббривиатура команда)
#else
#define SYMB_i_GLASH	0x20 //символ С (аббривиатура команда)
#endif

#define SYMB_E			0x79 //символ С (аббривиатура команда)
#define SYMB_L			0x38 //символ С (аббривиатура команда)

#endif /* STELA_IRCONTROL_H */