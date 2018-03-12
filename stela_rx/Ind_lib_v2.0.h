/*
 * Ind_lib.h
 *
 * Created: 08.07.2015 11:48:28
 *  Author: trainee

 v1.5 - � ISR (TIMER2_OVF_vect) ��� ������ ��� ������������ ��������� ��������� OE0 - OE3 �� ������ (����: case 1:{PORT_OE &=~ ((1<<OE0)|(1<<OE1));break;})
 - ��������� pwc ���������� ���������� ������������ ���������� �������� (��������� ����� OE ��������� ����� ����)
 v.1.6 - ��������� pwc ����� ���� ���������� � ������� ����� ������� (���������� ����� #ifndef pwc #endif)
 v.1.7 -  � ��������� display_10code � display_10code_point ��� LED_TYPE==3 �������� ������������ �������� �� �����������
 v.1.8 - ��� ���� ����� ���� �������� ��������� display_dynamic (������������� � display_send)
 v.1.9	- ������ ��������� DYNAMIC, ������ ���������� ��������� display_send(uint8_t dig_num)
		- �������� ��� ��������� LED_TYPE 5 ��� ����� im-49.3 � ����������������� ������������ ����,
			 ���������� ������������ ������������ ���� ���������� ���������� pwc
*/ 


#ifndef IND_LIB_H_
	#define IND_LIB_H_

	#include <avr/io.h>
	#include <avr/interrupt.h>

	//LED_TYPE - ��� ����� ���������:
	// 0 -	���������� �����;
	// 1 -	����� "�����������(prodv2011)"; 
	// 2 -	��������� �� ����� IM-51; 
	// 3 -	������������ ��������� �� ����� DLD_x;
	// 4 -	��� ����� IM-49 � �������� ������������� ���������, ������������ ��������� (�������� ��� ����� DLD);
	// 5 - ��� ����� im-49.3 � ����������������� ������������ ����
	#define LED_TYPE 5
	#define CONTROLLER_TYPE 1 // 0 - ���� ���������� �������� CU_v.4.3; 1 - �� �� ����� LED-����������� CM_1.3|CM_v.2.0

	#if (CONTROLLER_TYPE==0)
		#define PORT_595	PORTB
		#define DDR_595		DDRB
		#define PORT_OE		PORTB
		#define DDR_OE		DDRB
		#define DATA		PB2         //����� ������
		#define LATCH		PB4			//������� ��������
		
		#if ((LED_TYPE==0)||(LED_TYPE==4))		//����� IM-49 (���������� �����)
			#define CLK		PB3 		//�������� ��� ������ ������
			#define OE_595	PB5         // OE ���������� ��������
		#elif (LED_TYPE==1)		//����� "�����������(prodv2011)"
			#define CLK		PB5			//�������� ��� ������ ������
			#define OE_595	PB3			// OE ���������� ��������
		#endif
		
	#elif (CONTROLLER_TYPE==1)
		#define PORT_595 PORTD
		#define DDR_595  DDRD 		
		#define PORT_OE  PORTB
		#define DDR_OE DDRB
		#define DATA  PD5          //����� ������
		#define CLK   PD6 		  //�������� ��� ������ �������
		#define LATCH   PD7		  //������� ��������
		#define OE0 PB0         // OE ���������� ��������
		#define OE1 PB1         // OE ���������� ��������
		#define OE2 PB2         // OE ���������� ��������
		#define OE3 PB3         // OE ���������� ��������
	#endif

	#define MAXDIGNUMBER 13 // ������������ ��� ������� � �������� �������� �����

//��������� pwc ��� ����������������� ����������� ��������� 
// ���������� ������������ ���������� �������� (��� pwc=1 - ������ ������������ ���������, pwc=4 - �����������)
//����� IM-51 #if ((LED_TYPE==3)||(LED_TYPE==4)) //����� IM-49.3 #if (LED_TYPE==5)
//#define pwc	2			
	uint8_t pwc;
	void set_PWC(uint8_t PWM_index);
	
	uint8_t OE_count;
	uint8_t Dsort[4];
	void digit_sort(uint8_t dig3, uint8_t dig2, uint8_t dig1, uint8_t dig0); //���������� �������� �����
	
	uint8_t D[4];
	void display_send(uint8_t dig_num); //�������� ������ � �������� �������� �����

	void init595 ();  //������������� ������� ��� ���������� ��������
	void CLK_PULSE ();
	void LATCH_PULSE ();
	void send_data8 (uint8_t data);
	void send_data8_back (uint8_t data);
	//void send_data16 (uint16_t data);
	void display_7code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); //�� ���� ���� � 7���. ����, D0 ������ ������ ����� 
	//������� ������ ���� - 4 ���������� ����� - ����� �� ����� � 7���������� ���� ��� ���� ������ �����
	void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0); 
	void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask);
	void display_dnum(int32_t num);
	void initPWM(); //������������� ������ ���


#endif /* IND_LIB_H_ */