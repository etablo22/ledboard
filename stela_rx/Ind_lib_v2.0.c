/*
 * Ind_lib.c
 *
 * Created: 08.07.2015 12:00:41
 *  Author: trainee
 
 v1.5	- � ISR (TIMER2_OVF_vect) ��� ������ ��� ������������ ��������� ��������� OE0 - OE3 �� ������ (����: case 1:{PORT_OE &=~ ((1<<OE0)|(1<<OE1));break;})
		- ��������� pwc ���������� ���������� ������������ ���������� �������� (��������� ����� OE ��������� ����� ����) ��� ����� IM_51
 v.1.8	- � ���������� display_10code... ��� ���� ����, ����� prodv2011, ���������� ����������� ���� ��� �������� � �������� �����	
 v.1.9	- ������ ��������� DYNAMIC, ������ ���������� ��������� display_send(uint8_t dig_num)
		- �������� ��� ��������� LED_TYPE 5 ��� ����� im-49.3 � ����������������� ������������ ����, 
			���������� ������������ ������������ ���� ���������� ���������� pwc
 v.2.0	- ������� ��������� set_PWM ��� ��������� �������� pwc
		- ���������� ��������� display_send - �������� break; � ��������� switch-case
 */

#include "Ind_lib_v2.0.h"

#ifndef LED_TYPE
#define LED_TYPE 5 //0- ���������� ����� IM-49.2; 1 - ����� "�����������"; 2-��������� �� ����� IM-51; 3-������������ ���������; 4-IM-49
//0 - standard 7-segcode 4-digit indicator
//1 - "prodv2011"-table with non-standard coding
//2 - 7-segcode 4-digit indicator with backwards Dig3,Dig4
//3 - dynamic indication on 7-segcode 4-digit indicator
//4 - ��� ����� IM-49 � �������� ������������� ���������, ������������ ��������� (�������� ��� ����� DLD)
//5 - ��� ����� im-49.3 � ����������������� ������������ ����
#endif


//1 ���� ������

//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
uint8_t ABCD_TABLE [MAXDIGNUMBER]= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x40, 0x00, 0x63};

//������ ������� CBA_TABLE ������������ ������� send_data8_back();
//----------------------------------0-----1-----2-----3-----4-----5-----6-----7-----8------9--minus--null---^C--
//uint8_t CBA_TABLE [MAXDIGNUMBER]= {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6, 0x02, 0x00, 0xC6};


//--------------------------------0-------1------2------3------4------5------6------7------8-------9---minus--null----^C--
uint16_t Dig0 [MAXDIGNUMBER] = {0x1EC0,0x1800,0x1740,0x1D40,0x1980,0x0DC0,0x0FC0,0x1840,0x1FC0,0x1DC0,0x0100,0x0000,0x11C0};

//--------------------------------0-------1------2------3------4------5------6------7------8-------9---minus--null----^C--
uint16_t Dig1 [MAXDIGNUMBER] = {0x202F,0x0028,0x2036,0x203C,0x0039,0x201D,0x201F,0x2028,0x203F,0x203D,0x0010,0x0000,0x2031};

uint16_t Dig2 [MAXDIGNUMBER] = {0xF401,0xC000,0xB801,0xE801,0xCC00,0x6C01,0x7C01,0xC001,0xFC01,0xEC01,0x0800,0x0000,0x8C01};
   
uint16_t Dig3 [MAXDIGNUMBER] = {0x0378,0x0140,0x03B0,0x03E0,0x01C8,0x02E8,0x02F8,0x0340,0x03F8,0x03E8,0x0080,0x0000,0x0388};


	//������������� ������ ���
#if (CONTROLLER_TYPE==0) //CU_v.4.3
	void initPWM()
	{
		DDR_595 |= (1<<OE_595);  //������������ ����� ��� �� �����
		PORT_595 &=~ (1<<OE_595);

		//for Atmega_88
		// 	TCCR2=(1<<CS21);//|(1<<CS20); //
		// 	TCCR2=(1<<WGM21)|(1<<WGM20); //
		
		//for Atmega8A
		TCCR2=(1<<CS22)|(1<<CS20)|(1<<WGM21)|(1<<WGM20);
		TIMSK |= (1<<OCIE2)|(1<<TOIE2);
		
		OCR2=127;   //��������� �������� �������
	}

	#if (LED_TYPE==4) //�� ���� �����
		//��� - ��������� ��������� � 0
		ISR (TIMER2_COMP_vect)
		{	send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(0x00);
			send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(0x00);
			LATCH_PULSE();} //�� ��� �����
		//��� - ��������� ��������� � 1
		ISR (TIMER2_OVF_vect)
		{
			PORT_OE |= (1<<OE_595);
			display_send(OE_count);
			display_send(OE_count); //�� ��� �����
//			ADCSTART++;
		}
	#else
		ISR (TIMER2_COMP_vect){PORT_OE &=~ (1<<OE_595);}
		ISR (TIMER2_OVF_vect){PORT_OE |= (1<<OE_595);}
	#endif

#elif (CONTROLLER_TYPE==1) //Controll module: CM_1.3; CM_v.2.0
	void initPWM()
	{
		DDR_OE |= (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);  //������������ ������ ��� �� �����
		PORT_OE &=~ (1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3);

		//for Atmega_88
		// 	TCCR2=(1<<CS21);//|(1<<CS20); //
		// 	TCCR2=(1<<WGM21)|(1<<WGM20); //
		
		//for Atmega8A
		TCCR2=(1<<CS22)|(1<<CS20)|(1<<WGM21)|(1<<WGM20);
		TIMSK |= (1<<OCIE2)|(1<<TOIE2);
		
		OCR2=127;   //��������� �������� �������
	}

	#if ((LED_TYPE==3)||(LED_TYPE==4))
		//��������� ������ ��������� � 0
		ISR (TIMER2_COMP_vect){PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//��������� ������ ��������� � 1
		ISR (TIMER2_OVF_vect){
			switch (OE_count){
				case 1:{display_send(OE_count);PORT_OE |= ((1<<OE0));break;}
				case 2:{display_send(OE_count);PORT_OE |= ((1<<OE1));break;}
				case 3:{display_send(OE_count);PORT_OE |= ((1<<OE2));break;}
				case 4:{display_send(OE_count);PORT_OE |= ((1<<OE3));break;}
			}
			OE_count++;
			if (OE_count<1) OE_count=4;
			if (OE_count>4) OE_count=1;
		}
	#elif (LED_TYPE==2)
		//��������� ������ ��������� � 0
		ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//��������� ������ ��������� � 1
		ISR (TIMER2_OVF_vect){
			for (int jp=0;jp<pwc;jp++){			//pwc - ���������� ���������� ������������ ���������� ��������
				switch (OE_count+jp){
					case 1:{PORT_OE &=~ ((1<<OE0));break;}
					case 2:{PORT_OE &=~ ((1<<OE1));break;}
					case 3:{PORT_OE &=~ ((1<<OE2));break;}
					case 4:{PORT_OE &=~ ((1<<OE3));break;}
					case 5:{PORT_OE &=~ ((1<<OE0));break;}
					case 6:{PORT_OE &=~ ((1<<OE1));break;}
					case 7:{PORT_OE &=~ ((1<<OE2));break;}
					case 8:{PORT_OE &=~ ((1<<OE3));break;}
				}
			}
			OE_count++;
			if (OE_count<1) OE_count=4;
			if (OE_count>4) OE_count=1;
		}
	#elif (LED_TYPE==5)	//����������������� ���������: ����������� �������� �������� (� ������ pwc), ������� ��������������� ��� �� ����� OE
	//��������� ������ ��������� � 0
	ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
	//��������� ������ ��������� � 1
	ISR (TIMER2_OVF_vect){
		PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));
		OE_count++;
		if (OE_count<1) OE_count=4;
		if (OE_count>4) OE_count=1;
		display_send(OE_count);
	}
	#else	//
		//��������� ������ ��������� � 0
		ISR (TIMER2_COMP_vect){PORT_OE |= ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));}
		//��������� ������ ��������� � 1
		ISR (TIMER2_OVF_vect){
			PORT_OE &=~ ((1<<OE0)|(1<<OE1)|(1<<OE2)|(1<<OE3));
			OE_count++;
			if (OE_count==0) display_send(1);
		}
	#endif
#endif


//������������� ������� ��� ���������� ��������� ���������
void init595 ()
{	
	DDR_595 |= (1<<DATA)|(1<<CLK)|(1<<LATCH);  //������������ ������� �����
	PORT_595 &=~ (1<<DATA)|(1<<CLK)|(1<<LATCH);
}

//������� ������ CLK �������� HC595
void CLK_PULSE ()
{
	PORT_595|=_BV(CLK);	//������� �� SCL
	asm("nop");
	PORT_595&=~_BV(CLK);
	asm("nop");
}

//������� ������� LE �������� HC595
void LATCH_PULSE ()
{
	PORT_595|=_BV(LATCH);	//������� �� Latch clock
	asm("nop");
	PORT_595&=~_BV(LATCH);
	asm("nop");
}
//������ 1 ����� � ������� ������
void send_data8 (uint8_t data)
{
	unsigned char i;
	for (i=0;i<8;i++)
		{
			if ((data&0x80)!=0) PORT_595|=_BV(DATA);//���������� ������ �� PD0
			else PORT_595&=~_BV(DATA);
			CLK_PULSE();
			data=(data<<1);
		}
}
//������ 1 ����� � ������� ������ � �������� �������
void send_data8_back (uint8_t data)
{
	unsigned char i;
	for (i=0;i<8;i++)
	{
		if ((data&0x01)!=0) PORT_595|=_BV(DATA);//���������� ������ �� PD0
		else PORT_595&=~_BV(DATA);
		CLK_PULSE();
		data=(data>>1);
	}
}

//������ 2� ���� � ������� ������
void send_data16 (uint16_t data)
{
	unsigned char i;
	for (i=0;i<16;i++)
	{
		if ((data&0x8000)!=0) PORT_595|=_BV(DATA);//���������� ������ �� PD0
		else PORT_595&=~_BV(DATA);
		CLK_PULSE();
		data=(data<<1);
	}
}

//�� ���� ���� � 7���. ����, D0 ������ ������ ����� 
void display_7code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0)
{
	#if (LED_TYPE==0)
	{
		send_data8(D0);
		send_data8(D1);
		send_data8(D2);
		send_data8(D3);
	}
	#elif (LED_TYPE==1)
	{
	}
	#elif (LED_TYPE==2)
	{
		send_data8(D0);
		send_data8(D1);
		send_data8_back(D2);
		send_data8_back(D3);
	}
	//��� ������������ ���������
	#elif (LED_TYPE==3)
	{
		//����������� �������� ����� Table Digit LED-driver
		D[0]=D0;
		D[1]=D1;
		D[2]=D2;
		D[3]=D3;
	}
	//��� ����� IM-49 � �������� ������������� ��������� - ������������ ���������
	#elif (LED_TYPE==4)
	{
		send_data8_back(D0);
		send_data8_back(D1);
		send_data8_back(D2);
		send_data8_back(D3);
	}
	#endif	
	LATCH_PULSE();
}

//����� �� ����� � ���������� �������������
//�� ���� ���� ��� ���������� �����, D0 ������ ������ ����� 
void display_10code (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0)
{
	//����� �� ������� ����� "prodv2011" � ���� �����
	#if (LED_TYPE==1)
	{
		int16_t D01=0;
		int16_t D23=0;
		D01|=Dig0[D0]|Dig1[D1];
		D23|=Dig2[D2]|Dig3[D3];
		send_data16(D23);
		send_data16(D01);
		LATCH_PULSE();
	} 
	//����� �� ����� � ���� ABCD
	#else
	{
		//����������� ������������ ��������
		D[Dsort[0]]=ABCD_TABLE[D0];
		D[Dsort[1]]=ABCD_TABLE[D1];
		D[Dsort[2]]=ABCD_TABLE[D2];
		D[Dsort[3]]=ABCD_TABLE[D3];
	}
	#endif	   	
}

//����� �� ����� � ���������� ������������� - � ������ �����: point - ����� �����
//�� ���� ���� ��� ���������� �����, D0 ������ ������ �����
void display_10code_point (uint8_t D3, uint8_t D2, uint8_t D1, uint8_t D0, uint8_t pointmask)
{
	//����� �� ������� ����� "prodv2011" � ���� �����
	#if (LED_TYPE==1)
	{
		int16_t D01=0;
		int16_t D23=0;
		D01|=Dig0[D0]|Dig1[D1];
		D23|=Dig2[D2]|Dig3[D3];
		send_data16(D23);
		send_data16(D01);
		LATCH_PULSE();
	}
	//����� �� ����� � ���� ABCD
	#else
	{
		//����������� ������������ ��������
		D[Dsort[0]]=ABCD_TABLE[D0]|((pointmask<<7)&0x80);		//������ �������� ��� ������, �.�. � ������� ������� ��������������� ������ ����
		D[Dsort[1]]=ABCD_TABLE[D1]|((pointmask<<6)&0x80);
		D[Dsort[2]]=ABCD_TABLE[D2]|((pointmask<<5)&0x80);
		D[Dsort[3]]=ABCD_TABLE[D3]|((pointmask<<4)&0x80);
	}
	#endif
}


//����� �� ����� ����������� ����� �� 0 �� 9999
void display_dnum(int32_t num)
{
	uint8_t D_0=(num%10);
	uint8_t D_1=(num/10);
	uint8_t D_2=D_1/10;
	uint8_t D_3=D_2/10;
	D_1=D_1%10;
	D_2=D_2%10;
	display_10code (D_3,D_2,D_1,D_0);
//	display_10code ((uint8_t)num/1000,(uint8_t)num/100%10,(uint8_t)num/10%10,(uint8_t)num%10);
}

//��������� ������������ (�������� �������)
void display_send(uint8_t dig_num)
{
	//����� �� ���������� ����� � ���� ABCDEF
	#if (LED_TYPE==0)
	{
		send_data8(D[0]); send_data8(D[1]); send_data8(D[2]); send_data8(D[3]);
		LATCH_PULSE();
	}
	//����� �� ����� IM-51 � ���� ABCD+DCBA
	#elif (LED_TYPE==2)
	{
		send_data8(D[0]); send_data8(D[1]); 
		send_data8_back(D[2]); send_data8_back(D[3]);
		LATCH_PULSE();
	}
	#elif (LED_TYPE==3)
//		send_data8_back(ABCD_TABLE[D[dig_num-1]]|0x80);
		send_data8_back(D[dig_num-1]);
	#elif (LED_TYPE==4)
//			send_data8_back(D[dig_num-1]);
		{send_data8_back(D[0]);send_data8_back(D[1]);send_data8_back(D[2]);send_data8_back(D[3]);}
// 			switch (dig_num){
// 				case 1:{send_data8(0x00);send_data8(0x00);send_data8_back(D[2]);send_data8_back(D[3]);break;}
// 				case 2:{send_data8(0x00);send_data8_back(D[1]);send_data8_back(D[2]);send_data8(0x00);break;}
// 				case 3:{send_data8_back(D[0]);send_data8_back(D[1]);send_data8(0x00);send_data8(0x00);break;}
// 				case 4:{send_data8_back(D[0]);send_data8(0x00);send_data8(0x00);send_data8_back(D[3]);break;}
// 			}
	#elif (LED_TYPE==5)
		switch (pwc) 
		{
			case 1:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(0x00);send_data8(0x00);send_data8(D[3]);break;}
					case 2:{send_data8(0x00);send_data8(0x00);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(0x00);send_data8(D[1]);send_data8(0x00);send_data8(0x00);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(0x00);send_data8(0x00);break;}
				}
				break;
			case 2:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(0x00);send_data8(D[2]);send_data8(D[3]);break;}
					case 2:{send_data8(0x00);send_data8(D[1]);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(D[0]);send_data8(D[1]);send_data8(0x00);send_data8(0x00);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(0x00);send_data8(D[3]);break;}
				}
				break;
			case 3:
				switch (dig_num){
					case 1:{send_data8(0x00);send_data8(D[1]);send_data8(D[2]);send_data8(D[3]);break;}
					case 2:{send_data8(D[0]);send_data8(D[1]);send_data8(D[2]);send_data8(0x00);break;}
					case 3:{send_data8(D[0]);send_data8(D[1]);send_data8(0x00);send_data8(D[3]);break;}
					case 4:{send_data8(D[0]);send_data8(0x00);send_data8(D[2]);send_data8(D[3]);break;}
				}
				break;
			case 4:
				switch (dig_num){
					default: {send_data8(D[0]);send_data8(D[1]);send_data8(D[2]);send_data8(D[3]);break;}
				}
				break;
		}
	#endif
	LATCH_PULSE();
}

//���������� �������� �����
void digit_sort(uint8_t dig3, uint8_t dig2, uint8_t dig1, uint8_t dig0){
	Dsort[0]=dig0;
	Dsort[1]=dig1;
	Dsort[2]=dig2;
	Dsort[3]=dig3;
} 

//��������� �������� pwc - ���������� ������������ ���������� �������� (��� pwc=1 - ������ ������������ ���������, pwc=4 - �����������)
void set_PWC(uint8_t PWM_index){
	pwc=PWM_index;
}
