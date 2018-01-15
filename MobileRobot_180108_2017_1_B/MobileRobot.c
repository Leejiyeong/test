#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Interface.h"
#include <math.h>
#include "Move.h"
#include "Motor.h"
#include "Sensor.h"

#define SW1            (~PINB&0x10)
#define SW2            (~PINB&0x20)
#define SW3            (~PINB&0x40)
volatile float speed=0, next_speed=0, acc=5, wspeed=0, next_wspeed=0, accW=4, acc_W=2;

#define WorkARM() StopMotion(9); LED_ON(0); while(PIND&0x10); LED_OFF(0);
volatile unsigned int sec=0;
void HolonomicW(float f_agl, float f_speed, float fw_speed);
void Read_Colors(unsigned char *color);
void non_Holonomic(float Fx, float Fy, float Fw);
void readsensor(int mode);
int Camera() ;
void Turn_and_Drive(double f_agl, int f_speed, int fw_speed, unsigned int mm, int dgree, unsigned int stop, unsigned int wstop);
int PSD_Wall_Follow(int end,int select, int ycm, int psd_end, int value, int speed);
void Sensor_Follow(int end, int speed);
void LSB(int delay);
void RSB(int delay);
int LineW(int W,int speed, int time);

int main(void)
{    
	int flag = 0, cnt=0, cnt2=0;
	unsigned char SENSOR=0;
    Interface_init(); //�������̽� �ʱ�ȭ
	LM629_HW_Reset();
	MCU_init();	   // MCU �ʱ�ȭ
	Motor_init();  // Motor ����̹� �ʱ�ȭ
	Sensor_init();
	TCCR1A=0x00;
	TCCR1B=0x05;
	TCNT1H=0xFF;
	TCNT1L=0x70;
	TIMSK=0x04;
	sei();
	while(1){
		if (SW1)
		{int mode=0;
			while(1) {
				mode = LineW(20, 200, 200);
				
				if((mode == 0) || (mode == 4)) {
					non_Holonomic(0, 0, 0);
					break;
				}
				else if(mode == 2) {
					Turn_and_Drive(0, 300, 50, 200, 90, 250, 85);
					LineW(20, 200, 200);
					Turn_and_Drive(0, 300, -50, 100, 90, 100, 85);
				}
				else if(mode == 3) {
					Turn_and_Drive(0, 300, -50, 100, 90, 100, 85);
					LineW(20, 200, 200);
					Turn_and_Drive(0, 300, 50, 200, 90, 250, 85);
				}
			}
			
			if((Camera_Cmd(1, 103) != 0) && mode == 4)
				flag = 1;
			else if((Camera_Cmd(2, 103) != 0) && mode == 4)
				flag = 2;
			else
				flag = 3;
			
			while(1) {
				if(psd_value[1] > 130)
					break;
				else
					non_Holonomic(0, -200, 0);
			}
			PSD_Wall_Follow(3, 1, 140, 0, 0, 200);
			if(((Camera_Cmd(flag, 102)>110) && (Camera_Cmd(flag, 102)<140)) || (mode == 0)) {
				Turn_and_Drive(0, 200, 60, 100, 180, 50, 170);
				RSB(500);
				return;
			}
			
			Turn_and_Drive(90, 200, 60, 200, 90, 150, 80);
			while(1)
			{
				if(psd_value[1] > 110) {
					Turn_and_Drive(0, 200, -40, 600, 90, 550, 80);
					if(((Camera_Cmd(flag, 102)>110) && (Camera_Cmd(flag, 102)<140)) || (mode == 0)) {
						Turn_and_Drive(180, 200, 0, 250, 0, 200, 0);
						Turn_and_Drive(0, 200, 70, 550, 180, 500, 170);
						RSB(500);
						return;
					}
					else{
						Turn_and_Drive(0, 0, 70, 0, 90, 0, 80);
						if(((Camera_Cmd(flag, 102)>110) && (Camera_Cmd(flag, 102)<140)) || (mode == 0)) {
							Turn_and_Drive(180, 200, 70, 400, 180, 350, 170);
							RSB(500);
							return;
						}
						else {
							non_Holonomic(0, 0, 0);
							break;
						}
					}
				}
				else
					non_Holonomic(200, 0, 0);
			}
		} 
		else if(SW2) 
		{
			readsensor(1);
		}
		else if(SW3)
		{
			while(1) {
				if(psd_value[1] > 200) {
					non_Holonomic(0, 0, 0);
					break;
				}
				else non_Holonomic(200,0,-(psd_value[8]-120));
				
				if(flag == 0 && (psd_value[1] > 150)) {
					if(Camera_Cmd(1, 103) != 0)
						flag = 1;
					else if(Camera_Cmd(2, 103) != 0)
						flag = 2;
				}
				display_char(0, 0, flag);
			}

			Turn_and_Drive(180, 200, 0, 100, 0, 50, 0);
			while(1) {
				if(Camera_Cmd(0, 1) == 125) break;
				else {
					non_Holonomic(0, 0, 10);
					cnt++;
				}
			}
			Turn_and_Drive(0, 200, 0, 100, 0, 50, 0);
			while(1) {
				if(cnt == 0) break;
				else {
					non_Holonomic(0, 0, -10);
					cnt--;
				}
			}

			Turn_and_Drive(180, 200, 0, 400, 0, 450, 0);
			Turn_and_Drive(0, 0, -70, 0, 92, 0, 85);
			non_Holonomic(0, 0, 0);

			if(Camera_Cmd(flag, 102) >= 125)
				while( !( (Camera_Cmd(flag, 102)>135) && (Camera_Cmd(flag,102)<160) ) ) non_Holonomic(0, 200, 0);
			else if(Camera_Cmd(flag, 102) < 125)
				while( !( (Camera_Cmd(flag, 102)>135) && (Camera_Cmd(flag,102)<160) ) ) non_Holonomic(0, -200, 0);
			
			while(!(READ_SENSOR()&&0x01)) non_Holonomic(200, 0, 0);
			
			sec=0;
			while(sec < 100) non_Holonomic(150, 0, 0);
			non_Holonomic(0, 0, 0);

			//- ��ٸ� Ÿ��
			int mode=0;
			while(1) {
				mode = LineW(20, 200, 100);
				
				if((mode == 0) || (mode == 4)) {
					non_Holonomic(0, 0, 0);
					break;
				}
				else if(mode == 2) {
					Turn_and_Drive(0, 300, 50, 300, 90, 250, 85);
					LineW(20, 200, 200);
					Turn_and_Drive(0, 300, -50, 100, 90, 100, 85);
				}
				else if(mode == 3) {
					Turn_and_Drive(0, 300, -50, 100, 90, 100, 85);
					LineW(20, 200, 200);
					Turn_and_Drive(0, 300, 50, 300, 90, 250, 85);
				}
			}
			
			if((Camera_Cmd(1, 103) != 0) && mode == 4)
				flag = 1;
			else if((Camera_Cmd(2, 103) != 0) && mode == 4)
				flag = 2;
			else
				flag = 3;
			
			display_char(0, 0, flag);
		}
	}  		
}

//������ Y������ ���� Ÿ�� �Լ� 
//int W�� �κ� �����ӵ� (-���� ������ �������� ���� Ÿ��, +�� ������ ���������� ����Ÿ��)
//int speed�� �κ� �����ӵ� 
//int time�� ���� ����� �����ϴ� �ð�

/*int LineW(int W,int speed, int time)
{
	int Lcount = 0;
	while(1)
	{
		if(READ_SENSOR() == 28)
			return 1;
		else if(READ_SENSOR() == 24)
			return 2;
		else if(READ_SENSOR() == 12)
			return 3;
		if(READ_SENSOR() == 0)
		{
			Lcount++;
			if(Lcount > time)
			return 0;
		}
		else Lcount=0;
		if(READ_SENSOR() != 8)
		{
			non_Holonomic(speed,W,0);
		}
		else if(READ_SENSOR() == 8)
		{
			non_Holonomic(speed,-W,0);
		}
	}
}*/



int LineW(int W,int speed, int time)
{
	if(W < 0)
		while(READ_SENSOR() != 8) non_Holonomic(0, -50, 0);
	else
		while(READ_SENSOR() != 8) non_Holonomic(0, 50, 0);

	sec=0;
	while(1)
	{
		display_char(0, 0, READ_SENSOR());
		if(READ_SENSOR() == 0)
		{
			if(sec > time)
				return 0;
		}
		else sec=0;
		if(READ_SENSOR() == 28)
			return 1;
		else if(READ_SENSOR() == 24) 
			return 2;
		else if(READ_SENSOR() == 12) 
			return 3;
		
		if(READ_SENSOR() != 8)
		{
			non_Holonomic(speed,W,0);
		}
		else if(READ_SENSOR() == 8)
		{
			non_Holonomic(speed,-W,0);
		}

		if((Camera_Cmd(0, 0) > 100) && (Camera_Cmd(0, 1) > 110) && (Camera_Cmd(0, 1) < 135)) {
			return 4;
		}
	}
}

//1. �������� (1: �ݼӼ���, 2: 
//2. �ӵ�
void Sensor_Follow(int end, int speed)
{
	unsigned char SENSOR=0;
	if(end == 1) {
		SENSOR = READ_SENSOR();
		if(SENSOR&0X01) return;
		if(READ_SENSOR() == 8) non_Holonomic(speed, 0, 0);
		else if(SENSOR&0X04) non_Holonomic(speed, 0, -10);
		else if(SENSOR&0X10) non_Holonomic(speed, 0, 10);
	}
}

//1. �̵����� 
//2. �̵��ӵ�
//ȸ���̵��ӵ� ���̳ʽ� �ϰ�� ����ȸ��
//�̵��Ÿ�
//ȸ����
//���� ����
//ȸ����������

void Turn_and_Drive(double f_agl, int f_speed, int fw_speed, unsigned int mm, int dgree, unsigned int stop, unsigned int wstop){
	double distance=0, distanceW=0;
	float S_distance=0, S_distanceW=0;
	unsigned char flg0=0, flg1=0;

	TCNT1H=0xFF; TCNT1L=0x70;
	sec=1;

	acc=5;	accW=3;
	next_speed=f_speed;
	next_wspeed=fw_speed;

	while(1){

		if(sec){
			sec=0;

			S_distance=speed*0.01;	//0.01 ���� �̵��Ÿ�
			S_distanceW=wspeed*0.01;	//0.01 ���� �̵��Ÿ� 

			f_agl=f_agl-S_distanceW;

			if(f_agl<0)f_agl+=360;
			else if(f_agl>=360)f_agl-=360;

			HolonomicW((int)(f_agl),speed,wspeed);

			distance+=S_distance;
			distanceW+=S_distanceW;
			if(distance>=stop && stop!=0)next_speed=100;
			if(fabs(distanceW)>=wstop && wstop!=0){
				next_wspeed=20;
				if(wspeed<=0)next_wspeed=-20;
			}

			if(distance>=mm || (distance*-1)>=mm){
				flg0=1;
				next_speed=0;
				speed=0;
			}
			if(fabs(distanceW)>=dgree){
				flg1=1;
				next_wspeed=0;
				wspeed=0;
			}
		}
		if(flg0 && flg1)break;
	}
}

//��Ÿ�� �Լ������
//1. ��������
//2. ���� ����(select ���� 1�ϰ�� 1������ ���ʺ�, 8�ϰ�� 8������ �����ʺ�)
//3. �����Ÿ� ������
//4. �������� ����(�������� ������� �����ϴ� ����)
//5. �������ǿ� ���� �Ÿ���(�������� 1)�պ���������� �պ������Ÿ�.
//								   2)����������ΰ�� ����������Ÿ�.
//6. �ӵ�

int PSD_Wall_Follow(int end,int select, int ycm, int psd_end, int value, int speed)  
{
	int oldpsd = 0;
	sec=0;
	
	while(1)
	{
		if(select == 1)	
		{
			if(psd_value[select] > 80 || sec < 200)  non_Holonomic(speed,0,(psd_value[select]-ycm));
		}
		else  non_Holonomic(speed,0,-(psd_value[select]-ycm));
		oldpsd = psd_value[select];

		if(end == 1)
		{
			if((psd_value[psd_end] > value) && ( sec > 100 ))
			{
				non_Holonomic(0,0,0);
				return 1;	
			}
		}
		else if(end == 2)
		{
			if( (  (oldpsd - psd_value[select])  > value ) && ( sec > 100 ) )
			{
				if(select == 1)
				{
					non_Holonomic(0,0,-50);
					_delay_ms(100);
				}
				non_Holonomic(0,0,0);
				return 2;
			} 
		}
		else
		{
			if((READ_SENSOR() != 0)&&(sec>100))
			return 3;
		}
	}
}

void LSB(int delay)
{
	int err1, err2 = 0;
	int cnt = 0;
	while(cnt < delay)
	{
		if((psd_value[3] - psd_value[1]) == 60)
			if((psd_value[4] - 250) == 0)
				if((psd_value[2] - 235) == 0)
			 		break;
		
		
		err1 = psd_value[4] - 240;
		err2 = psd_value[2] - 245;
		non_Holonomic(err1, err2, psd_value[1] - psd_value[3] + 60);
		cnt++;
		_delay_ms(1);
		display_char(0, 0, cnt);

	}
	non_Holonomic(0, 0, 0);

}

void RSB(int delay)
{
	lcd_clear_screen();
	int err1, err2 = 0;
	int cnt = 0;
	while(cnt < delay)
	{
		if(psd_value[6] - (psd_value[8] + 80) == 0)
			if((psd_value[5] - 210) == 0)
				if ((psd_value[7] - 190) == 0)
			 		break;
		
		
		err1 = psd_value[5] - 160;
		err2 = psd_value[7] - 190;
		if (psd_value[7] > 230)
			err2 = 50;
		if (psd_value[5] >= 240)
			err1 = 50;
		if (psd_value[6] == 250)
			err2+=60;
		non_Holonomic(err1, -err2, (psd_value[8] < 70 ? 0 : ((psd_value[6] - (psd_value[8] + 80)) / 2)));
		cnt++;
		_delay_ms(1);
		display_char(0, 0, cnt);

	}
	non_Holonomic(0, 0, 0);
	lcd_clear_screen();
}

ISR (TIMER1_OVF_vect)
{
//	TCNT1H=0xC7; TCNT1L=0xC0; //1��

//	TCNT1H=0xFA; TCNT1L=0x60; //0.1��

	TCNT1H=0xFF; TCNT1L=0x70; //0.01��

	++sec;

	if(next_speed>speed){
		speed+=acc;
		if(next_speed<=speed)speed=next_speed;
	}
	else if(next_speed<speed){
		speed-=acc;
		if(next_speed>=speed)speed=next_speed;
	}
	if(next_wspeed>wspeed){
		wspeed+=acc_W;
		if(next_wspeed<=wspeed)wspeed=next_wspeed;
	}
	else if(next_wspeed<wspeed){
		wspeed-=acc_W;
		if(next_wspeed>=wspeed)wspeed=next_wspeed;
	}
}

int Camera() 
{
	sec=0;
	while(1)
	{
		non_Holonomic(0, -100, 0);
		sec++;
		display_char(1, 0, Camera_Cmd(1, 102));
		display_char(0, 0, sec);
		if(sec > 10) {
			non_Holonomic(0, 0, 0);
			display_char(0, 4, -1);
			return -1;
		} else if(Camera_Cmd(1, 102) == 125) {
			non_Holonomic(0, 0, 0);
			display_char(0, 4, 1);
			return 1;
		} else if(Camera_Cmd(2, 102) == 125) {
			non_Holonomic(0, 0, 0);
			display_char(0, 4, 2);
			return 2;
		}
	}
}

int cameraconnect(void)
{
	int count = 0;
	putchar1(112);
	while(!rx1_flg)
	{
		count++;
		if (count > 500)
			return 0;
	}
	return 1;
}
void Read_Colors(unsigned char *color)
{
	int buff = 0, i, j;
	for(i = 0; i < 4; i++)
		color[i] = 0;
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 4; j++)
		{
			buff = Camera_Cmd(i + 3, (j * 2) + 1);
			if (buff == 0)
				break;
			else
			{
				if (buff < 70)
					color[0] = i + 1;
				else if(buff < 120)
					color[1] = i + 1;
				else if(buff < 170)
					color[2] = i + 1;
				else if(buff < 220)
					color[3] = i + 1;
			}
		}
	}

}
void readsensor(int mode)
{
	lcd_clear_screen();
	while(1) 
	{
		display_char(0, 6, psd_value[0]);
		display_char(0, 0, psd_value[1]);
		display_char(1, 0, psd_value[2]);
		display_char(2, 0, psd_value[3]);
		display_char(3, 0, psd_value[4]);
		display_char(3, 12, psd_value[5]);
		display_char(0, 12, psd_value[8]);
		display_char(1, 12, psd_value[7]);
		display_char(2, 12, psd_value[6]);
		display_char(2, 6, READ_SENSOR());
		if (mode == 1)
		{
			display_char(0, 17, Camera_Cmd(1, 102));
			display_char(1, 17, Camera_Cmd(1, 103));
			display_char(2, 17, Camera_Cmd(2, 102));
			display_char(3, 17, Camera_Cmd(2, 103));
		//	display_char(2, 17, Camera_Cmd(3, 102));
		//	display_char(0, 17, Camera_Cmd(0, 000));
		//	display_char(1, 17, Camera_Cmd(0, 001));
		//	display_char(2, 17, Camera_Cmd(0, 003));
		}
		else if(mode == 0){
			lcd_display_str(0, 17, "X");
			lcd_display_str(1, 17, "X");
			lcd_display_str(2, 17, "X"); 
		}
	}
}
void HolonomicW(float f_agl, float f_speed, float fw_speed){
	long Fx=0, Fy=0, Fw=0; //�ӵ�

	Fx = f_speed * cos(f_agl*0.017453);
	Fy = f_speed * sin(f_agl*0.017453);
	Fw=fw_speed;
	if(f_agl>=360||f_agl<0){
		Fx=0;
		Fy=0;
		Fw=f_speed;
		if(f_agl<0)Fw=-f_speed;
	}	

	non_Holonomic(Fx,Fy,Fw);
}

void non_Holonomic(float Fx, float Fy, float Fw){

	float V[3]={0,0,0};

	V[0]=( 0.056*Fx)+(0.033*Fy)+(0.14*Fw);
	V[1]=(-0.065*Fy)+(0.14*Fw);
	V[2]=(-0.056*Fx)+(0.033*Fy)+(0.14*Fw);

	SetVelocity(0, V[0]*65536);
	SetVelocity(1, V[1]*65536);
	SetVelocity(2, V[2]*65536);

	StartMotion();

}
