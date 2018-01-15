#include <util/delay.h>
#include <avr/interrupt.h>
#include "Interface.h"
#include "Move.h"
#include "Motor.h"

volatile unsigned char rx1_flg=0, rx1_buff=0;
volatile unsigned char CameraV1_buff[137],CameraV1_EN=0, CameraV1_flg=0, CameraV1_cnt=0;

// Function  : MCU 초기화.
// Parameter : 없음
// Return    : 없음
void MCU_init(void)
{
	// USART1 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART1 Receiver: On
	// USART1 Transmitter: On
	// USART1 Mode: Asynchronous
	// USART1 Baud Rate: 9600
	UCSR1A=0x00;
	UCSR1B=0x98;
	UCSR1C=0x06;
	//UBRR1H=0x00;
	//UBRR1L=0x5F;
	UBRR1H=0x00;
	UBRR1L=0x07;

	sei();
}

ISR(USART1_RX_vect)
{
	char data;
	
	data=UDR1;
	while(!(UCSR1A&0xB8)); //수신완료 및 에러 검출되지 않을시
	rx1_flg=1;              //수신완료 플레그 set
	rx1_buff=data;

	if(CameraV1_EN){
		//PORTB^=0X07;
		if(rx1_buff=='V'){
			CameraV1_cnt=0;
			//PORTB^=0X07;
		}
		CameraV1_buff[CameraV1_cnt++]=rx1_buff;
	}
}

char getchar1(void)  //uart 수신하기
{
	char data;
	
	data=rx1_buff;
	rx1_buff=0;
	cli();           //글로벌 인터럽트 disable
	rx1_flg=0;       //수신완료 플레그 reset
	sei();           //글로벌 인터럽트 enable          
	return data;     //수신데이터 반환
}

void putchar1(char data) //uart 송신화기
{
	while(!(UCSR1A&0x20));  //송신 준비 완료시
	UDR1=data;              //송신 버퍼에 데이터 쓰기
}



// Function  : LCD, LED, Buzzer, Switch를 사용하기 위해 초기화
// Parameter : 없음
// Return    : 없음
void Interface_init(void)
{
    // LCD / EX I/O Expander
	TWDR = 0xFF;
	TWBR = 0x48;
}

// Function  : I2C를 사용하여 LCD모듈로 한바이트 전송
// Parameter :
//          data - 전송할 한바이트 데이터
// Return    : 없음
void lcd_write_data(unsigned char data)
{
	// 시작
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
	// 준비상태 체크
    while(!(TWCR & (1 << TWINT)));
	
	// 주소 전송
    TWDR = SLA << 1;
	
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
	
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1 << TWINT)));
	
    // 정지
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

// Function  : 원하는 라인에 한줄의 문자열 출력
// Parameter :
//          line   - 출력할 라인번호(0~3)
//          string - 출력할 문자열(최대 20자)
// Return    : 없음
void lcd_display_str(unsigned char Y_line, unsigned char X_line,char *string) //lcd 스트링데이터 쓰기
{
    int i=X_line;

	if((Y_line<4) && (X_line<20)){
		lcd_write_data(0x1B);   lcd_write_data(0x4C);
		lcd_write_data(X_line); lcd_write_data(Y_line);
		_delay_us(100);
    	while(((i++)<20)&&*string){
        	lcd_write_data(*string++);
			_delay_us(40);
		}
    }
}

void write_lcd_data(unsigned char Y_line, unsigned char X_line, long data) //lcd데이터 쓰기
{
    if(Y_line<4 && X_line<20){
		lcd_write_data(0x1B);   lcd_write_data(0x4C);
		lcd_write_data(X_line); lcd_write_data(Y_line);
		_delay_us(100);
		lcd_write_data(data);
		_delay_us(40);
    }
}

void lcd_clear_screen(void){ lcd_write_data(0x1B); lcd_write_data(0x43); _delay_ms(20); } //lcd전체화면 클리어
void display_char(unsigned char line, unsigned char col, unsigned char data) ////lcd char 데이터 표시
{
    unsigned char val=0, val_buff=0;

	val=data;   

	val_buff=val/100;             
	write_lcd_data(line, col, val_buff+'0' );
	val=val-(100*val_buff);

	val_buff=val/10;
	write_lcd_data(line, 1+col, val_buff+'0' );
	val=val-(10*val_buff);

	write_lcd_data(line, 2+col, val+'0');
}


