#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <avr/io.h>

#define SLA             0x07

#define LED_DDR         DDRB
#define LED_PORT        PORTB
#define LED1            0
#define LED2            1
#define LED3            2

#define BZ_DDR          DDRB
#define BZ_PORT         PORTB
#define BZ              3

void MCU_init(void);
void Interface_init(void);
char getchar1(void);
void putchar1(char data);

// 해당 LED를 On
#define LED_ON(X)       (LED_PORT |=  (1 << (X)))
// 해당 LED를 Off
#define LED_OFF(X)      (LED_PORT &= ~(1 << (X)))

// Buzzer On
#define BZ_ON()         (BZ_PORT  |=  (1 << BZ));
// Buzzer Off
#define BZ_OFF()        (BZ_PORT  &= ~(1 << BZ));

void lcd_write_data(unsigned char data);
void lcd_display_str(unsigned char Y_line, unsigned char X_line,char *string);
void write_lcd_data(unsigned char Y_line, unsigned char X_line, long data);
void lcd_clear_screen(void);
void display_char(unsigned char line, unsigned char col, unsigned char data);

volatile extern unsigned char rx1_flg, rx1_buff;
volatile extern unsigned char CameraV1_buff[137], CameraV1_EN, CameraV1_flg, CameraV1_cnt;

//volatile extern long INT_Encoder[];

#endif		// __INTERFACE_H
