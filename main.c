/*
 * ESE519_Lab4_Pong_Starter.c
 *
 * Created: 9/21/2021 21:21:21 AM
 * Author : J. Ye
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include "lib\ST7735.h"
#include "lib\LCD_GFX.h"

void Initialize()
{
	lcd_init();
}

int main(void)
{
	Initialize();
		
    while (1) 
    {
		
    }
}