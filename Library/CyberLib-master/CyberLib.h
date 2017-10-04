//Библиотека разрабатывалась для контроллера CarDuino http://carmonitor.ru/ru/carduinonanoduo-p-120.html
//Все права на библиотеку CyberLib принадлежат компании CarMonitor.ru
//Обновления библиотеки скачивайте на сайте http://www.cyber-place.ru/showthread.php?p=3789#post3789

#ifndef CyberLib_H
#define CyberLib_H

// #include <inttypes.h>
// #include <avr/pgmspace.h>
// #include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif 

//****************Begin End*************
#define START labelbegin:
#define Start labelbegin:
#define END goto labelbegin;
#define End goto labelbegin;

#define NOP __asm__ __volatile__ ("nop\n\t")
#define nop __asm__ __volatile__ ("nop\n\t")
#define Nop __asm__ __volatile__ ("nop\n\t")

#define Pi 3.1415926535897932384626433832795
#define pi 3.1415926535897932384626433832795

//*************************************************Все на 328 процессоре**********************************
//********************************************************************************************************
//********************************************************************************************************
#if defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega88__) 
//****************INPUT PINS*************
#define D0_In DDRD &=B11111110
#define D1_In DDRD &=B11111101
#define D2_In DDRD &=B11111011
#define D3_In DDRD &=B11110111
#define D4_In DDRD &=B11101111
#define D5_In DDRD &=B11011111
#define D6_In DDRD &=B10111111
#define D7_In DDRD &=B01111111
 
#define D8_In DDRB &= B11111110
#define D9_In DDRB &= B11111101
#define D10_In DDRB &=B11111011
#define D11_In DDRB &=B11110111
#define D12_In DDRB &=B11101111
#define D13_In DDRB &=B11011111

#define D14_In DDRC &=B11111110
#define D15_In DDRC &=B11111101
#define D16_In DDRC &=B11111011
#define D17_In DDRC &=B11110111
#define D18_In DDRC &=B11101111
#define D19_In DDRC &=B11011111

//***************Output Pins*************
#define MotorA0 DDRD |=B00010000
#define MotorA1 DDRD |=B00100000
#define MotorB0 DDRD |=B01000000
#define MotorB1 DDRD |=B10000000

#define D0_Out DDRD |=B00000001
#define D1_Out DDRD |=B00000010
#define D2_Out DDRD |=B00000100
#define D3_Out DDRD |=B00001000
#define D4_Out DDRD |=B00010000
#define D5_Out DDRD |=B00100000
#define D6_Out DDRD |=B01000000
#define D7_Out DDRD |=B10000000

#define D8_Out DDRB |= B00000001
#define D9_Out DDRB |= B00000010
#define D10_Out DDRB |=B00000100
#define D11_Out DDRB |=B10001000
#define D12_Out DDRB |=B00010000
#define D13_Out DDRB |=B00100000

#define D14_Out DDRC |=B00000001
#define D15_Out DDRC |=B00000010
#define D16_Out DDRC |=B00000100
#define D17_Out DDRC |=B00001000
#define D18_Out DDRC |=B00010000
#define D19_Out DDRC |=B00100000

//***************Status High Pins*************
#define D0_High PORTD |=B00000001
#define D1_High PORTD |=B00000010
#define D2_High PORTD |=B00000100  
#define D3_High PORTD |=B00001000  
#define D4_High PORTD |=B00010000  
#define D5_High PORTD |=B00100000     
#define D6_High PORTD |=B01000000      
#define D7_High PORTD |=B10000000     

#define D8_High PORTB |=B00000001     
#define D9_High PORTB |=B00000010   
#define D10_High PORTB|=B00000100     
#define D11_High PORTB|=B00001000   
#define D12_High PORTB|=B00010000   
#define D13_High PORTB|=B00100000   
  
#define D14_High PORTC |=B00000001   
#define D15_High PORTC |=B00000010    
#define D16_High PORTC |=B00000100   
#define D17_High PORTC |=B00001000      
#define D18_High PORTC |=B00010000     
#define D19_High PORTC |=B00100000   

//***************Status Low Pins*************
#define D0_Low PORTD &= B11111110
#define D1_Low PORTD &= B11111101
#define D2_Low PORTD &= B11111011
#define D3_Low PORTD &= B11110111  
#define D4_Low PORTD &= B11101111  
#define D5_Low PORTD &= B11011111 
#define D6_Low PORTD &= B10111111 
#define D7_Low PORTD &= B01111111 

#define D8_Low PORTB &= B11111110 
#define D9_Low PORTB &= B11111101 
#define D10_Low PORTB &=B11111011 
#define D11_Low PORTB &=B11110111 
#define D12_Low PORTB &=B11101111 
#define D13_Low PORTB &=B11011111 

#define D14_Low PORTC &= B11111110  
#define D15_Low PORTC &= B11111101 
#define D16_Low PORTC &= B11111011 
#define D17_Low PORTC &= B11110111
#define D18_Low PORTC &= B11101111
#define D19_Low PORTC &= B11011111 

//**************Invert Status Pins*************
#define D0_Inv PORTD ^=B00000001
#define D1_Inv PORTD ^=B00000010
#define D2_Inv PORTD ^=B00000100
#define D3_Inv PORTD ^=B00001000
#define D4_Inv PORTD ^=B00010000
#define D5_Inv PORTD ^=B00100000
#define D6_Inv PORTD ^=B01000000
#define D7_Inv PORTD ^=B10000000

#define D8_Inv PORTB ^=B00000001
#define D9_Inv PORTB ^=B00000010
#define D10_Inv PORTB^=B00000100
#define D11_Inv PORTB^=B00001000
#define D12_Inv PORTB^=B00010000
#define D13_Inv PORTB^=B00100000

#define D14_Inv PORTC ^=B00000001
#define D15_Inv PORTC ^=B00000010
#define D16_Inv PORTC ^=B00000100
#define D17_Inv PORTC ^=B00001000
#define D18_Inv PORTC ^=B00010000
#define D19_Inv PORTC ^=B00100000

//**************READ Status Pins*************
#define D0_Read (PIND & B00000001)
#define D1_Read ((PIND & B00000010)>>1)
#define D2_Read ((PIND & B00000100)>>2)
#define D3_Read ((PIND & B00001000)>>3)
#define D4_Read ((PIND & B00010000)>>4)
#define D5_Read ((PIND & B00100000)>>5)
#define D6_Read ((PIND & B01000000)>>6)
#define D7_Read ((PIND & B10000000)>>7)

#define D8_Read (PINB & B00000001)
#define D9_Read ((PINB & B00000010)>>1)
#define D10_Read ((PINB & B00000100)>>2)
#define D11_Read ((PINB & B00001000)>>3)
#define D12_Read ((PINB & B00010000)>>4)
#define D13_Read ((PINB & B00100000)>>5)

#define D14_Read (PINC & B00000001)
#define D15_Read ((PINC & B00000010)>>1)
#define D16_Read ((PINC & B00000100)>>2)
#define D17_Read ((PINC & B00001000)>>3)
#define D18_Read ((PINC & B00010000)>>4)
#define D19_Read ((PINC & B00100000)>>5)

//**************Analog READ*******************
#define A0_Read (AnRead(B01000000))
#define A1_Read (AnRead(B01000001))
#define A2_Read (AnRead(B01000010))
#define A3_Read (AnRead(B01000011))
#define A4_Read (AnRead(B01000100))
#define A5_Read (AnRead(B01000101))
#define A6_Read (AnRead(B01000110))
#define A7_Read (AnRead(B01000111))	
    uint16_t AnRead(uint8_t An_pin);
	
//**************Small UART******************************
	void UART_Init(uint32_t UART_BAUD_RATE);
	void UART_SendByte(uint8_t b);
	bool UART_ReadByte(uint8_t & data);
	void UART_SendArray(uint8_t *buffer, uint16_t bufferSize);
	
//************SPI********************************
	void StartSPI(uint8_t SPI_Mode, uint8_t SPI_Div, uint8_t SPI_Change_Shift );
	void StopSPI(void);
	uint8_t ReadSPI(void);
	void SendSPI(uint8_t SPI_data) ;
	
//**************Converter*******************	
	uint8_t CharToDec(uint8_t digit);	
	uint8_t DecToChar(uint8_t number);
	
//******************EEPROM*******************************
	void WriteEEPROM_Byte(uint8_t addr, uint8_t data);  //сохранить в EEPROM тип Byte
	void WriteEEPROM_Word(uint16_t addr, uint16_t data);  //сохранить в EEPROM тип Word
	void WriteEEPROM_Long(uint8_t addr, uint32_t data);  //сохранить в EEPROM тип Long
	uint8_t ReadEEPROM_Byte(uint8_t addr);  // считываем значение Byte из EEPROM
	uint16_t ReadEEPROM_Word(uint16_t addr);  // считываем значение Word из EEPROM
	uint32_t ReadEEPROM_Long(uint8_t addr);  // считываем значение Long из EEPROM

//**************Timer1*************************
	extern void (*func)();	
	void StartTimer1(void (*isr)(), uint32_t set_us);
	void StopTimer1(void);
	void ResumeTimer1(void);
	void RestartTimer1(void);
	
//**************Find****************************
uint16_t find_similar(uint16_t *buf, uint8_t size_buff, uint8_t range);

//**************Beep**********************
void beep(uint16_t dur, uint16_t frq);

//**************Soft Reset**********************
 void reset();
 
//**************Delay*************************	
	void delay_us(uint16_t tic_us);	
	void delay_ms(uint16_t tic_ms);	
// ******************************************MEGA**********************************************
//*********************************************************************************************
//*********************************************************************************************
#elif  defined (__AVR_ATmega2560__) || defined (__AVR_ATmega2561__)
//****************INPUT PINS*************

// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX

#define D2_In DDRE &=B11101111	// PE 4 PWM2
#define D3_In DDRE &=B11011111	// PE 5 PWM3
#define D4_In DDRG &=B11011111	// PG 5 PWM4
#define D5_In DDRE &=B11110111	// PE 3 PWM5
#define D6_In DDRH &=B11110111	// PH 3 PWM6
#define D7_In DDRH &=B11101111	// PH 4 PWM7
#define D8_In DDRH &=B11011111	// PH 5 PWM8
#define D9_In DDRH &=B10111111	// PH 6 PWM9
#define D10_In DDRB &=B11101111	// PB 4 PWM10
#define D11_In DDRB &=B11011111	// PB 5 PWM11
#define D12_In DDRB &=B10111111	// PB 6 PWM12
#define D13_In DDRB &=B01111111	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL

#define D22_In DDRA &=B11111110	// PA 0 D22
#define D23_In DDRA &=B11111101	// PA 1 D23
#define D24_In DDRA &=B11111011	// PA 2 D24
#define D25_In DDRA &=B11110111	// PA 3 D25
#define D26_In DDRA &=B11101111	// PA 4 D26
#define D27_In DDRA &=B11011111	// PA 5 D27
#define D28_In DDRA &=B10111111	// PA 6 D28
#define D29_In DDRA &=B01111111	// PA 7 D29
#define D30_In DDRC &=B01111111	// PC 7 D30
#define D31_In DDRC &=B10111111	// PC 6 D31
#define D32_In DDRC &=B11011111	// PC 5 D32
#define D33_In DDRC &=B11101111	// PC 4 D33
#define D34_In DDRC &=B11110111	// PC 3 D34
#define D35_In DDRC &=B11111011	// PC 2 D35
#define D36_In DDRC &=B11111101	// PC 1 D36
#define D37_In DDRC &=B11111110	// PC 0 D37
#define D38_In DDRD &=B01111111	// PD 7 D38
#define D39_In DDRG &=B11111011	// PG 2 D39
#define D40_In DDRG &=B11111101	// PG 1 D40
#define D41_In DDRG &=B11111110	// PG 0 D41
#define D42_In DDRL &=B01111111	// PL 7 D42
#define D43_In DDRL &=B10111111	// PL 6 D43
#define D44_In DDRL &=B11011111	// PL 5 D44
#define D45_In DDRL &=B11101111	// PL 4 D45
#define D46_In DDRL &=B11110111	// PL 3 D46
#define D47_In DDRL &=B11111011	// PL 2 D47
#define D48_In DDRL &=B11111101	// PL 1 D48
#define D49_In DDRL &=B11111110	// PL 0 D49
#define D50_In DDRB &=B11110111	// PB 3 SPI_MISO
#define D51_In DDRB &=B11111011	// PB 2 SPI_MOSI
#define D52_In DDRB &=B11111101	// PB 1 SPI_SCK
#define D53_In DDRB &=B11111110	// PB 0 SPI_SS


//***************Output Pins*************
// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX
#define D2_Out DDRE |=B00010000		// PE 4 PWM2
#define D3_Out DDRE |=B00100000		// PE 5 PWM3
#define D4_Out DDRG |=B00100000		// PG 5 PWM4
#define D5_Out DDRE |=B00001000		// PE 3 PWM5
#define D6_Out DDRH |=B00001000		// PH 3 PWM6
#define D7_Out DDRH |=B00010000		// PH 4 PWM7
#define D8_Out DDRH |=B00100000		// PH 5 PWM8
#define D9_Out DDRH |=B01000000		// PH 6 PWM9
#define D10_Out DDRB |=B00010000	// PB 4 PWM10
#define D11_Out DDRB |=B00100000	// PB 5 PWM11
#define D12_Out DDRB |=B01000000	// PB 6 PWM12
#define D13_Out DDRB |=B10000000	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL


#define D22_Out DDRA |=B00000001	// PA 0 D22
#define D23_Out DDRA |=B00000010	// PA 1 D23
#define D24_Out DDRA |=B00000100	// PA 2 D24
#define D25_Out DDRA |=B00001000	// PA 3 D25
#define D26_Out DDRA |=B00010000	// PA 4 D26
#define D27_Out DDRA |=B00100000	// PA 5 D27
#define D28_Out DDRA |=B01000000	// PA 6 D28
#define D29_Out DDRA |=B10000000	// PA 7 D29
#define D30_Out DDRC |=B10000000	// PC 7 D30
#define D31_Out DDRC |=B01000000	// PC 6 D31
#define D32_Out DDRC |=B00100000	// PC 5 D32
#define D33_Out DDRC |=B00010000	// PC 4 D33
#define D34_Out DDRC |=B00001000	// PC 3 D34
#define D35_Out DDRC |=B00000100	// PC 2 D35
#define D36_Out DDRC |=B00000010	// PC 1 D36
#define D37_Out DDRC |=B00000001	// PC 0 D37
#define D38_Out DDRD |=B10000000	// PD 7 D38
#define D39_Out DDRG |=B00000100	// PG 2 D39
#define D40_Out DDRG |=B00000010	// PG 1 D40
#define D41_Out DDRG |=B00000001	// PG 0 D41
#define D42_Out DDRL |=B10000000	// PL 7 D42
#define D43_Out DDRL |=B01000000	// PL 6 D43
#define D44_Out DDRL |=B00100000	// PL 5 D44
#define D45_Out DDRL |=B00010000	// PL 4 D45
#define D46_Out DDRL |=B00001000	// PL 3 D46
#define D47_Out DDRL |=B00000100	// PL 2 D47
#define D48_Out DDRL |=B00000010	// PL 1 D48
#define D49_Out DDRL |=B00000001	// PL 0 D49
#define D50_Out DDRB |=B00001000	// PB 3 SPI_MISO
#define D51_Out DDRB |=B00000100	// PB 2 SPI_MOSI
#define D52_Out DDRB |=B00000010	// PB 1 SPI_SCK
#define D53_Out DDRB |=B00000001	// PB 0 SPI_SS


//***************Status High Pins*************
// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX
#define D2_High PORTE |=B00010000	// PE 4 PWM2
#define D3_High PORTE |=B00100000	// PE 5 PWM3
#define D4_High PORTG |=B00100000	// PG 5 PWM4
#define D5_High PORTE |=B00001000	// PE 3 PWM5
#define D6_High PORTH |=B00001000	// PH 3 PWM6
#define D7_High PORTH |=B00010000	// PH 4 PWM7
#define D8_High PORTH |=B00100000	// PH 5 PWM8
#define D9_High PORTH |=B01000000	// PH 6 PWM9
#define D10_High PORTB |=B00010000	// PB 4 PWM10
#define D11_High PORTB |=B00100000	// PB 5 PWM11
#define D12_High PORTB |=B01000000	// PB 6 PWM12
#define D13_High PORTB |=B10000000	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL

#define D22_High PORTA |=B00000001	// PA 0 D22
#define D23_High PORTA |=B00000010	// PA 1 D23
#define D24_High PORTA |=B00000100	// PA 2 D24
#define D25_High PORTA |=B00001000	// PA 3 D25
#define D26_High PORTA |=B00010000	// PA 4 D26
#define D27_High PORTA |=B00100000	// PA 5 D27
#define D28_High PORTA |=B01000000	// PA 6 D28
#define D29_High PORTA |=B10000000	// PA 7 D29
#define D30_High PORTC |=B10000000	// PC 7 D30
#define D31_High PORTC |=B01000000	// PC 6 D31
#define D32_High PORTC |=B00100000	// PC 5 D32
#define D33_High PORTC |=B00010000	// PC 4 D33
#define D34_High PORTC |=B00001000	// PC 3 D34
#define D35_High PORTC |=B00000100	// PC 2 D35
#define D36_High PORTC |=B00000010	// PC 1 D36
#define D37_High PORTC |=B00000001	// PC 0 D37
#define D38_High PORTD |=B10000000	// PD 7 D38
#define D39_High PORTG |=B00000100	// PG 2 D39
#define D40_High PORTG |=B00000010	// PG 1 D40
#define D41_High PORTG |=B00000001	// PG 0 D41
#define D42_High PORTL |=B10000000	// PL 7 D42
#define D43_High PORTL |=B01000000	// PL 6 D43
#define D44_High PORTL |=B00100000	// PL 5 D44
#define D45_High PORTL |=B00010000	// PL 4 D45
#define D46_High PORTL |=B00001000	// PL 3 D46
#define D47_High PORTL |=B00000100	// PL 2 D47
#define D48_High PORTL |=B00000010	// PL 1 D48
#define D49_High PORTL |=B00000001	// PL 0 D49
#define D50_High PORTB |=B00001000	// PB 3 SPI_MISO
#define D51_High PORTB |=B00000100	// PB 2 SPI_MOSI
#define D52_High PORTB |=B00000010	// PB 1 SPI_SCK
#define D53_High PORTB |=B00000001	// PB 0 SPI_SS


//***************Status Low Pins*************
// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX

#define D2_Low PORTE &=B11101111	// PE 4 PWM2
#define D3_Low PORTE &=B11011111	// PE 5 PWM3
#define D4_Low PORTG &=B11011111	// PG 5 PWM4
#define D5_Low PORTE &=B11110111	// PE 3 PWM5
#define D6_Low PORTH &=B11110111	// PH 3 PWM6
#define D7_Low PORTH &=B11101111	// PH 4 PWM7
#define D8_Low PORTH &=B11011111	// PH 5 PWM8
#define D9_Low PORTH &=B10111111	// PH 6 PWM9
#define D10_Low PORTB &=B11101111	// PB 4 PWM10
#define D11_Low PORTB &=B11011111	// PB 5 PWM11
#define D12_Low PORTB &=B10111111	// PB 6 PWM12
#define D13_Low PORTB &=B01111111	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL

#define D22_Low PORTA &=B11111110	// PA 0 D22
#define D23_Low PORTA &=B11111101	// PA 1 D23
#define D24_Low PORTA &=B11111011	// PA 2 D24
#define D25_Low PORTA &=B11110111	// PA 3 D25
#define D26_Low PORTA &=B11101111	// PA 4 D26
#define D27_Low PORTA &=B11011111	// PA 5 D27
#define D28_Low PORTA &=B10111111	// PA 6 D28
#define D29_Low PORTA &=B01111111	// PA 7 D29
#define D30_Low PORTC &=B01111111	// PC 7 D30
#define D31_Low PORTC &=B10111111	// PC 6 D31
#define D32_Low PORTC &=B11011111	// PC 5 D32
#define D33_Low PORTC &=B11101111	// PC 4 D33
#define D34_Low PORTC &=B11110111	// PC 3 D34
#define D35_Low PORTC &=B11111011	// PC 2 D35
#define D36_Low PORTC &=B11111101	// PC 1 D36
#define D37_Low PORTC &=B11111110	// PC 0 D37
#define D38_Low PORTD &=B01111111	// PD 7 D38
#define D39_Low PORTG &=B11111011	// PG 2 D39
#define D40_Low PORTG &=B11111101	// PG 1 D40
#define D41_Low PORTG &=B11111110	// PG 0 D41
#define D42_Low PORTL &=B01111111	// PL 7 D42
#define D43_Low PORTL &=B10111111	// PL 6 D43
#define D44_Low PORTL &=B11011111	// PL 5 D44
#define D45_Low PORTL &=B11101111	// PL 4 D45
#define D46_Low PORTL &=B11110111	// PL 3 D46
#define D47_Low PORTL &=B11111011	// PL 2 D47
#define D48_Low PORTL &=B11111101	// PL 1 D48
#define D49_Low PORTL &=B11111110	// PL 0 D49
#define D50_Low PORTB &=B11110111	// PB 3 SPI_MISO
#define D51_Low PORTB &=B11111011	// PB 2 SPI_MOSI
#define D52_Low PORTB &=B11111101	// PB 1 SPI_SCK
#define D53_Low PORTB &=B11111110	// PB 0 SPI_SS

//**************Invert Status Pins*************
// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX
#define D2_Inv PORTE ^=B00010000	// PE 4 PWM2
#define D3_Inv PORTE ^=B00100000	// PE 5 PWM3
#define D4_Inv PORTG ^=B00100000	// PG 5 PWM4
#define D5_Inv PORTE ^=B00001000	// PE 3 PWM5
#define D6_Inv PORTH ^=B00001000	// PH 3 PWM6
#define D7_Inv PORTH ^=B00010000	// PH 4 PWM7
#define D8_Inv PORTH ^=B00100000	// PH 5 PWM8
#define D9_Inv PORTH ^=B01000000	// PH 6 PWM9
#define D10_Inv PORTB ^=B00010000	// PB 4 PWM10
#define D11_Inv PORTB ^=B00100000	// PB 5 PWM11
#define D12_Inv PORTB ^=B01000000	// PB 6 PWM12
#define D13_Inv PORTB ^=B10000000	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL

#define D22_Inv PORTA ^=B00000001	// PA 0 D22
#define D23_Inv PORTA ^=B00000010	// PA 1 D23
#define D24_Inv PORTA ^=B00000100	// PA 2 D24
#define D25_Inv PORTA ^=B00001000	// PA 3 D25
#define D26_Inv PORTA ^=B00010000	// PA 4 D26
#define D27_Inv PORTA ^=B00100000	// PA 5 D27
#define D28_Inv PORTA ^=B01000000	// PA 6 D28
#define D29_Inv PORTA ^=B10000000	// PA 7 D29
#define D30_Inv PORTC ^=B10000000	// PC 7 D30
#define D31_Inv PORTC ^=B01000000	// PC 6 D31
#define D32_Inv PORTC ^=B00100000	// PC 5 D32
#define D33_Inv PORTC ^=B00010000	// PC 4 D33
#define D34_Inv PORTC ^=B00001000	// PC 3 D34
#define D35_Inv PORTC ^=B00000100	// PC 2 D35
#define D36_Inv PORTC ^=B00000010	// PC 1 D36
#define D37_Inv PORTC ^=B00000001	// PC 0 D37
#define D38_Inv PORTD ^=B10000000	// PD 7 D38
#define D39_Inv PORTG ^=B00000100	// PG 2 D39
#define D40_Inv PORTG ^=B00000010	// PG 1 D40
#define D41_Inv PORTG ^=B00000001	// PG 0 D41
#define D42_Inv PORTL ^=B10000000	// PL 7 D42
#define D43_Inv PORTL ^=B01000000	// PL 6 D43
#define D44_Inv PORTL ^=B00100000	// PL 5 D44
#define D45_Inv PORTL ^=B00010000	// PL 4 D45
#define D46_Inv PORTL ^=B00001000	// PL 3 D46
#define D47_Inv PORTL ^=B00000100	// PL 2 D47
#define D48_Inv PORTL ^=B00000010	// PL 1 D48
#define D49_Inv PORTL ^=B00000001	// PL 0 D49
#define D50_Inv PORTB ^=B00001000	// PB 3 SPI_MISO
#define D51_Inv PORTB ^=B00000100	// PB 2 SPI_MOSI
#define D52_Inv PORTB ^=B00000010	// PB 1 SPI_SCK
#define D53_Inv PORTB ^=B00000001	// PB 0 SPI_SS


//**************READ Status Pins*************
// PE 0 ** 0 ** USART0_RX
// PE 1 ** 1 ** USART0_TX
#define D2_Read ((PINE & B00010000)>>4)		// PE 4 PWM2
#define D3_Read ((PINE & B00100000)>>5)		// PE 5 PWM3
#define D4_Read ((PING & B00100000)>>5)		// PG 5 PWM4
#define D5_Read ((PINE & B00001000)>>3)		// PE 3 PWM5
#define D6_Read ((PINH & B00001000)>>3)		// PH 3 PWM6
#define D7_Read ((PINH & B00010000)>>4)		// PH 4 PWM7
#define D8_Read ((PINH & B00100000)>>5)		// PH 5 PWM8
#define D9_Read ((PINH & B01000000)>>6)		// PH 6 PWM9
#define D10_Read ((PINB & B00010000)>>4)	// PB 4 PWM10
#define D11_Read ((PINB & B00100000)>>5)	// PB 5 PWM11
#define D12_Read ((PINB & B01000000)>>6)	// PB 6 PWM12
#define D13_Read ((PINB & B10000000)>>7)	// PB 7 PWM13

// PJ 1 ** 14 ** USART3_TX
// PJ 0 ** 15 ** USART3_RX
// PH 1 ** 16 ** USART2_TX
// PH 0 ** 17 ** USART2_RX
// PD 3 ** 18 ** USART1_TX
// PD 2 ** 19 ** USART1_RX
// PD 1 ** 20 ** I2C_SDA
// PD 0 ** 21 ** I2C_SCL

#define D22_Read (PINA & B00000001)		// PA 0 D22
#define D23_Read ((PINA & B00000010)>>1)	// PA 1 D23
#define D24_Read ((PINA & B00000100)>>2)	// PA 2 D24
#define D25_Read ((PINA & B00001000)>>3)	// PA 3 D25
#define D26_Read ((PINA & B00010000)>>4)	// PA 4 D26
#define D27_Read ((PINA & B00100000)>>5)	// PA 5 D27
#define D28_Read ((PINA & B01000000)>>6)	// PA 6 D28
#define D29_Read ((PINA & B10000000)>>7)	// PA 7 D29
#define D30_Read ((PINC & B10000000)>>7)	// PC 7 D30
#define D31_Read ((PINC & B01000000)>>6)	// PC 6 D31
#define D32_Read ((PINC & B00100000)>>5)	// PC 5 D32
#define D33_Read ((PINC & B00010000)>>4)	// PC 4 D33
#define D34_Read ((PINC & B00001000)>>3)	// PC 3 D34
#define D35_Read ((PINC & B00000100)>>2)	// PC 2 D35
#define D36_Read ((PINC & B00000010)>>1)	// PC 1 D36
#define D37_Read (PINC & B00000001)		// PC 0 D37
#define D38_Read ((PIND & B10000000)>>7)	// PD 7 D38
#define D39_Read ((PING & B00000100)>>2)	// PG 2 D39
#define D40_Read ((PING & B00000010)>>1)	// PG 1 D40
#define D41_Read (PING & B00000001)		// PG 0 D41
#define D42_Read ((PINL & B10000000)>>7)	// PL 7 D42
#define D43_Read ((PINL & B01000000)>>6)	// PL 6 D43
#define D44_Read ((PINL & B00100000)>>5)	// PL 5 D44
#define D45_Read ((PINL & B00010000)>>4)	// PL 4 D45
#define D46_Read ((PINL & B00001000)>>3)	// PL 3 D46
#define D47_Read ((PINL & B00000100)>>2)	// PL 2 D47
#define D48_Read ((PINL & B00000010)>>1)	// PL 1 D48
#define D49_Read (PINL & B00000001)		// PL 0 D49
#define D50_Read ((PINB & B00001000)>>3)	// PB 3 SPI_MISO
#define D51_Read ((PINB & B00000100)>>2)	// PB 2 SPI_MOSI
#define D52_Read ((PINB & B00000010)>>1)	// PB 1 SPI_SCK
#define D53_Read (PINB & B00000001)		// PB 0 SPI_SS
//**************Delay*************************	
	void delay_us(uint16_t tic_us);	
	void delay_ms(uint16_t tic_ms);	

//******************EEPROM*******************************
	void WriteEEPROM_Byte(uint8_t addr, uint8_t data);  //сохранить в EEPROM тип Byte
	void WriteEEPROM_Word(uint16_t addr, uint16_t data);  //сохранить в EEPROM тип Word
	void WriteEEPROM_Long(uint8_t addr, uint32_t data);  //сохранить в EEPROM тип Long
	uint8_t ReadEEPROM_Byte(uint8_t addr);  // считываем значение Byte из EEPROM
	uint16_t ReadEEPROM_Word(uint16_t addr);  // считываем значение Word из EEPROM
	uint32_t ReadEEPROM_Long(uint8_t addr);  // считываем значение Long из EEPROM

//*************************************************Leonardo и все на 32U4**********************************
//********************************************************************************************************
//********************************************************************************************************
#elif defined (__AVR_ATmega32U4__)
//****************INPUT PINS*************
#define D0_In DDRD &=B11111011
#define D1_In DDRD &=B11110111
#define D2_In DDRD &=B11111101
#define D3_In DDRD &=B11111110
#define D4_In DDRD &=B11101111
#define D5_In DDRC &=B10111111
#define D6_In DDRD &=B01111111
#define D7_In DDRE &=B10111111
 
#define D8_In DDRB  &=B11101111
#define D9_In DDRB  &=B11011111
#define D10_In DDRB &=B10111111
#define D11_In DDRB &=B01111111
#define D12_In DDRD &=B10111111
#define D13_In DDRC &=B01111111

#define D14_In DDRF &=B01111111
#define D15_In DDRF &=B10111111
#define D16_In DDRF &=B11011111
#define D17_In DDRF &=B11101111
#define D18_In DDRF &=B11111101
#define D19_In DDRF &=B11111110

//***************Output Pins*************
#define D0_Out DDRD |=B00000100
#define D1_Out DDRD |=B00001000
#define D2_Out DDRD |=B00000010
#define D3_Out DDRD |=B00000001
#define D4_Out DDRD |=B00010000
#define D5_Out DDRC |=B01000000
#define D6_Out DDRD |=B10000000
#define D7_Out DDRE |=B01000000
 
#define D8_Out DDRB  |=B00010000
#define D9_Out DDRB  |=B00100000
#define D10_Out DDRB |=B01000000
#define D11_Out DDRB |=B10000000
#define D12_Out DDRD |=B01000000
#define D13_Out DDRC |=B10000000

#define D14_Out DDRF |=B10000000
#define D15_Out DDRF |=B01000000
#define D16_Out DDRF |=B00100000
#define D17_Out DDRF |=B00010000
#define D18_Out DDRF |=B00000010
#define D19_Out DDRF |=B00000001

//***************Status High Pins*************
#define D0_High PORTD |=B00000100
#define D1_High PORTD |=B00001000
#define D2_High PORTD |=B00000010
#define D3_High PORTD |=B00000001
#define D4_High PORTD |=B00010000
#define D5_High PORTC |=B01000000
#define D6_High PORTD |=B10000000
#define D7_High PORTE |=B01000000
 
#define D8_High PORTB  |=B00010000
#define D9_High PORTB  |=B00100000
#define D10_High PORTB |=B01000000
#define D11_High PORTB |=B10000000
#define D12_High PORTD |=B01000000
#define D13_High PORTC |=B10000000

#define D14_High PORTF |=B10000000
#define D15_High PORTF |=B01000000
#define D16_High PORTF |=B00100000
#define D17_High PORTF |=B00010000
#define D18_High PORTF |=B00000010
#define D19_High PORTF |=B00000001


//***************Status Low Pins*************
#define D0_Low PORTD &=B11111011
#define D1_Low PORTD &=B11110111
#define D2_Low PORTD &=B11111101
#define D3_Low PORTD &=B11111110
#define D4_Low PORTD &=B11101111
#define D5_Low PORTC &=B10111111
#define D6_Low PORTD &=B01111111
#define D7_Low PORTE &=B10111111
 
#define D8_Low PORTB  &=B11101111
#define D9_Low PORTB  &=B11011111
#define D10_Low PORTB &=B10111111
#define D11_Low PORTB &=B01111111
#define D12_Low PORTD &=B10111111
#define D13_Low PORTC &=B01111111

#define D14_Low PORTF &=B01111111
#define D15_Low PORTF &=B10111111
#define D16_Low PORTF &=B11011111
#define D17_Low PORTF &=B11101111
#define D18_Low PORTF &=B11111101
#define D19_Low PORTF &=B11111110

//**************Invert Status Pins*************
#define D0_Inv PORTD ^=B00000100
#define D1_Inv PORTD ^=B00001000
#define D2_Inv PORTD ^=B00000010
#define D3_Inv PORTD ^=B00000001
#define D4_Inv PORTD ^=B00010000
#define D5_Inv PORTC ^=B01000000
#define D6_Inv PORTD ^=B10000000
#define D7_Inv PORTE ^=B01000000
 
#define D8_Inv PORTB  ^=B00010000
#define D9_Inv PORTB  ^=B00100000
#define D10_Inv PORTB ^=B01000000
#define D11_Inv PORTB ^=B10000000
#define D12_Inv PORTD ^=B01000000
#define D13_Inv PORTC ^=B10000000

#define D14_Inv PORTF ^=B10000000
#define D15_Inv PORTF ^=B01000000
#define D16_Inv PORTF ^=B00100000
#define D17_Inv PORTF ^=B00010000
#define D18_Inv PORTF ^=B00000010
#define D19_Inv PORTF ^=B00000001

//**************READ Status Pins*************
#define D0_Read ((PIND & B00000100)>>2)
#define D1_Read ((PIND & B00001000)>>3)
#define D2_Read ((PIND & B00000010)>>1)
#define D3_Read ((PIND & B00000001)
#define D4_Read ((PIND & B00010000)>>4)
#define D5_Read ((PINC & B01000000)>>6)
#define D6_Read ((PIND & B10000000)>>7)
#define D7_Read ((PINE & B01000000)>>6)
 
#define D8_Read ((PINB  & B00010000)>>4)
#define D9_Read ((PINB  & B00100000)>>5)
#define D10_Read ((PINB & B01000000)>>6)
#define D11_Read ((PINB & B10000000)>>7)
#define D12_Read ((PIND & B01000000)>>6)
#define D13_Read ((PINC & B10000000)>>7)

#define D14_Read ((PINF & B10000000)>>7)
#define D15_Read ((PINF & B01000000)>>6)
#define D16_Read ((PINF & B00100000)>>5)
#define D17_Read ((PINF & B00010000)>>4)
#define D18_Read ((PINF & B00000010)>>1)
#define D19_Read ((PINF & B00000001)


//**************Analog READ*******************
#define A0_Read (AnRead(B10000000))
#define A1_Read (AnRead(B01000000))
#define A2_Read (AnRead(B00100000))
#define A3_Read (AnRead(B00010000))
#define A4_Read (AnRead(B00000010))
#define A5_Read (AnRead(B00000001))
uint16_t AnRead(uint8_t An_pin);


//**************Delay*************************	
	void delay_us(uint16_t tic_us);	
	void delay_ms(uint16_t tic_ms);
	
//**************Low High Pins**********************
void D_High(uint16_t Pin);
void D_Low(uint16_t Pin);
void D_High2(uint16_t Pin);
void D_Low2(uint16_t Pin);

void D_Pin(uint16_t Pin, uint16_t Status);
void D_Pin2(uint16_t Pin, uint16_t Status);

		
#else
#error  Ваш контроллер библиотекой CyBerLib не поддерживается
#endif

	
//**************End***************************
#endif //CyberLib_H
