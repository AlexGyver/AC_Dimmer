#include "CyberLib.h"
//*************************************************Все на 328 процессоре**********************************
//********************************************************************************************************
//********************************************************************************************************
#if defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
//**********Small UART****************************
#define UART_DOUBLESPEED
#define UART_TXREADY UDRE0
#define UART_RXREADY RXC0
#define UART_DOUBLE	U2X0
#define UDR  UDR0
#define UCRA UCSR0A
#define UCRB UCSR0B
#define UCRC UCSR0C
#define UCRC_VALUE ((1<<UCSZ01) | (1<<UCSZ00))
#define RXCIE RXCIE0
#define TXCIE TXCIE0
#define RXEN RXEN0
#define TXEN TXEN0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define SIG_UART_TRANS SIG_USART_TRANS
#define SIG_UART_RECV  SIG_USART_RECV
#define SIG_UART_DATA  SIG_USART_DATA
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 4UL)) / ((uint32_t)(baudRate) * 8UL) - 1)

void UART_Init(uint32_t UART_BAUD_RATE)
{
	UBRRH = (UART_CALC_BAUDRATE(UART_BAUD_RATE)>>8) & 0xFF;
	UBRRL = (UART_CALC_BAUDRATE(UART_BAUD_RATE) & 0xFF);
	UCSR0A = ( 1<<UART_DOUBLE );
	UCRB = ((1<<TXEN) | (1<<RXEN));
	UCRC = UCRC_VALUE;
}

void UART_SendByte(uint8_t data)
{
	while (!(UCRA & (1<<UART_TXREADY)));
	UDR = data;
}

bool UART_ReadByte(uint8_t& data)
{
	if (UCRA & (1<<UART_RXREADY))
	{
		data = UDR;
		return true;
	} else return false;
}

void UART_SendArray(uint8_t *buffer, uint16_t bufferSize)
{
for(uint16_t i=0; i<bufferSize; i++)
  UART_SendByte(buffer[i]);
}
//************SPI********************************
#define SPI_MODE0 SPCR &= ~((1<<CPOL)|(1<<CPHA))
#define SPI_MODE1 SPCR = (1<<CPHA)
#define SPI_MODE2 SPCR = (1<<CPOL)
#define SPI_MODE3 SPCR = (1<<CPOL)|(1<<CPHA)
	//Передний фронт Задний фронт
	// CPOL = 0		CPHA = 0	Выборка нарастающим фронтом, Установка данных падающим фронтом	mode0
	// CPOL = 0		CPHA = 1	Установка данных нарастающим фронтом, Выборка падающим фронтом	mode1
	// CPOL = 1		CPHA = 0	Выборка падающим фронтом, Установка данных нарастающим фронтом	mode2
	// CPOL = 1		CPHA = 1	Установка данных падающим фронтом, Выборка нарастающим фронтом	mode3
	
#define SPI_DIV2 SPSR |= (1<<SPI2X);
#define SPI_DIV4 SPCR &= ~((1<<SPR1)|(1<<SPR0))
#define SPI_DIV8 SPSR |= (1<<SPI2X); SPCR |=(1<<SPR0)
#define SPI_DIV16 SPCR |=(1<<SPR0)
#define SPI_DIV32 SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1)
#define SPI_DIV64 SPCR |= (1<<SPR1)
//#define SPI_DIV64 SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1)|(1<<SPR0)
#define SPI_DIV128 SPCR |= (1<<SPR1)|(1<<SPR0) 
	// SPI2X	SPR1	SPR0	Частота SCK
	// 1		0		0		fosc/2
	// 0		0		0		fosc/4
	// 1		0		1		fosc/8
	// 0		0		1		fosc/16
	// 1		1		0		fosc/32
	// 0		1		0		fosc/64
	// 1		1		1		fosc/64
	// 0		1		1		fosc/128	
	
#define SPI_MSB SPCR &= ~(1<<DORD)
#define SPI_LSB SPCR |= (1<<DORD)
	// DORD |= (0<<DORD); //Порядок сдвига данных. DORD=1 первым передается младший бит. DORD=0 первым передается старший бит.	
	// SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0
    // (1<<MSTR); //режим Master
    // (1<<SPE); // Включить SPI
	// (1<<SPIE); //Включить прерывание
	// (1<<SPI2X); //Удвоить частоту тактирования
	// (0<<CPOL); //полярность тактового сигнала
	// (0<<CPHA); //фаза тактового сигнала	
	
void StartSPI(uint8_t SPI_Mode, uint8_t SPI_Div, uint8_t SPI_Change_Shift )
{
	D10_Out; D10_High;	//SS
	D11_Out; D11_Low;	//MOSI
	D12_In;				//MISO
	D13_Out; D13_Low;	//CLK	

	SPCR = 0; //моде0, MSB, 
	SPSR &= ~(1<<SPI2X); //отключить удвоение скорости
	
	switch (SPI_Mode) 
	{
    case 1: SPI_MODE1; break;
    case 2: SPI_MODE2; break;
    case 3: SPI_MODE3; break;	  
	}
	
	switch (SPI_Div) 
	{
    case 2: SPI_DIV2; break;	
   // case 4: SPI_DIV4; break;
    case 8: SPI_DIV8; break;
    case 16: SPI_DIV16; break;	
    case 32: SPI_DIV32; break;
    case 64: SPI_DIV64; break;
    case 128: SPI_DIV128; break;	  
	}	
	
	if(SPI_Change_Shift==0)SPI_LSB; //напрвление байта LSB или MSB
	
	SPCR |= (1<<SPE)|(1<<MSTR); //Включить SPI в реж. Мастер
	//SPSR |= (1<<SPI2X);
	//SPCR = (1<<SPIE)|(1<<SPE)|(1<<DORD)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR1)|(1<<SPR0); 
}

void StopSPI(void) 
{
	SPCR &= ~(1<<SPE);
}

uint8_t ReadSPI(void) 
{
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

void SendSPI(uint8_t SPI_data) 
{
  SPDR = SPI_data;
  while(!(SPSR & (1<<SPIF)));
			
    // __asm__ volatile
			// (
			// ".spi_not_ready: out %[spdr], %[spi_data]"	"\n\t" //отправляем пользовательские данные в SPI
            // "in __tmp_reg__,%[spsr]"				"\n\t"	//Читаем регистр SPSR
            // "sbrs __tmp_reg__, %[spif]"				"\n\t"	//и проверяем флаг SPIF
            // "rjmp .spi_not_ready"					"\n\t"	//если данные не отправлены повторяем процедуру
            // ::
            // [spsr] "I" (_SFR_IO_ADDR(SPSR)),
            // [spif] "I" (SPIF),
            // [spdr] "I" (_SFR_IO_ADDR(SPDR)),
            // [spi_data] "r" (SPI_data)
            // );							
}

//**********AnalogRead***************************
uint16_t AnRead(uint8_t An_pin)
{
  ADMUX=An_pin;   
  delay_us(10);	  
  ADCSRA=B11000110;	//B11000111-125kHz B11000110-250kHz 
  while (ADCSRA & (1 << ADSC));
  An_pin = ADCL;
  uint16_t An = ADCH; 
  return (An<<8) + An_pin;
}
//**********Converter****************************
uint8_t CharToDec(uint8_t digit)
{
	return digit-48;
}

uint8_t DecToChar(uint8_t number)
{
	return number+48;
}

//******************EEPROM*******************************
void WriteEEPROM_Byte(uint8_t addr, uint8_t data)  //сохранить в EEPROM
{
		eeprom_write_byte((uint8_t*)addr, data);
}

void WriteEEPROM_Word(uint16_t addr, uint16_t data)
{
		eeprom_write_word((uint16_t*)addr, data);
}

void WriteEEPROM_Long(uint8_t addr, uint32_t data)  //сохранить в EEPROM
{           
  addr *= 4;
        eeprom_write_byte((uint8_t*)addr, data & 0xFF);
        eeprom_write_byte((uint8_t*)addr+1, (data & 0xFF00) >> 8);
        eeprom_write_byte((uint8_t*)addr+2, (data & 0xFF0000) >> 16);
        eeprom_write_byte((uint8_t*)addr+3, (data & 0xFF000000) >> 24);
		
	  // addr *= 2;
        // eeprom_write_word((uint16_t*)addr, data & 0xFFFF);
        // eeprom_write_word((uint16_t*)addr+1, (data & 0xFFFF0000) >> 16);
}

uint8_t ReadEEPROM_Byte(uint8_t addr)
{
		return eeprom_read_byte((uint8_t*)addr);
}

uint16_t ReadEEPROM_Word(uint16_t addr)
{
		return eeprom_read_word((uint16_t*)addr);
}

uint32_t ReadEEPROM_Long(uint8_t addr)  // считываем значение из EEPROM
{
  addr *= 4; 
        uint32_t ir_code = eeprom_read_byte((uint8_t*)addr+3); 
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+2);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+1);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr);
		//eeprom_read_word((uint16_t*) addr)
  return ir_code;
}
//**************Timer****************************
#define DIV_0    TCCR1B = (1 << CS10) //Делитель 0
#define DIV_8    TCCR1B = (1 << CS11) //Делитель 8
#define DIV_64   TCCR1B = ((1 << CS11) | (1 << CS10)) //Делитель 64
#define DIV_256  TCCR1B = (1 << CS12) //Делитель 256
#define DIV_1024 TCCR1B = ((1 << CS12) | (1 << CS10)) //Делитель 1024

void (*func)();
volatile uint16_t dub_tcnt1;
 
void StartTimer1(void (*isr)(), uint32_t set_us)
{	
//	cli();
	TIMSK1 &= ~(1<<TOIE1);//запретить прерывания по таймеру1
	func = *isr; 	//указатель на функцию
	TCCR1A = 0;		//timer1 off
	TCCR1B = 0; 	//prescaler off (1<<CTC1)-3й бит

	//uint8_t oldSREG = SREG;

	//if(set_us < 6) set_us = 6;	//min
	//if(set_us > 4194304) set_us = 4194303;  //max
	if(set_us > 5 && set_us < 4096) { set_us = 65584 - (set_us << 4); DIV_0;} else
	if(set_us > 4095 && set_us < 32768) { set_us = 65542 - (set_us << 1); DIV_8; } else
	if(set_us > 32767 && set_us < 262144) { set_us = 65536 - (set_us >> 2); DIV_64;} else
	if(set_us > 262143 && set_us < 1048576) { set_us = 65536 - (set_us >> 4); DIV_256; } else
	if(set_us > 1048575 && set_us < 4194304) { set_us = 65536 - (set_us >> 6);  DIV_1024;} else TCCR1B = 1;
	
	
	dub_tcnt1 = set_us;
	TCNT1 = 0;//dub_tcnt1;	// выставляем начальное значение TCNT1 
	//OCR1A = dub_tcnt1;	
	//TCNT1H=0;//обнуляем регистр TCNT1
	//TCNT1L=0;
	TIMSK1 |= (1 << TOIE1); // разрешаем прерывание по переполнению таймера	
	sei();   
}

void StopTimer1(void)
{
  TIMSK1 &= ~(1<<TOIE1);    //запретить прерывания по таймеру1
}

void ResumeTimer1(void)
{
	TIMSK1 |= (1<<TOIE1);	//Продолжить отсчет, (разрешить прерывания по таймеру1)
}

void RestartTimer1(void)
{
	TCNT1 = dub_tcnt1;
	TIMSK1 |= (1<<TOIE1);	//разрешить прерывания по таймеру1
}

ISR(TIMER1_OVF_vect) 
{
	TCNT1 = dub_tcnt1;	  
	(*func)();
}
//**************Поиск макс повторяющегося элемента в массиве****************************
uint16_t find_similar(uint16_t *buf, uint8_t size_buff, uint8_t range) 
{
 uint8_t maxcomp=0; //счётчик максимального колличества совпадений
 uint16_t mcn=0;	//максимально часто встречающийся элемент массива
 uint16_t comp;	//временная переменная
 range++;	//допустимое отклонение

	for (uint8_t i=0; i<size_buff; i++) 
	{
		comp=buf[i];	//кладем элемент массива в comp
		uint8_t n=0;	//счётчик совпадении
		for (uint8_t j=0; j<size_buff; j++)	{ if (buf[j]>comp-range && buf[j]<comp+range) n++;} // ищем повторения элемента comp в массиве buf	
		if (n > maxcomp) //если число повторов больше чем было найдено ранее
		{
			maxcomp=n; //сохраняем счетяик повторов
			mcn=comp; //сохраняем повторяемый элемент
		}		
	}
 return mcn;
}
//***************Beep от 50 до 2000Гц****************
void beep(uint16_t dur, uint16_t frq)
{
  //dur=(1000/frq)*dur;  //расчет длительности бипа
  uint16_t per=500000/frq; //длит. полупер в мкс
  dur=dur/(per/250)*2;
  for(uint16_t i=0; i<dur; i++)
  {
    D11_High; 
    delay_us(per); 
    D11_Low;
    delay_us(per);
  } 
} 
//**************програмный сброс*******************
void reset() 
{
	  wdt_disable();  //отключить, так как он может быть уже включен
	  wdt_enable(WDTO_15MS); //установим минимально возможное время 15мс
	  while (1) {}	//зациклить
}

//**************Delay****************************
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 цикла
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; вычесть из регистра значение N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}
// ******************************************MEGA**********************************************
//*********************************************************************************************
//*********************************************************************************************
#elif  defined (__AVR_ATmega2560__) || defined (__AVR_ATmega2561__)
//******************EEPROM*******************************
void WriteEEPROM_Byte(uint8_t addr, uint8_t data)  //сохранить в EEPROM
{
		eeprom_write_byte((uint8_t*)addr, data);
}

void WriteEEPROM_Word(uint16_t addr, uint16_t data)
{
		eeprom_write_word((uint16_t*)addr, data);
}

void WriteEEPROM_Long(uint8_t addr, uint32_t data)  //сохранить в EEPROM
{           
  addr *= 4;
        eeprom_write_byte((uint8_t*)addr, data & 0xFF);
        eeprom_write_byte((uint8_t*)addr+1, (data & 0xFF00) >> 8);
        eeprom_write_byte((uint8_t*)addr+2, (data & 0xFF0000) >> 16);
        eeprom_write_byte((uint8_t*)addr+3, (data & 0xFF000000) >> 24);
		
	  // addr *= 2;
        // eeprom_write_word((uint16_t*)addr, data & 0xFFFF);
        // eeprom_write_word((uint16_t*)addr+1, (data & 0xFFFF0000) >> 16);
}

uint8_t ReadEEPROM_Byte(uint8_t addr)
{
		return eeprom_read_byte((uint8_t*)addr);
}

uint16_t ReadEEPROM_Word(uint16_t addr)
{
		return eeprom_read_word((uint16_t*)addr);
}

uint32_t ReadEEPROM_Long(uint8_t addr)  // считываем значение из EEPROM
{
  addr *= 4; 
        uint32_t ir_code = eeprom_read_byte((uint8_t*)addr+3); 
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+2);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr+1);
        ir_code = (ir_code << 8) | eeprom_read_byte((uint8_t*)addr);
		//eeprom_read_word((uint16_t*) addr)
  return ir_code;
}
//**************Delay****************************
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 цикла
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; вычесть из регистра значение N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}
//*************************************************Leonardo и все на 32U4**********************************
//********************************************************************************************************
//********************************************************************************************************
#elif defined (__AVR_ATmega32U4__)
void delay_ms(uint16_t tic_ms)
{
		while(tic_ms)
		{
			delay_us(999);
			tic_ms--;
		}
}

void delay_us(uint16_t tic_us)
{
	tic_us *= 4; //1us = 4 цикла
	__asm__ volatile 
		  (	
			"1: sbiw %0,1" "\n\t" //; вычесть из регистра значение N
			"brne 1b"				
			: "=w" (tic_us)
			: "0" (tic_us)
		  );
}

#endif

