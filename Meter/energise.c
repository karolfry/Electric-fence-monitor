/*
 * energise.c
 *
 *  Created on: 2 cze 2015
 *      Author: Karol
 */

#include "energise.h"
#include "nrf24l01.h"
#include <avr/io.h>
#include <util/delay.h>

uint8_t SPI_transmit(uint8_t x)
{
	SPDR =x;
	while(!(SPSR&(1<<SPIF))) ;
	return SPDR;
}

uint8_t GetReg(uint8_t reg)
{
_delay_us(10);
CSN_L;
_delay_us(10);
SPI_transmit(R_REGISTER+reg);
_delay_us(10);
reg = SPI_transmit(NOP);
_delay_us(10);
CSN_H;
return reg;
}

void WriteToNRF(uint8_t reg, uint8_t Package)
{
	_delay_us(10);
	CSN_L;
	_delay_us(10);
	SPI_transmit(W_REGISTER + reg);
	_delay_us(10);
	SPI_transmit(Package);
	_delay_us(10);
	CSN_H;
}

uint8_t *NRF_Transmit(uint8_t R_W, uint8_t reg, uint8_t *val, uint8_t antVal)
{
	if(R_W ==W) reg = W_REGISTER +reg;

	static uint8_t ret[32];
	_delay_us(10);
	CSN_L;
	_delay_us(10);
	SPI_transmit(reg);
	_delay_us(10);

	int i;
	for(i=0; i<antVal;i++)
	{
		if(R_W == R && reg != W_TX_PAYLOAD)
		{
			ret[i]=SPI_transmit(NOP);
			_delay_us(10);
		}
		else
		{
			SPI_transmit(val[i]);
			_delay_us(10);
		}
	}
	CSN_H;
	return ret;
}

void nrf_init_TX(void)
{
_delay_ms(100);

uint8_t val[5];
//EN_AA enable-potwierdzenie odbioru
val[0]=0x01;
NRF_Transmit(W,EN_AA,val,1);

//number of enabled data pipes(1-5)
val[0] = 0x01;
NRF_Transmit(W,EN_RXADDR,val,1); //data pipe 0

//adres width
val[0]=0x03;
NRF_Transmit(W,SETUP_AW,val,1);//5 bytes adress

//RF channel setup
val[0]=38; //250kbps -0db
NRF_Transmit(W,RF_SETUP,val,1);

//RX adress
uint8_t i;
for(i=0;i<5;i++)
{
	val[i]=0x12;

}

NRF_Transmit(W,RX_ADDR_P0,val,5);

//TX adress
for(i=0;i<5;i++)
{
	val[i]=0x12;

}
NRF_Transmit(W,TX_ADDR,val,5);

//d³ugosc danych
val[0] =5;
NRF_Transmit(W,RX_PW_P0,val,1);

//Config reg: RX, POWER ON,
val[0] = 0x1E;
NRF_Transmit(W,CONFIG,val,1);

_delay_ms(100);

//retry time and number
val[0]=0x2F;
NRF_Transmit(W,SETUP_RETR,val,1);
}

void NRF_Buffer_Send (uint8_t *W_Buff)
{
	NRF_Transmit(R,FLUSH_TX,W_Buff,0);
	NRF_Transmit(R,W_TX_PAYLOAD,W_Buff,5);

	_delay_ms(10);
	CE_H;
	_delay_us(200);
	CE_L;
	_delay_ms(10);
}

void NRF_reset(void)
{
	_delay_us(10);
	CSN_L;
	_delay_us(10);
	SPI_transmit(W_REGISTER+STATUS);
	_delay_us(10);
	SPI_transmit(0x70);
	_delay_us(10);
	CSN_H;
}

void SPI_master_init(void)
{
	//spi master init
	DDRB |=(1<<PB3)|(1<<PB5);
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
	CSN_H;
	CE_L;

}

void ADC_init(void)
{
	//ADC
	  ADCSRA |= (1<<ADPS2)|(1<<ADPS1); // preskaler
	  ADMUX |= (1<<REFS1|1<<REFS0); // nap odniesienia
	  ADMUX |= (1<<ADLAR); // pom 8-bit
	  ADCSRA |=(1<<ADEN); // w³ przetwornika
}

void Interrupt_init(void)
{
	//int0 (od NRF'a)
	DDRD &=~(1<<PD2);
	MCUCR |= (1<<ISC01);
	GICR |=(1<<INT0);

	//int1 (od Fotodiody)
	DDRD &=~(1<<PD3);
	//MCUCR |= (1<<ISC11);
	MCUCR |= (1<<ISC10)|(1<<ISC11); //wyzwalanie zboczem narastaj¹cym
	GICR |=(1<<INT1);


}

void Timer0_init(void)
{
	//timer0
	 TCCR0 |= (1<<CS02)|(1<<CS00);
	 TIMSK |= (1<<TOIE0);
}

void V_reset(void)
{
	Q_RESET_L;
	_delay_ms(10);
	Q_RESET_H;
}
