/*
 * energise.h
 *
 *  Created on: 2 cze 2015
 *      Author: Karol
 */

#ifndef ENERGISE_H_
#define ENERGISE_H_
#include <avr/io.h>
///////////// wyjscia przerzutników/////////////

//Q1
#define PD2_PIN (1<<PD1)
#define PD2_L (!(PIND & PD2_PIN))

//Q2
#define PD3_PIN (1<<PD5)
#define PD3_L (!(PIND & PD3_PIN))

//Q3
#define PD4_PIN (1<<PD6)
#define PD4_L (!(PIND & PD4_PIN))

//Q4
#define PD5_PIN (1<<PD0)
#define PD5_L (!(PIND & PD5_PIN))


//Q5
#define PD6_PIN (1<<PC4)
#define PD6_L (!(PINC & PD6_PIN))

//Q_RESET
#define Q_RESET_PIN (1<<PD4)
#define Q_RESET_H PORTD |=Q_RESET_PIN
#define Q_RESET_L PORTD &=~Q_RESET_PIN
//////////////////////////////////////////////////////////////////

//LED
#define LED_PIN (1<<PB7)
#define LED_OFF PORTB |=LED_PIN
#define LED_ON PORTB &=~LED_PIN
#define LED_TOG PORTB ^=LED_PIN


//ADC_EN// kontrola nap. akumulatora
#define ADC_EN_PIN (1<<PC1)
#define ADC_EN_OFF PORTC |=ADC_EN_PIN
#define ADC_EN_ON PORTC &=~ADC_EN_PIN

//NRF_EN// kontola zaisilania NRF
#define NRF_EN_PIN (1<<PB0)
#define NRF_ON PORTB |=NRF_EN_PIN
#define NRF_OFF 	PORTB &=~NRF_EN_PIN

// Status ³adowania solarnego
#define CHARG_STAT_PIN (1<<PC2)
#define CHARG_STAT (!(PINC & CHARG_STAT_PIN))
///////////////

///NRF
#define W 1
#define R 0

#define CSN_PIN (1<<PB2)
#define CSN_H PORTB |= CSN_PIN
#define CSN_L PORTB &=~CSN_PIN

#define CE_PIN (1<<PD7)
#define CE_H PORTD |= CE_PIN
#define CE_L PORTD &=~CE_PIN


uint8_t SPI_transmit(uint8_t x);
uint8_t GetReg(uint8_t reg);
void WriteToNRF(uint8_t reg, uint8_t Package);
uint8_t *NRF_Transmit(uint8_t R_W, uint8_t reg, uint8_t *val, uint8_t antVal);
void nrf_init_TX(void);
void NRF_Buffer_Send (uint8_t *W_Buff);
void NRF_reset(void);
void SPI_master_init(void);
void ADC_init(void);
void Interrupt_init(void);
void Timer0_init(void);
void V_reset (void);

#endif /* ENERGISE_H_ */
