#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nrf24l01.h"
#include "energise.h"
#include <avr/wdt.h>

uint8_t pomiar=100;
uint8_t v_level=0;
uint8_t period=0;
uint8_t reciv_unconf =0; // zmienna zliczajaca brak potwierdzenia odbioru

void Led_blink(int delay)
{
			 LED_ON;
			 _delay_ms(delay);
			 LED_OFF;
			 _delay_ms(delay);
}

void CPU_Reset()  // reset procesora
{
	wdt_enable(WDTO_15MS);
	cli();
	while(1)
		{
		LED_ON;
		}
}

int main(void)
{
	//DDRB |= LED_PIN;  !!!!! wylaczona dioda
	DDRB |= CSN_PIN|NRF_EN_PIN;
	DDRD |= Q_RESET_PIN |CE_PIN;
	DDRC |=	ADC_EN_PIN;


	// !  JESZCZE WYJSCIA PRZERZUTNIKÓW
	DDRC &=~CHARG_STAT_PIN;
	PORTC |=CHARG_STAT_PIN;

	LED_OFF;

	//wyl NRf
	NRF_OFF;
	ADC_EN_OFF;
	V_reset ();
	_delay_ms(1000);
	//



	SPI_master_init();
	ADC_init(); // !!! zrób singel mode
	Interrupt_init();
	Timer0_init();

	NRF_ON;
	ADC_EN_ON;
	_delay_ms(2000);	//Czas na obudzenie NRF'a

	 uint8_t test =0;
	 test = GetReg(STATUS);
	 nrf_init_TX();
	 uint8_t bufor[5];
	 bufor[0] = 11;
	 bufor[1] = 10;
	 bufor[2] = 10;
	 bufor[3] = 10;
	 bufor[4] = 10;

	 Led_blink(200);

	 while(!(test == 0x0E))  //B³ad komunikacji z NRF
	 {
		 LED_ON;
		 _delay_ms(1000);
		 LED_OFF;
		 _delay_ms(1000);

		 CPU_Reset(); // reset ukladu jesli blad komunikacji z nrf

	 }

	 sei();
	 ADCSRA |= (1<<ADSC);
	 _delay_ms(1000);


	while(1)
		{


			 if(period>6)
			 	{
				 reciv_unconf ++; //inkrementuj licznik odbioru
				 Led_blink(50);
				 if(reciv_unconf>10) CPU_Reset(); //jesli brak potwierdzenia odbioru to reset uk³adu

				 wdt_enable(WDTO_2S); //wlaczenie Watchdog

				 if(!(ADCSRA & (1<<ADSC)))
				 		 			{
				 		 				pomiar = ADCH;
				 		 				ADC_EN_ON;
				 		 				ADCSRA |= (1<<ADSC);
				 		 			}

				// 	 LED_ON;
				 CE_H;
				 	 _delay_ms(1);
				 	 bufor[0]=pomiar;
				 	 if(CHARG_STAT)bufor[1]=0;
				 	 else bufor[1]=1;

				 	 bufor[2] = v_level;
				 	 v_level = 0;
				 	 //bufor[1]=0;
				 	 NRF_Buffer_Send(bufor);
				 	 NRF_reset();
				 	 CE_L;
				 	 period = 0;

				 	 V_reset ();
				 	LED_OFF;
				 //	_delay_ms(10);

				 	wdt_disable(); // wylaczenie Watchdog

				 	MCUCR &=~((1<<SM2) | (1<<SM1)|(1<<SM0));
			 	}

//			 LED_ON;
//			_delay_ms(20);
//			LED_OFF;
			_delay_ms(200);


		}
}

ISR(TIMER0_OVF_vect)
{
	if(period<8) period++;
	if(period<6) MCUCR &=~((1<<SM2) | (1<<SM1)|(1<<SM0));
	if(!(ADCSRA & (1<<ADSC))) ADC_EN_OFF;

	//LED_ON;
	//_delay_ms(20);
	//LED_OFF;
	//V_reset ();
}

SIGNAL(INT1_vect)	// Przerwanie od fotodiody (najni¿szy poziom)
{
	v_level=1;

	if(!(PD2_L))
		{
		v_level++;
		if(!(PD3_L))
			{
			v_level++;
			if(!(PD4_L))
				{
				v_level++;
				if(!(PD5_L))
					{
					v_level++;
					if(!(PD6_L))
						{
						v_level++;
						}
					}
				}
			}
		}

	period = 6;
}

SIGNAL(INT0_vect) // Przerwanie od NRF'a jesli pakiet zostal poprawnie wyslany
{
	 reciv_unconf=0;
//	LED_ON;
//	_delay_ms(20);
//	LED_OFF;
}
