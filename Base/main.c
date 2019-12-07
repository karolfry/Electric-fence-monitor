/*
 * main.c
 *
 *  
 *      Author: Karol
 */


#include <avr/io.h>
#include <util/delay.h>
#include "nrf24l01.h"
#include <avr/interrupt.h>
#include "HD44780.h"
#include "1Wire/ds18x20.h"
#include <avr/eeprom.h>

#define LED_PIN (1<<PB7)
#define LED_OFF PORTB |=LED_PIN
#define LED_ON PORTB &=~LED_PIN
#define LED_TOG PORTB ^=LED_PIN

#define LCD_LED_PIN (1<<PC3)
#define LCD_LED_ON PORTC |=LCD_LED_PIN
#define LCD_LED_OFF PORTC &=~LCD_LED_PIN

#define BUZER_PIN (1<<PB1)
#define BUZER_ON PORTB |=BUZER_PIN
#define BUZER_OFF PORTB &=~BUZER_PIN
#define BUZER_TOG PORTB ^=BUZER_PIN

#define OUT_PIN (1<<PB1)
#define OUT_ON PORTB |= OUT_PIN;
#define OUT_OFF PORTB &=~OUT_PIN


#define CE_PIN (1<<PD7)
#define CE_H PORTD |= CE_PIN
#define CE_L PORTD &=~CE_PIN

#define CSN_PIN (1<<PB2)
#define CSN_H PORTB |= CSN_PIN
#define CSN_L PORTB &=~CSN_PIN

#define SWITCH_PIN (1<<PD5)
#define SWITCH_DW (!(PIND & SWITCH_PIN))

#define W 1
#define R 0

uint8_t flag=0;
static uint8_t* data;
float temp_f=0;
uint8_t temp_d=0,temp_d1=0,temp_c=0;
uint8_t charging;
uint8_t con_stat=10;
uint8_t lcd_led_time = 20;
uint8_t znak, cal, dzies;
uint8_t znak_l, cal_l, dzies_l;
uint8_t temp_DS1820=200,flag_DS1820=0;
uint8_t v_level=0,v_level_last=10;
uint8_t w_flag=0;
uint8_t con_flag=0;
uint8_t z_count=0;
uint8_t alarm_flag=0;
uint8_t con_alarm_flag=0;
uint8_t con_alarm_count=0;
uint8_t count=0;
uint8_t con_alarm_off=0;
uint8_t alarm_off=0;
uint8_t last_V_mesure;

uint8_t alarm_prog =0; // prog wyzwalania alarmu
uint8_t alarm_prog_new =0; //nowy prog wyzwalania alarmu w ustawieniach
uint8_t EEMEM alarm_prog_eeprom; // przechowywanie poziomu wyzwalania w eeprom
uint8_t eeprom_save=0; //flaga do zapisu w eeprom

uint8_t al_p_change = 0; //wejscie do ustawien
uint8_t al_p_blink_count = 0; // licznik od migania znacznikiem
uint8_t al_p_blink_stat = 0;//flaga wyzwalajace wyswietlanie znacznika w petli glownej

uint8_t chang_al_prog_count=0; //licznik sprawdzajacy przytrzymanie switcha przed wejsciem do ustawien
uint8_t sw_relase=0;	//flaga do sprawdzania czy przycisk zostal puszczony

uint8_t disable_al_prog_count=0; //licznik odliczajacy do wyjscia z ustawien, jesli brak aktywnosci



//definicje wlasnych znakow do lcd
char znak1[8] = {0x1F,0x11,0x11,0x11,0x11,0x11,0x11,0x1F};//puste pole od wskaznika
char znak2[8] = {0x02,0x04,0x08,0x1F,0x02,0x14,0x18,0x1C};//blyskawica
char znak3[8] = {4,10,10,10,10,17,17,14};	//termometr
char znak4[8] = {14,27,17,17,31,31,31,31}; //bateria
char znak5[8] = {8,20,8,7,8,8,8,7}; //stopnie celc.
char znak6[8] = {4,14,31,27,17,32,32,32};

uint8_t SPI_transmit(uint8_t x)
{
	SPDR =x;
	while(!(SPSR&(1<<SPIF))) ;
	return SPDR;
}
/////////////////////////////NRF////////
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
val[0] = 0x1F;
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

/////////////////////////////////////////////


//DS18b20
void wysw_temp()
{
	//LCD_Clear();
	LCD_GoTo(10,1);
	LCD_WriteText("       ");

	if(znak)
		{
		LCD_WriteText("-");
		LCD_GoTo(10,1);
		}
	else LCD_GoTo(11,1);
	LCD_Int(cal);
	LCD_WriteText(",");
	LCD_Int(dzies);
	LCD_WriteData(0x04);


	LCD_GoTo(0,0);
	//LCD_Int(gSensorIDs[0]);



}

int main(void)
{
DDRB |= LED_PIN | CSN_PIN |OUT_PIN|BUZER_PIN;
DDRD |= CE_PIN;
DDRC |= LCD_LED_PIN;

DDRD &= ~(SWITCH_PIN);
PORTD |= SWITCH_PIN;

LCD_LED_OFF;
LED_ON;


//spi master init
DDRB |=(1<<PB3)|(1<<PB5);
SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
CSN_H;
CE_L;
//////////

//timer0
TCCR0 |= (1<<CS02)|(1<<CS00);
TIMSK |= (1<<TOIE0);


LCD_Initalize();

//szuknie Ds1820
uint8_t liczba_sensorow;
	liczba_sensorow = search_sensors();

	DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);

//wgranie znakow do lcd

LCD_WriteCommand(0x40);

for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak1[i]);
	}
LCD_WriteCommand(0x80);

LCD_WriteCommand(0x48);

for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak2[i]);
	}
LCD_WriteCommand(0x80);

LCD_WriteCommand(0x50);

for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak3[i]);
	}
LCD_WriteCommand(0x80);

LCD_WriteCommand(0x58);

for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak4[i]);
	}
LCD_WriteCommand(0x80);

LCD_WriteCommand(0x60);

for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak5[i]);
	}
LCD_WriteCommand(0x80);

LCD_WriteCommand(0x68);
for(int i=0;i<=7;i++)
	{
	LCD_WriteData(znak6[i]);
	}
LCD_WriteCommand(0x80);


//LCD_WriteCommand(0x40);
//
//for(int i=0;i<=7;i++)
//	{
//	LCD_WriteData(znak2[i]);
//	}
//LCD_WriteCommand(0x80);
//BUZER_ON;
//_delay_ms(500);
//BUZER_OFF;
//int0
DDRD &=~(1<<PD2);
PORTD |=(1<<PD2);
MCUCR |= (1<<ISC01);
GICR |=(1<<INT0);
sei();
/////////////
uint8_t test =0;
 test = GetReg(STATUS);
 nrf_init_TX();
 NRF_reset();
 uint8_t bufor[5];
 bufor[0] = 10;
 bufor[1] = 10;
 bufor[2] = 10;
 bufor[3] = 10;
 bufor[4] = 10;

 CE_H;
// Ekran powitalny
 lcd_led_time = 30;
 LCD_WriteData(0x01);
 LCD_WriteText("ENERGIZER CTRL");
 LCD_WriteData(0x01);
 LCD_GoTo(4,1);
 LCD_WriteText("by K@ROL");
//////////////////////////////////
 _delay_ms(3000);
 LCD_Clear();
	if(!(test == 0x0E))
	{
		LCD_WriteText("NRF ERROR");
		while(1);
	}

// odczyt progu wyzwalania z eeprom
	alarm_prog = eeprom_read_byte(&alarm_prog_eeprom);
	if(alarm_prog>5)alarm_prog=0; //jesli wartos progu spoza zakresu to ustaw prog na 0
	alarm_prog_new=alarm_prog;

//

 LCD_GoTo(9,0);
 LCD_WriteData(0x03); //wysw znak baterii

 LCD_GoTo(9,1);
 LCD_WriteData(0x02); // wysw znak term.

 LCD_GoTo(alarm_prog,1); //wyswietlanie znacznika
 LCD_WriteData(0x05);



while(1)
{


if((con_stat<20) & (w_flag==1))
{
	v_level = data[2];
	if(v_level != v_level_last)
	{
	LCD_GoTo(0,0);
	LCD_WriteText("         ");
	LCD_GoTo(0,0);
	for(int i=1;i<=v_level;i++)LCD_WriteData(0xff);
	for(int i=0;i<=(5-v_level);i++)LCD_WriteData(0x00);
	v_level_last=v_level;
	}

	if((v_level <(alarm_prog+1)) & (alarm_off==0))  //jesli poziom mniejszy od progu wyzwalania to odliczaj do alarmu
	{
		z_count++;
		if(z_count>5) alarm_flag=1; //jesli 5 imp z rzedu mniejszych od  progu to uruchom alarm
	}

	if(v_level>= (alarm_prog+1)) //jesli poziom wiekszy rowny progowi to reset licznika alarmu
	{
		alarm_off=0;
		z_count=0;
	}
	else if(alarm_off==1) // jeli alarm wy³¹czony ale pastuch dalej nie dzia³a to podswietlaj ekran
		{
		lcd_led_time=20;
		}

// Przeliczenie pomiaru nap bateri na %
	//temp_f=((2.54f/255.0f)*(float)data[0]*2.0f)-0.04f;
	temp_f = (((2.56f*(float)data[0]))/255.0f)*2;
	temp_f -= 3.6;
	temp_f = (temp_f/0.6)*100;
	if(temp_f>100.0)temp_f = 100;

	//temp_c=temp_f;
	//LCD_Int(temp_c);
	//temp_d=(temp_f*100.0f)-(float)temp_c*100.0;
	//temp_d1 = temp_d / 100;
	//temp_d = temp_d%100;

	temp_c=temp_f;
	// jesli inna wartosc niz poprzednia to wyswietl
	if(temp_c != last_V_mesure)
	{

	LCD_GoTo(11,0);
	LCD_WriteText("      ");
	LCD_GoTo(11,0);

	LCD_Int(temp_c);
	LCD_WriteText("%");

	//LCD_WriteText(".");
	//LCD_Int(temp_d1);
	//LCD_Int(temp_d);
	//LCD_WriteText("V");
	last_V_mesure = temp_c;
	}

	//sprawdzenie czy ³aduje
	if((data[1]==0) & (charging<50)) charging +=5;
	else if((data[1]==1) & (charging>0)) charging--;
	if(charging >0)
	{
	LCD_GoTo(15,0);

			LCD_WriteData(0x01);

	}
	w_flag=0;
	con_flag=0;
	con_alarm_count=0;
	con_alarm_flag=0;
	con_alarm_off=0;
}

	else if((con_stat>=20)&(con_flag==0))
	{

		LCD_GoTo(0,0);
		LCD_WriteText("Brak p");
		con_flag=1;
		//LCD_WriteText(0x01);
		v_level_last=10;		// zmiana starych wartoci ¿eby po przywruceniu po³¹czenia wywietli³y siê poprawne wartoci
		 last_V_mesure = 0;
	}



///pomiar i wyswietlanie temp
	if((temp_DS1820>15) & (flag_DS1820==0))
	{
		DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);
		temp_DS1820=0;
		flag_DS1820=1;

	}

	if(flag_DS1820==1)
	{
		if(DS18X20_OK == DS18X20_read_meas(gSensorIDs[0], &znak, &cal, &dzies))
		{

			flag_DS1820=0;

			if((znak != znak_l)|(cal != cal_l)|(dzies !=dzies_l))
			{
				wysw_temp();
				znak_l = znak;
				cal_l = cal;
				dzies_l = dzies;
			}
		}
	}

	if(al_p_blink_stat==1) // miganie znacznikiem
	{
		al_p_blink_stat=0;
		LCD_GoTo(0,1);
		LCD_WriteText("      ");
		LCD_GoTo(alarm_prog_new,1);
		if(al_p_blink_count<2)LCD_WriteData(0x05);
		else LCD_WriteData(0x20);

	}

	if(eeprom_save==1) // zapis progu do eeprom
	{
		eeprom_save=0;
		eeprom_busy_wait();
		eeprom_write_byte(&alarm_prog_eeprom,alarm_prog);
	}

}

}

ISR(TIMER0_OVF_vect)
{
	if(temp_DS1820<20)
	{
		temp_DS1820++;
	}

	if(SWITCH_DW)
	{
		LCD_LED_ON;
		lcd_led_time=30;
	}

	if(con_stat<25)
	{

	con_stat++;
	if(con_stat==20)lcd_led_time=20;
	}

	if(lcd_led_time>0)
	{

		lcd_led_time--;
		if(lcd_led_time>1)LCD_LED_ON;
		if (lcd_led_time==1) LCD_LED_OFF;
	}

	if((alarm_flag==1) | (con_alarm_flag==1))
	{
		BUZER_TOG;
		lcd_led_time=5;
	}
	else BUZER_OFF;

	if((alarm_flag==1)&(SWITCH_DW))
	{
		alarm_flag=0;
		alarm_off=1;
	}

	if(con_alarm_off==0)
			{
			con_alarm_count++;
			if(con_alarm_count>200) con_alarm_flag=1;
			}

	if((con_alarm_flag==1)&(SWITCH_DW))
		{
			con_alarm_flag=0;
			con_alarm_off=1;
		}



	if((alarm_flag==0)&(con_alarm_flag==0)&(al_p_change==0)& SWITCH_DW) // jesli brak alarmow i brak aktywnej zmiany ustawien to sprawdzaj czy uzytkownik chce wejsc w ustawienia (przytrzymanie przycisku)
	{
		chang_al_prog_count++;
		if(chang_al_prog_count>15)
		{
			al_p_change=1; // jesli przytrzymany switch to wejdz do ustawien
			disable_al_prog_count =20;
		}
	}else chang_al_prog_count=0;

	if(al_p_change==1) //jesli jestesmy w ustawieniach
		{
			al_p_blink_count++;
			if (al_p_blink_count>3)al_p_blink_count=0;
			al_p_blink_stat=1; //flaga od wyswietlania znaku na lcd w petli glownej
			if(SWITCH_DW ) //jesli miga i nacisnieto sw to zwieksz poziom
			{
				if(sw_relase==1)
				{
					sw_relase=0;
					alarm_prog_new++;
					if(alarm_prog_new>5)alarm_prog_new=0;
					disable_al_prog_count=20;//jesli wcisnieto sw to reset licznika wyjscia z ustawien
				}

			}else sw_relase=1;

			disable_al_prog_count--;
			if(disable_al_prog_count==0)//wyjscie z ustawien jesli dluzszy czas bezczynnosci
			{
				al_p_change=0;
				al_p_blink_count=0;//zerujemy licznik migania zeby miec pewnosc ze wskaznik bedzie wyswietlony po wyjsciu z ustawien

				alarm_prog=alarm_prog_new; //przypisanie nowej wartosci progu przy wyjsciu z ustawien
				eeprom_save=1; //zapis do eeprom
			}
		}



}

SIGNAL(INT0_vect)
{
	if(con_stat>20)
		{
		lcd_led_time=20; //jesli odzyskano po³¹czenie w³¹cz podswietlenie lcd
		//cal_l +=1;//¿eby wyswietli³ temperaturê
		}

	CE_L;


	LED_ON;
	data=NRF_Transmit(R,R_RX_PAYLOAD,data,5);


	w_flag = 1;

	NRF_reset();
	_delay_us(10);
	CE_H;
	con_stat=0; //zerowanie zmiennej sprawdzaj¹cej "stan" po³¹czenia;
	LED_OFF;



}
