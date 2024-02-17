#include "frdm_bsp.h" 
#include "lcd1602.h"
#include "leds.h" 
#include "stdio.h"
#include <stdlib.h>
#include "math.h"
#include "i2c.h"
#include "uart0.h"
#define	ZYXDR_Mask	1<<3	// Maska bitu ZYXDR w rejestrze STATUS
#define table_size 10
static uint8_t arrayXYZ[6];
static uint8_t sens;
static uint8_t status;
int X, Y, Z;
uint32_t baud_rate, i=0;
int licznik_czasu = 0;
int sum_of_number_of_steps;
uint8_t step_start = 1;
uint8_t step_end = 0;
uint16_t number_of_steps = 0;


char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};

//zmienne do usredniania
int sum=0;


long accelerometer_samples[table_size];
long derivative_of_accelerometer_samples[table_size-1];
int systick_interrupt=0;

void SysTick_Handler(void)	// Podprogram obslugi przerwania od SysTick'a
{ 
	systick_interrupt = 1;
}


void pedometer(){
	for( int i=0;i<table_size-1;i++){
		//szukanie wartosci max dla poczatku kroku jezeli step_start = 1
		if(step_start > 0){
			if(accelerometer_samples[i] >2000){ // jezeli wartosci zaczynaja malec to znaczy ze maximum wystapilo
				step_start = 0;
				step_end = 1;
				}
			}
		// szukanie wartosci min dla koncu kroku jezeli step_end = 1
		if(step_end > 0){
			if(accelerometer_samples[i] < 1900){ // jezeli wartosci zaczynaja rosnac to znaczy ze minimum wystapilo
				step_end = 0;
				step_start=1;
				sum_of_number_of_steps++;
				}
			}
	}
}

  static int number_of_sample=0; // deklaracja tylko przy pierwszym odpaleniu

void data_aquisition(){
	accelerometer_samples[number_of_sample] = abs(X)+abs(Y)+abs(Z); //modul
	number_of_sample++;

	if( table_size == number_of_sample){ //ustawia tablice tak, zeby zapisywalo dane od poczatku do konca
		number_of_sample = 0;
		pedometer();	
		sprintf(display,"%d\n\r",(int)accelerometer_samples[number_of_sample])	;
					for(i=0;display[i]!=0;i++)
						{
							while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy nadajnik jest pusty?
							UART0->D = display[i];		// Wyslij aktualna wartosc licznika
						}
		LCD1602_SetCursor(0,0);
		sprintf(display,"Steps: %d", sum_of_number_of_steps);
		LCD1602_Print(display);
	}
}

int main (void)
{ 
	
	
	LED_Init();
	LCD1602_Init();	// Tu jest równiez inicjalizacja portu I2C0
	LCD1602_Backlight(TRUE);

	sens=0;	// Wybór czulosci: 0 - 2g; 1 - 4g; 2 - 8g
	I2C_WriteReg(0x1d, 0x2a, 0x0);	// ACTIVE=0 - stan czuwania
	I2C_WriteReg(0x1d, 0xe, sens);	 		// Ustaw czulosc zgodnie ze zmienna sens
	I2C_WriteReg(0x1d, 0x2a, 0x1);	 		// ACTIVE=1 - stan aktywny

	UART0_Init();
	
	UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );	
	
		UART0->BDH = 0;			//Dla CLOCK_SETUP=0 BR=28800
		UART0->BDL =91;			//Dla CLOCK_SETUP=0 BR=28800
		baud_rate=28800; 
	
	UART0->C2 |= UART0_C2_TE_MASK;		//Wlacz nadajnik
	
	SysTick_Config(SystemCoreClock/10);	// Start licznika SysTick ( przerwanie co 10ms -- 100 próbek na sekunde)
	while(1)
	{
		// funkcja odczytuje dane co 10ms
		if(systick_interrupt == 1){
				I2C_ReadReg(0x1d, 0x0, &status);
				status&=ZYXDR_Mask;
				if(status)	// Sprawdzam czy dane sa gotowe do odczytu
				{
						I2C_ReadRegBlock(0x1d, 0x1, 6, arrayXYZ);
						X=(1000*((int16_t)((arrayXYZ[0]<<8)|arrayXYZ[1])>>2)/(4096>>sens));
						Y=(1000*((int16_t)((arrayXYZ[2]<<8)|arrayXYZ[3])>>2)/(4096>>sens));
						Z=(1000*((int16_t)((arrayXYZ[4]<<8)|arrayXYZ[5])>>2)/(4096>>sens)); // aby wartosci nie byly zmiennoprzecinkowe xyz sa intami
						
						
						data_aquisition();
						if (licznik_czasu == 0) //Zlicza nam 15 sekund w dól, pózniej ilosc kroków zebrana przez 15 sekund jest pomnozona przez 4 i w ten sposób mamy kroki na minute
						{
							licznik_czasu = 150;
							LCD1602_SetCursor(0,1);
							sprintf(display,"Steps/min: %d ", 4*(sum_of_number_of_steps - number_of_steps));
							LCD1602_Print(display);
							number_of_steps = sum_of_number_of_steps;
							
						}else
							licznik_czasu --;
						
				}
			
			  systick_interrupt = 0;
		}
		
	}

}
