/*
    Firmware for SUN tactile sensors

    Copyright 2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <16F690.H>

#define CLOCK_SP 8000000
#fuses HS, NOWDT, NOPROTECT
#define TxD PIN_B7
#define RxD PIN_B5

#device ADC=10
#use delay (internal=CLOCK_SP)
#use rs232(stream=pc,baud=57600,parity=N,xmit=TxD,rcv=RxD,stop=1,BITS=8)


char a, low, high;
int16 value; 
#INT_TIMER0
void isr_timer(){
output_bit(PIN_C2,1);
set_timer0(178);			
set_adc_channel(5);
                   	delay_us(10);
                   	value=read_adc();
                   	delay_us(30);
                   	low=value;
                  	high=value>>8;
                  	fputc(low,pc);
                  	fputc(high,pc);
output_bit(PIN_C2,0);
}

void main (void){  
   	setup_adc(ADC_CLOCK_DIV_16);
    
    //Setto le porte analogiche    
    setup_adc_ports(NO_ANALOGS);         
    setup_adc_ports(sAN5);
a=0;


 //Inizializzo la trasmissione SPI
	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_256|RTCC_8_BIT);
start:  
      //Controllo se arriva il carattere di START
      while(a!='a'){
         if(kbhit()){
            a=fgetc(pc);
         }
      }  
set_timer0(178);
enable_interrupts(INT_TIMER0);
enable_interrupts(GLOBAL);
    
   while(1){
		if(kbhit()){
            a=fgetc(pc);
			if(a=='b'){
disable_interrupts(INT_TIMER0);
disable_interrupts(GLOBAL);
goto start;
   }      
}

   }
}      
