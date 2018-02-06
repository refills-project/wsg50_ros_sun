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

#include <16F1824.H>

//////// Fuses: LP,XT,HS,RC,INTRC_IO,ECL,ECM,ECH,NOWDT,WDT_SW,WDT_NOSL,WDT
//////// Fuses: PUT,NOPUT,NOMCLR,MCLR,PROTECT,NOPROTECT,CPD,NOCPD,NOBROWNOUT
//////// Fuses: BROWNOUT_SW,BROWNOUT_NOSL,BROWNOUT,CLKOUT,NOCLKOUT,NOIESO
//////// Fuses: IESO,NOFCMEN,FCMEN,WRT,WRT_800,WRT_200,NOWRT,PLL_SW,PLL
//////// Fuses: NOSTVREN,STVREN,BORV25,BORV19,DEBUG,NODEBUG,NOLVP,LVP

#FUSES NOWDT      //No Watch Dog Timer 
#FUSES INTRC_IO   //Internal RC Osc, no CLKOUT 
#FUSES NOCPD      //No EE protection 
#FUSES NOPROTECT  //Code not protected from reading 
#FUSES NOMCLR     //Master Clear pin enabled 
#FUSES NOPUT      //Power Up Timer 
#FUSES BROWNOUT   //Reset when brownout detected 
#FUSES NOIESO     //Internal External Switch Over mode disabled 
#FUSES NOFCMEN    //Fail-safe clock monitor enabled 
#FUSES NODEBUG      //Debug mode for ICD              
#FUSES NOWRT      //Program Memory Write Protected 
#FUSES PLL_SW     // PLL under software control, disabled 
#FUSES NOLVP      //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O 
#FUSES BORV25     //Brownout reset at 2.5V 
#FUSES NOCLKOUT   //Output clock on OSC2 

#define CLOCK_SP 32000000
#define TxD PIN_C4
#define RxD PIN_C5
#define CS1 PIN_A0
#define CS2 PIN_A1
#define CLK_PIN PIN_A2 
#define blink PIN_A5

#use delay (clock=CLOCK_SP)
#use rs232(stream=pc,baud=115200,parity=N,xmit=TxD,rcv=RxD,stop=1,BITS=8)

// Start/Stop command
char a, i;

// Sensor data
char CHdata[50];

// Read an ADC channel
// chipSelect : the digital I/O where the ADC CS is attached to
// nADC : the number of the ADC chip (1,2)
// CH : the CH to read
// nextCH : the next CH to select for the conversion
// CHdata : a pointer to the data variable
void readADCch(char nADC, char CH, char nextCH, char *CHdata) {
    
    char data1 = 0b10000011, data2 = 0b01110000;

    // Select the CS pin
    int chipSelect = 0;
    if (nADC == 1) {
      chipSelect = CS1;
    }
    else if (nADC == 2) {
      chipSelect = CS2;
    }

    // Compute the index for data variable
    int index = (CH + (nADC - 1) * 16) * 2;

    // take the chip select low to select the device:
    output_bit(chipSelect,0); 
    
    CHdata[index + 1] = spi_read(data1 | (nextCH << 2));
    CHdata[index] = spi_read(data2);

    // take the chip select high to de-select:
     output_bit(chipSelect, 1);

    // Remove che CH number from the data
    CHdata[index + 1] &= 0b00001111;
    
    // Delay for proper A/D acquisition
    delay_us(20);
}

void leggi_tattile(){
    
    output_bit(CLK_PIN,1);
  
    readADCch(1, 0, 1, CHdata);
    readADCch(1, 1, 2, CHdata);
    readADCch(1, 2, 3, CHdata);
    readADCch(1, 3, 4, CHdata);
    readADCch(1, 4, 5, CHdata);
    readADCch(1, 5, 6, CHdata);
    readADCch(1, 6, 7, CHdata);
    readADCch(1, 7, 8, CHdata);
    readADCch(1, 8, 9, CHdata);
    readADCch(1, 9, 10, CHdata);
    readADCch(1, 10, 11, CHdata);
    readADCch(1, 11, 12, CHdata);
    readADCch(1, 12, 13, CHdata);
    readADCch(1, 13, 14, CHdata);
    readADCch(1, 14, 15, CHdata);
    readADCch(1, 15, 0, CHdata);
    readADCch(2, 0, 1, CHdata);
    readADCch(2, 1, 2, CHdata);
    readADCch(2, 2, 3, CHdata);
    readADCch(2, 3, 4, CHdata);
    readADCch(2, 4, 8, CHdata);
    readADCch(2, 5, 7, CHdata);
    readADCch(2, 6, 6, CHdata);
    readADCch(2, 7, 5, CHdata);
    readADCch(2, 8, 0, CHdata);
    
    // Send data over USART
    i = 0;
    for(i = 0; i < 50; i++) {
        fputc(CHdata[i], pc);
    }
    //fputc('c',pc);

    output_bit(CLK_PIN,0);
}

void main (void){  
    a=0;
    
    //Inizializzo la trasmissione SPI
	setup_spi(SPI_MASTER | SPI_L_TO_H | SPI_CLK_DIV_4);

    //Inizializzo la comunicazione con l'AD7490, settando il Control Register per la prossima conversione   
    readADCch(1, 0, 0, CHdata);
    readADCch(2, 0, 0, CHdata);  
    
    //Ciclo di lettura: 'z' per inizializzazione, 'a' per richiesta di lettura tattile  
    while(1){
//output_bit(blink,1);
//delay_ms(1000);
//output_bit(blink,0);
//delay_ms(1000);
//output_bit(blink,1);

		if(kbhit()){
            a=fgetc(pc);

			if(a=='z')
                fputc('x',pc);
            else if(a=='a') 
                leggi_tattile();
        }   
    }
}      
