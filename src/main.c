/*
MULTIMETER
Author: SPRO2 GROUP 7
*/

//Include necessary libraries
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"
#include <stdint.h>

// include LCD libraries
#include "ssd1306.h"

//Definitions & Variables
#define ADC_PIN_capacitor     PC0    //ADC channel for CAPACITANCE  
#define ADC_PIN_resistor      PC1    //ADC channel for RESISTANCE & VOLTAGE
#define chargePin             PD5 
#define dischargePin          PD3       

#define RESISTANCE_MODE_PIN   PC2 
#define CAPACITANCE_MODE_PIN  PC3 
#define VOLTAGE_MODE_PIN      ADC6

#define resistorValue 10000.0F //10k resistor

float microFarads;
volatile uint32_t ms_counter = 0; //millisecond counter
uint16_t elapsedTime;
uint16_t analogVoltage;
float buffer = 0.00;
int Vin = 5.00; //5V applied by Arduinno
float Vout = 0.00; //VOut in Volts
float Rref= 10.00; // Set this values to the value of the used resistor in K ohms
float R2 = 0.00; //Unknown resistor set to 0

float activate_Resistance;
float activate_Capacitance;
float activate_Voltage;

//Function prototypes
void initADC();
uint16_t ADC_read(uint8_t adc_channel);
uint32_t millis(void);
void initUSART();
void USART_Transmit(char data);
void initTimer1(void);
void DisplayMicroFarads(float capacitance);
void DisplayFloatResistance(float resistance2);
void DisplayFloatVoltage(float voltage);

void setup(){

  PORTC |= (1<<PORTC5) | (1<<PORTC4); //LCD C4 & C5
  PORTC |= (1<<PORTC0) | (1<<PORTC2) | (1<<PORTC3); //ENABLE pins C0,C2,C3 (C1 (A1) DISABLED)

  //ADC registers
  ADMUX = (1<<MUX0); //A1 pin for analog input (ADC1)
            //first 00 REFS1 REFS0 for Voltage reference; AREF internal Vrot turned ofF
  ADCSRA = (1<<ADEN); //ENABLE ADC  //second BIT 6 is set to 1 to start conversion

}

int main(void) {

  initUSART();
  initADC();
  initTimer1();
  sei(); 

  // LCD INIT
  SSD1306_Init (SSD1306_ADDR); // 0X3C

  DDRD |= (1<<DDD2) | (1<<DDD6); //D2 AND D6 as OUTPUT
  PORTD |= (1<<PORTD2); //D2 as HIGH
  DDRD |= (1 << chargePin); //chargePin OUTPUT
  DDRD &= ~(1 << dischargePin); //dischargePin INPUT

  while (1) {

    activate_Resistance = ADC_read(RESISTANCE_MODE_PIN);
    activate_Capacitance = ADC_read(CAPACITANCE_MODE_PIN);
    //activate_Voltage = ADC_read(VOLTAGE_MODE_PIN);

    if (activate_Capacitance != 0){

      PORTD |= (1 << chargePin); //chargePin HIGH

      uint32_t startTime = millis();
      while (ADC_read(ADC_PIN_capacitor) < 648); //wait until capacitor reaches value
      elapsedTime = millis() - startTime;
      microFarads = ((float)elapsedTime / resistorValue) * 1000.0;

      //printLong(elapsedTime);
      // printString(" mS    ");

      SSD1306_ClearScreen(); 

      SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
      SSD1306_DrawString ("CAPACITANCE");
      SSD1306_UpdateScreen (SSD1306_ADDR);  // update

      if (microFarads > 0){

        SSD1306_SetPosition (0, 1); 
        DisplayMicroFarads(microFarads);

        SSD1306_UpdateScreen (SSD1306_ADDR);  // update
      }
      if (microFarads == 0){

        SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
        SSD1306_DrawString ("CAPACITANCE");
        SSD1306_SetPosition (0, 0); 
        SSD1306_DrawString ("Insert");
        SSD1306_SetPosition (0, 1); 
        SSD1306_DrawString ("Capacitor");
        SSD1306_UpdateScreen (SSD1306_ADDR);  // update

      } 
    }

    if(activate_Resistance != 0){

      analogVoltage = ADC_read(ADC_PIN_resistor); // Read analog Voltage

      //convert to Volts
      buffer = analogVoltage * Vin;
      Vout = (buffer)/ 1024.00;

      buffer = (Vin/Vout) -1;
      R2 = (Rref * buffer*1000) - 30; // *1000 because we express it in ohms / -30 due to tolerances
      if (R2 < 65){ // to be able to read 10 ohms resistors precisely
        R2 = 10;
      }

      SSD1306_ClearScreen(); 

      SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
      SSD1306_DrawString ("RESISTANCE");
      SSD1306_UpdateScreen (SSD1306_ADDR);  // update

        SSD1306_SetPosition (0, 1); 
        DisplayFloatResistance(R2); //use function to print value of RESISTANCE in the OLED with Ohms

        SSD1306_UpdateScreen (SSD1306_ADDR);  // update

      if (R2 < Rref*1000){ //MEASURE RESISTOR

        SSD1306_SetPosition (0, 1); 
        DisplayFloatResistance(R2); //use function to print value of RESISTANCE in the OLED with Ohms

        SSD1306_UpdateScreen (SSD1306_ADDR);  // update

      }

      if (R2 > Rref*1000){  //NO RESISTOR

        SSD1306_ClearScreen(); 

        SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
        SSD1306_DrawString ("RESISTANCE");
        SSD1306_SetPosition (0, 0); 
        SSD1306_DrawString ("Insert");
        SSD1306_SetPosition (0, 1); 
        SSD1306_DrawString ("Resistor");

        SSD1306_UpdateScreen (SSD1306_ADDR);  // update

      }
    }


    if (activate_Voltage != 0){
      
      SSD1306_ClearScreen(); 
      SSD1306_SetPosition (15, 3);   // center position TITLE (yellow)
      SSD1306_DrawString ("VOLTAGE");
      SSD1306_UpdateScreen (SSD1306_ADDR);  // update 

      SSD1306_SetPosition (0, 0); 
      DisplayFloatVoltage(Vout);

      SSD1306_UpdateScreen (SSD1306_ADDR);  // update

    }

      PORTD &= ~(1 << chargePin); //chargePin LOW
      DDRD |= (1 << dischargePin); //dischargePin OUTPUT
      PORTD &= ~(1 << dischargePin); //dischargePin LOW

      while (ADC_read(ADC_PIN_capacitor) > 0);//wait until capacitor discharges

      DDRD &= ~(1 << dischargePin); //dischargePin INPUT


      _delay_ms(1000);

  } //WHILE

} //MAIN




void initADC() {
  ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
  ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC and set prescaler to 8
}

//ADC function
uint16_t ADC_read(uint8_t ADC_channel){
  ADMUX &= 0xf0; // clear previously used channel, but keep internal reference
  ADMUX |= ADC_channel; // set the desired channel 
  ADCSRA |= (1<<ADSC);  //start a conversion
  while ( (ADCSRA & (1<<ADSC)) ); //wait for conversion to complete

  return ADC; //return to the calling function as a 16 bit unsigned int
}

void initUSART() {
    //set baud rate
    uint16_t ubrr = 103; //9600 baud for 16MHz
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); //enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); //set frame format: 8 data bits, 1 stop bit
}

void USART_Transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0))); //wait for empty transmit buffer
    UDR0 = data; //put data into buffer, sends the data
}

void initTimer1(void) {
    TCCR1B |= (1 << WGM12); //configure timer 1 for CTC mode
    TIMSK1 |= (1 << OCIE1A); //enable CTC interrupt
    OCR1A = 249; //16MHz / 64 prescaler / 1000 - 1 = 249
    TCCR1B |= (1 << CS11) | (1 << CS10); //start timer at Fcpu/64
}

ISR(TIMER1_COMPA_vect) {
    ms_counter++; //increment ms counter
}

uint32_t millis(void) {
    uint32_t millis_copy;
    cli(); 
    millis_copy = ms_counter; 
    sei(); 
    return millis_copy;
}

void DisplayMicroFarads(float capacitance) {

  char buffer[20]; //buffer to hold the formatted string
  sprintf(buffer, "%.3f uF", capacitance); // to format str of the capacitance value

  SSD1306_DrawString (buffer); //print capacitance value in OLED 
}

//function iused to print variables in the OLED with units, not only the number
void DisplayFloatResistance(float resistance2) {

  char buffer[20]; //buffer to hold the formatted string
  sprintf(buffer, "%.3f Ohms", resistance2); // to format str of the R2 value

  SSD1306_DrawString (buffer); //print R2 value in OLED 
}

// same function for Volts
void DisplayFloatVoltage(float voltage) {

  char buffer[20]; //buffer to hold the formatted string
  sprintf(buffer, "%.3f Volts", voltage); // to format str of the R2 value

  SSD1306_DrawString (buffer); //print R2 value in OLED 
}
