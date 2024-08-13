#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint16_t adc_val=0;
int voltage=0;

ISR(ADC_vect){
    adc_val=ADC;
    voltage= (float)adc_val/(float)1024*5000;
    if(voltage>=1600) PORTB=0x08;
    else PORTB=0x02;
}

ISR (INT0_vect)
{
    EIFR=0xFF;
    _delay_ms(3);
    if(EIFR!=0xFF){
        PORTB+=0x20;
        _delay_ms(1000);
        ADCSRA|= (1<<ADSC);
    }
}
int main() {
EICRA =(1<< ISC00) | (1<< ISC01);
EIMSK = (1<< INT0);

ADMUX |= 0b01000000; //Right adjusted, ADC3
ADCSRA |= 0b10001111;   //128 Prescaler and ASIE=1
sei();

DDRB=0xFF; //PORTB is output
PORTB=0x00;
while(1){
}
}
