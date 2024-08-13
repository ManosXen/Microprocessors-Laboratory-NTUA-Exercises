#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void write_2_nibbles(uint8_t c);
void lcd_clear_display();
void lcd_command(uint8_t com);
void lcd_data(unsigned char data);
void lcd_init();
void lcd_string(char *str);


uint16_t gas=0;  //Stores ADC voltage result
uint8_t output=0;

ISR(ADC_vect){		
	gas=ADC;
    if(gas>143){
        if(gas<293) output=1;
        else if (gas<438) output=3;
        else if (gas<584) output=7;
        else if (gas<729) output=15;
        else if (gas<874) output=31;
        else  output=63;
    }
}

int main()
{
   
    DDRB |= 0xFF;
    DDRD |= 0xFF;
    
           
    ADMUX |= 0b01000011; //Right adjusted, ADC3
    ADCSRA |= 0b10001111;   //128 Prescaler and ASIE=1
           
    lcd_init();
    sei();
    ADCSRA|= (1<<ADSC);    //Start ADC

    while (1)
    {
        if(gas>143){
            lcd_clear_display();
            char message[]="GAS DETECTED";
            lcd_string(message);
            
            //else output=64;
            while(gas>143){
                int i;
                PORTB=output;
                for(i=0; i<5; i++){
                    _delay_ms(100);
                    if(!(gas>143)) break;
                    ADCSRA|= (1<<ADSC);
                }
                
                if(!(gas>143)) break;

                PORTB=0;
                for(i=0; i<5; i++){
                    _delay_ms(100);
                    if(!(gas>143)) break;
                    ADCSRA|= (1<<ADSC);
                }
            }

        }else{
            lcd_clear_display();
            PORTB=0;
            char message[]="CLEAR";
            lcd_string(message);
            while(!(gas>143)){
                _delay_ms(100);
                //if(gas) break;
                ADCSRA|= (1<<ADSC);
            }
        }
    }
}
   
void write_2_nibbles(uint8_t c){
    uint8_t temp= c;
    PORTD = (PIND & 0x0f) + (temp & 0xf0); //LCD Data High Bytes
    PORTD|=0x08;
    asm("nop");
    asm("nop");
    PORTD&=~(0x08);
   
    c=(c<<4)|(c>>4);
    PORTD = (PIND & 0x0f) + (c & 0xf0); //LCD Data Low Bytes
   
   PORTD|=0x08;
    asm("nop");
    asm("nop");
    PORTD&=~(0x08);
   
}

void lcd_clear_display(){
    lcd_command(0x01);
    _delay_ms(5);
}

void lcd_command(uint8_t com){
    PORTD&=~4; //LCD_RS=0 => Instruction
    write_2_nibbles(com);
    _delay_us(250);
}

void lcd_data(uint8_t  data){
    PORTD|=4; //LCD_RS=1 => Data
    write_2_nibbles(data);
    _delay_us(250);
}

void lcd_init(){
    _delay_ms(200);
    int i=0;
    while(i<3){      //command to switch to 8 bit mode
        PORTD=0x30;
        PORTD|=0x08;
        asm("nop");
        asm("nop");
        PORTD&=~0x08;
        _delay_us(250);
        ++i;
    }
   
    PORTD=0x20;     //command to switch to 4 bit mode
    PORTD|=0x08;
    _delay_us(2);
    PORTD&=~0x08;
    _delay_us(250);
   
    lcd_command(0x28); //5*8 dots, 2 lines
    lcd_command(0x0c); //display on, cursor off
   
    lcd_clear_display();
   
}

void lcd_string(char *str){
    int i;
    for(i=0; str[i]!=0; i++) lcd_data(str[i]);
}

