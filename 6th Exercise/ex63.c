#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L
//A0=A1=A2=0 by hardware
// reading from twi device
// writing to twi device
// twi clock in Hz
//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2
// PCA9555 REGISTERS
typedef enum {
REG_INPUT_0 =0,
REG_INPUT_1 =1,
REG_OUTPUT_0 =2,
REG_OUTPUT_1 =3,
REG_POLARITY_INV_0 =4,
REG_POLARITY_INV_1 =5,
REG_CONFIGURATION_0 =6,
REG_CONFIGURATION_1 =7
} PCA9555_REGISTERS;


//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58

#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)




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

void lcd_string(const char *str){
    int i;
    for(i=0; str[i]!=0; i++) lcd_data(str[i]);
}


//initialize TWI clock
void twi_init(void){
    
    TWSR0 = 0; // PRESCALER_VALUE=1
    TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}

// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void){

    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while(!(TWCR0 & (1<<TWINT)));
    return TWDR0;

}

//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void){
        
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR0 & (1<<TWINT)));
    return TWDR0;
    
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address){

    uint8_t twi_status; // send START condition
    TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); 
    // wait until transmission completed
    while(!(TWCR0 & (1<<TWINT))); 

    // check value of TWI Status Register.
    twi_status = TW_STATUS & 0xF8;
    if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
    
    // send device address
    TWDR0 = address;
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    // wail until transmission completed and ACK/NACK has been received
    while(!(TWCR0 & (1<<TWINT)));

    // wail until transmission completed and ACK/NACK has been received
    while(!(TWCR0 & (1<<TWINT)));

    // check value of TWI Status Register.
    twi_status = TW_STATUS & 0xF8;
    if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) ) return 1;
    return 0;
}

// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address){
    uint8_t twi_status;
    while ( 1 ){
        // send START condition
        TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

        // wait until transmission completed
        while(!(TWCR0 & (1<<TWINT)));
        
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
        
        // send device address
        TWDR0 = address;
        TWCR0 = (1<<TWINT) | (1<<TWEN);
        
        // wail until transmission completed
        while(!(TWCR0 & (1<<TWINT)));
        
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) ){
            /* device busy, send stop condition to terminate write operation */
            TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
            // wait until stop condition is executed and bus released
            while(TWCR0 & (1<<TWSTO));
            continue;
        }
        break;
    }
}

// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data ){
    // send data to the previously addressed device
    TWDR0 = data;
    TWCR0 = (1<<TWINT) | (1<<TWEN);

    // wait until transmission completed
    while(!(TWCR0 & (1<<TWINT)));
    if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
    return 0;
}

// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
//        1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
return twi_start( address );
}

// Terminates the data transfer and releases the twi bus
void twi_stop(void){

    // send stop condition
    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

    // wait until stop condition is executed and bus released
    while(TWCR0 & (1<<TWSTO));
}

void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value){

    twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
    twi_write(reg);
    twi_write(value);
    twi_stop();

}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg){
    uint8_t ret_val;
    twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
    twi_write(reg);
    twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
    ret_val = twi_readNak();
    twi_stop();
    return ret_val;
}

//function to get the number of row to check if a button is pressed. Returns an 8 bit int with the buttons that were pressed.
uint8_t scan_row(uint8_t row){
    uint8_t check=1;
    check = check << row;
    PCA9555_0_write(REG_OUTPUT_1, ~check);
    uint8_t result = PCA9555_0_read(REG_INPUT_1)>>4;
    return result;
}

//function to check the entire 4x4 keypad. Returns an 16 bit int with the buttons that were pressed.
//The formation of the result is r0_r1_r2_r3 where its r0 is 4 bits IO1_7, IO1_6, IO_5, IO_4
uint16_t scan_keypad(){
    uint16_t result;
    result= (scan_row(0)<<12) + (scan_row(1)<<8) + (scan_row(2)<<4) +scan_row(3);
    return result;
}

uint16_t previous_result=0xFF;

uint16_t scan_keyboard_rising_edge(){
uint16_t result0 = scan_keypad();
_delay_ms(15);
uint16_t result1 = scan_keypad();
uint16_t same = (~result0 & ~result1);
previous_result=~same;
return same;
}


uint8_t keypad_to_ascii(){
    
        switch(scan_keyboard_rising_edge()) {
        case 0: return 0;
        case 1: return '1';
        case 2: return '2';
        case 4: return '3';
        case 8: return 'A';
        case 16: return '4';
        case 32: return '5';
        case 64: return '6';
        case 128: return 'B';
        case 256: return '7';
        case 512: return '8';
        case 1024: return '9';
        case 2048: return 'C';
        case 4096: return '*';
        case 8192: return '0';
        case 16384: return '#';
        case 32768: return 'D';
    }
        return 0;
}

uint8_t previous_call=0;

//function to get the digit from keypad. Prevents the microcontroller to get multiple digits when one button is tapped continiously
uint16_t getDigit(){
	
	uint8_t keypad=keypad_to_ascii();
	uint8_t final_result;
	if(keypad!=previous_call) final_result = keypad;
	else final_result=0;
	previous_call=keypad;
	return final_result;
}

int main(){

    twi_init();
    
    DDRD |= 0xFF;
    DDRB =0xFF;
    
    lcd_init();

    PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); //Set EXT_PORT1[7:4] as input and EXT_PORT[3:0] as output

    uint8_t input;
    char code[2];

    while(1){
        PORTB=0;
        uint8_t j=0;
        lcd_clear_display();
        while(j<2){
            input=getDigit();
            if(input!=0){
                lcd_data('*');
                code[j]=input;
                j++;
            }
        }
        lcd_clear_display();
        if(code[0] == '4' && code[1]=='5'){
            PORTB=0xFF;
            lcd_string("Right Password");
            _delay_ms(4000);
            PORTB=0x0;
            _delay_ms(1000);
        }else{
            lcd_string("Wrong Password");
            uint8_t i=0;
            while(i<10){
                PORTB=0xFF;
                _delay_ms(250);
                PORTB=0;
                _delay_ms(250);
                i++;
            }
        }
    }
}
