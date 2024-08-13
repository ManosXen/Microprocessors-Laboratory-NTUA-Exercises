;.include "m328PBdef.inc"
.org 0x00
rjmp reset
    
    
    
reset:
    .def temp=r19 
    .def duty=r18
    .equ FOSC_MHZ=16
    .equ DEL_mS=100
    .equ DEL_NU=FOSC_MHZ*DEL_mS
    
    ldi temp, LOW(RAMEND)
    out SPL, temp
    ldi temp,HIGH(RAMEND)
    out SPH, temp
    
	;2% 5.1Hz aprox 5    
    Table:
	.dw 5, 10, 15, 20, 26
	.dw 31, 36, 41, 46, 51
	.dw 56, 61, 66, 71, 77
	.dw 82, 87, 92, 97, 102
	.dw 107, 112, 117, 122, 128
	.dw 133, 138, 143, 148, 153
	.dw 158, 163, 168, 173, 179
	.dw 184, 189, 194, 199, 204
	.dw 209, 214, 219, 224, 230
	.dw 235, 240, 245, 250
	
    ldi temp, (1<<TOIE1)
    sts TIMSK1, temp
    sei
    ldi temp, (1<<WGM10) | (1<<COM1A1)
    sts TCCR1A, temp
    ldi temp, (1<<WGM12) | (1<<CS10)
    sts TCCR1B, temp
    
    ;setting PORTB as output
    ldi temp, 0b00111111
    out DDRB, temp
    
    ;setting PORTD as input
    clr temp
    out DDRD, temp
    
    ;Moving 50% duty cycle to duty variable and setting output
    ldi zh, high(Table*2)
    ldi zl, low(Table*2)
    adiw zl, 48
    lpm
    mov duty, r0
    sts OCR1AL, duty
    
    loop:
	check_increase:
	    sbis PIND, 1
	    rjmp check_decrease

	    cpi duty, 250            ;If duty is already 98% then no increase
	    breq wait_for_zero_PD1

	    adiw zl, 2		 ;If duty is lower than 98% then we increase the table pointer and retrieve value to duty and OCR1AL
	    lpm
	    mov duty, r0
	    sts OCR1AL, duty

	    wait_for_zero_PD1:       ;Wait until PD1 is zero
		sbic PIND, 1
		rjmp wait_for_zero_PD1
	
	check_decrease:
	    sbis PIND, 2
	    rjmp wait

	    cpi duty, 5            ;If duty is already 2% then no decrease
	    breq wait_for_zero_PD1

	    sbiw zl, 2		 ;If duty is lower than 2% then we decrease the table pointer and retrieve value to duty and OCR1AL
	    lpm
	    mov duty, r0
	    sts OCR1AL, duty

	    wait_for_zero_PD2:       ;Wait until PD1 is zero
		sbic PIND, 2
		rjmp wait_for_zero_PD2
		
	wait:
	    ldi r24, low(DEL_NU)
	    ldi r25, high(DEL_NU)
	    rcall wait_x_msec
	rjmp loop
	    
	    
	    
	    
	    
	    
wait_x_msec:
    
    ;init will consume 248*4+3=996 cycles
    init: ldi r23, 249

    loop1:
	subi r23, 1
	nop
	brne loop1

    sbiw r24, 1 ;2 cycles 
    brne init 
    
ret    
    
    
    
    
    


