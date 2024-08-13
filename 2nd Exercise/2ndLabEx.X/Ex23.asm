.org 0x00
rjmp reset
.org 0x04
rjmp ISR1
    
    
reset:
 
;.include "m328PBdef.inc"
.equ FOSC_MHZ=16
.equ DEL_mS=3000
.equ DEL_NU=FOSC_MHZ*DEL_mS
    
.equ DEL_Inter=10
.equ DEL_INT=FOSC_MHZ*DEL_Inter
    
.def lights_on = r22
    
;Init Stack Pointer
    ldi r24, LOW (RAMEND)
    out SPL, r24
    ldi r24, HIGH (RAMEND)
    out SPH, r24
    

;Init PORTB as output
    ldi r26, 0b11111111
    out DDR?, r26    
    
    
;Setting interrupt INT1
    ldi r26, (1 << ISC11) | (1 << ISC10)
    sts EICRA, r26
    
    ldi r26, (1<<INT0)
    out EIMSK, r26
    
    sei
    
loop:
    rjmp loop 
    
    
delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    sbiw r24,1 
    brne delay_ms
    ret
    
    
    
ISR1:
    in r17, SREG
    push r17
    
    debounce:   ;Debounce handling
	ldi r24, (1<<INTF1)
	out EIFR, r24
	ldi r24, low (DEL_INT)
	ldi r25, high (DEL_INT) 
	rcall delay_mS
	in r17, EIFR
	sbrc r17, 1       ;Swsto bit?
	rjmp exit
    
    sbrs lights_on, 0
    rjmp on
    
    already_open:
	ldi r24, low (500)
	ldi r25, high (500)
	ser r18
	out PORTB, r18
	rcall delay_mS
	ldi r18, 1
	out PORTB, r18
    
    on:
	ldi lights_on, 1
	ldi r24, low (DEL_NU)
	ldi r25, high (DEL_NU)
        ldi r18, 1
	out PORTB, r18
	rcall delay_mS
    
    off:
	ldi r18, 0
	out PORTB, r18
	ldi lights_on, 0
    
    pop r17
    exit:
	pop r25
	pop r24
	pop r17
	out SREG, r17
	reti








