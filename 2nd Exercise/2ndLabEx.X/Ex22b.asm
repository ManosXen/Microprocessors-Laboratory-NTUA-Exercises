.org 0x00
rjmp reset
.org 0x02
rjmp ISR0
    
    
reset:
 
;.include "m328PBdef.inc"
.equ FOSC_MHZ=16
.equ DEL_mS=1000
.equ DEL_NU=FOSC_MHZ*DEL_mS
    
.equ DEL_Inter=10
.equ DEL_INT=FOSC_MHZ*DEL_Inter
    

;Init PORTC as output
    ser r26
    out DDRC, r26

;Init PORTB as input
    clr r26
    out DDRB, r26
    
;Init Stack Pointer
    ldi r24, LOW (RAMEND)
    out SPL, r24
    ldi r24, HIGH (RAMEND)
    out SPH, r24
    
;Setting interrupt INT0
    ldi r26, (1 << ISC01) | (1 << ISC00)
    sts EICRA, r26
    
    ldi r26, (1<<INT0)
    out EIMSK, r26
    
    sei
    
loop1:
    rjmp loop1
    
delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    sbiw r24,1 brne delay_ms
    ret
    
    
    
ISR0:
    in r17, SREG
    push r17
    push r24
    push r25
    
    debounce:   ;Debounce handling
	ldi r24, (1<<INTF0)
	out EIFR, r24
	ldi r24, low (DEL_INT)
	ldi r25, high (DEL_INT) 
	rcall delay_mS
	sbic EIFR, INTF0
	rjmp exit
    
    ldi r19, 0  ;counter
    ldi r18, 5
    in r17, PINB
    com r17
    count:
	ror r17
	brcc zero
	lsl r19
	inc r19
	zero: 
	    dec r18
	    brne count
    ;com r19
    out PORTC, r19
    exit:
	pop r25
	pop r24
	pop r17
	out SREG, r17
	reti
    
    












