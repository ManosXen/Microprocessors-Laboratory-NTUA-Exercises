.org 0x00
rjmp reset
.org 0x004
rjmp ISR1
    
    
reset:
    ldi r16, 0
 
;.include "m328PBdef.inc"
.equ FOSC_MHZ=16
.equ DEL_mS=500
.equ DEL_NU=FOSC_MHZ*DEL_mS
.def interrupts=r16
    
;Init Stack Pointer
    ldi r24, LOW (RAMEND)
    out SPL, r24
    ldi r24, HIGH (RAMEND)
    out SPH, r24
    
;Init PORTD as input
    clr r16
    out DDRD, r16

;Init PORTC as output
    ldi r26, 0b00001111
    out DDRC, r26
   
;Setting interrupt INT1
    ldi r26, (1 << ISC11) | (1 << ISC10)
    sts EICRA, r26
    
    ldi r26, (1<<INT1)
    out EIMSK, r26
    
    sei

loop1:
    clr r26
loop2:
    out PORTB, r26
    
    ldi r24, low (DEL_NU)
    ldi r25, high (DEL_NU) 
    rcall delay_ms
    
    inc r26
    
    cpi r26, 16
    breq loop1
    rjmp loop2
    
delay_mS:
    
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    sbiw r24,1 brne delay_ms
    ret
 
ISR1:
    in r17, SREG
    push r17
    sbic PIND, 6
    reti
    inc interrupts
    out PORTC, interrupts
    reti


