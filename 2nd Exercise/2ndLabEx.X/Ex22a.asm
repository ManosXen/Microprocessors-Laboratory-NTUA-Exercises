.org 0x00
rjmp reset

    
    
reset:
 
;.include "m328PBdef.inc"
.equ FOSC_MHZ=16
.equ DEL_mS=1000
.equ DEL_NU=FOSC_MHZ*DEL_mS
    

;Init PORTC as output
    ser r26
    out DDRC, r26    
    
loop1:
    clr r26
loop2:
    out PORTC, r26
    
    ldi r24, low (DEL_NU)
    ldi r25, high (DEL_NU) 
    rcall delay_mS
    
    inc r26
    
    cpi r26, 32
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
    
    
    












