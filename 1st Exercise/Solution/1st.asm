;.include "m328PBdef.inc"


.org 0x000
.def temp=r16
    
.equ freq=1 ;MHz
.equ delay=100 ; ms    
.equ cycles=freq*delay   ;thousand cycles 
    
    
ldi temp, LOW(RAMEND)
out SPL, temp
ldi temp,HIGH(RAMEND)
out SPH, temp

    
ser temp
out DDRB, temp
    
    
con_loop:
    ldi r24, low(cycles)
    ldi r25, high(cycles)
    out PORTD, temp
    rcall wait_x_msec
    com temp
    rjmp con_loop



wait_x_msec:
    
    ;init will consume 248*4+3=996 cycles
    init: ldi r23, 249

    loop:
	subi r23, 1
	nop
	brne loop

    sbiw r24, 1 ;2 cycles 
    brne init 
    
ret 
   
;H teleutaia xiliada einai 1003 kykloi giati einai 996 apo to loop +2 apo to sbiw +1 apo to brne diladi 999 kai apo to ret exoume 4 ara synolo 1003