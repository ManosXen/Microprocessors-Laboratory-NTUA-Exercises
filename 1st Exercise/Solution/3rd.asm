.org 0x000
.def wagon=r16
.equ delay=1*1500
    
ldi wagon, LOW(RAMEND)
out SPL, wagon
ldi wagon,HIGH(RAMEND)
out SPH, wagon
    
ser wagon
out DDRD, wagon
ldi wagon, 0x01    
set
    
left:
    out PORTD, wagon
    ldi r24, low(delay)
    ldi r25, high(delay)
    rcall wait_x_msec
    lsl wagon
    cpi wagon, 0x00
    brne left
rotater:
    ldi r24, low(2000)
    ldi r25, high(2000)
    rcall wait_x_msec
    clt
    ldi wagon, 0x40
right:
    out PORTD, wagon
    ldi r24, low(delay)
    ldi r25, high(delay)
    rcall wait_x_msec
    lsr wagon
    cpi wagon, 0x00
    brne right

rotatel:
    ldi r24, low(2000)
    ldi r25, high(2000)
    rcall wait_x_msec
    set
    ldi wagon, 0x02
    rjmp left
    
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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    