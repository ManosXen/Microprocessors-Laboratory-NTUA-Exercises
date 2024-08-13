.org 0x000
.def global_counter=r16
.def A=r17
.def B=r18
.def C=r19
.def D=r20
.def F0=r21
.def F1=r22
.def temp=r23
    
ldi global_counter, 4
ldi A, 0x45
ldi B, 0x23    
ldi C, 0x21    
ldi D, 0x01
    
loop:
    
F0_calc:
    com A
    com B
    com C
    mov temp, A
    and temp, B
    and temp, C
    or temp, D
    com temp
    mov F0, temp

F1_calc:
    com C
    com D
    mov temp, A
    or temp, C
    mov F1, temp
    mov temp, B
    or temp, D
    and F1, temp

next:
    com A
    com B
    com D
    subi A, -1
    subi B, -2
    subi C, -4
    subi D, -5
    dec global_counter
    brne loop
    
    
    



