    $MODMAX10

; The Special Function Registers below were added to 'MODMAX10' recently.
; If you are getting an error, uncomment the three lines below.

; ADC_C DATA 0xa1
; ADC_L DATA 0xa2
; ADC_H DATA 0xa3

CSEG at 0
ljmp mycode

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
	




ADCREF_L      DATA 30H
ADCREF_H      DATA 31H

ADCLM_L       DATA 32H
ADCLM_H       DATA 33H

ADCOP_L       DATA 34H
ADCOP_H       DATA 35H

; -------- Results --------
TC_L          DATA 36H
TC_H          DATA 37H

TH_L          DATA 38H
TH_H          DATA 39H

TOVEN_L       DATA 3AH
TOVEN_H       DATA 3BH

TMP0          DATA 3CH
TMP1          DATA 3DH


dseg at 3EH
x:		ds	4
y:		ds	4
bcd:	ds	5
Count10ms: ds 1
PWM_Phase: ds 1
PWM_Duty: ds 1
Timer: ds 1
Target_Temp: ds 2


bseg

mf:		dbit 1
onesec: dbit 2
timeend: dbit 3



FREQ   EQU 33333333
BAUD   EQU 115200
T2LOAD EQU 65536-(FREQ/(32*BAUD))
TIMER0_RATE   EQU 100     ; 1000Hz, for a timer tick of 1ms
TIMER0_RELOAD EQU ((65536-(FREQ/(12*TIMER0_RATE))))


CSEG

InitSerialPort:
	; Configure serial port and baud rate
	clr TR2 ; Disable timer 2
	mov T2CON, #30H ; RCLK=1, TCLK=1 
	mov RCAP2H, #high(T2LOAD)  
	mov RCAP2L, #low(T2LOAD)
	setb TR2 ; Enable timer 2
	mov SCON, #52H
	ret

putchar:
    JNB TI, putchar
    CLR TI
    MOV SBUF, a
    RET

SendString:
    CLR A
    MOVC A, @A+DPTR
    JZ SSDone
    LCALL putchar
    INC DPTR
    SJMP SendString
SSDone:
    ret

$include(math32.asm)

cseg
; These 'equ' must match the wiring between the DE10Lite board and the LCD!
; P0 is in connector JPIO.  Check "CV-8052 Soft Processor in the DE10Lite Board: Getting
; Started Guide" for the details.
ELCD_RS equ P1.7
;ELCD_RW equ Px.x ; Not used.  Connected to ground 
ELCD_E  equ P1.1
ELCD_D4 equ P0.7
ELCD_D5 equ P0.5
ELCD_D6 equ P0.3
ELCD_D7 equ P0.1
PWM_PIN equ p3.7
Test_PIN equ p3.6


$NOLIST
$include(LCD_4bit_DE10Lite_no_RW.inc) ; A library of LCD related functions and utility macros
$LIST

; Look-up table for 7-seg displays


;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 0                     ;
;---------------------------------;
Timer0_Init:
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
	clr a
	mov Count10ms, a
	mov PWM_Phase, a
	
	mov PWM_Duty, #60
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	
    setb ET0  ; Enable timer 0 interrupt
    setb EA
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz square wave at pin P3.7 ;
;---------------------------------;
Timer0_ISR:
    ; reload timer (Timer0 has no auto-reload on CV-8052)
    mov TH0, #high(TIMER0_RELOAD)
    mov TL0, #low(TIMER0_RELOAD)
    ; TF0 is cleared automatically on interrupt entry (no need to clr)

    push acc
    push psw

    ; ---- DEBUG: toggle pin every interrupt (scope this!) ----
    ; Comment out once verified
    
    ; ---- 10ms tick counter -> 1 second flag ----
    inc count10ms
    inc PWM_Phase
    
    mov a, count10ms
    cjne a, #100, not_one_sec
    mov count10ms, #0
    mov PWM_Phase, #0
    setb onesec
    ljmp pwm_done
	
	not_one_sec:
	mov x+0, PWM_Phase
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    
   	mov y+0, PWM_Duty
    mov y+1, #0
    mov y+2, #0
    mov y+3, #0
    
    clr mf
	lcall x_gteq_y
	
	jnb mf, pwm_on
	clr PWM_PIN
    sjmp pwm_done
    
pwm_on:
    setb PWM_PIN
	

pwm_done:
    pop psw
    pop acc
    reti



myLUT:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F

Wait50ms:
;33.33MHz, 1 clk per cycle: 0.03us
	mov R0, #30
Wait50ms_L3:
	mov R1, #74
Wait50ms_L2:
	mov R2, #250
Wait50ms_L1:
	djnz R2, Wait50ms_L1 ;3*250*0.03us=22.5us
    djnz R1, Wait50ms_L2 ;74*22.5us=1.665ms
    djnz R0, Wait50ms_L3 ;1.665ms*30=50ms
    ret

Display_Voltage_7seg:
	
	mov dptr, #myLUT

	mov a, bcd+1
	swap a
	anl a, #0FH
	movc a, @a+dptr
	;anl a, #0x7f ; Turn on decimal point
	;mov HEX3, a
	
	mov a, bcd+1
	anl a, #0FH
	movc a, @a+dptr
	mov HEX2, a

	mov a, bcd+0
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX1, a
	
	mov a, bcd+0
	anl a, #0FH
	movc a, @a+dptr
	mov HEX0, a
	
	ret






Display_Temp_LCD:
    Set_Cursor(2,1)
    mov a, #'T'
    lcall ?WriteData
    mov a, #'='
    lcall ?WriteData

    ; Print 4 digits: bcd+1 high, bcd+1 low, bcd+0 high, bcd+0 low
    mov a, bcd+1
    swap a
    anl a, #0FH
    orl a, #'0'
    lcall ?WriteData

    mov a, bcd+1
    anl a, #0FH
    orl a, #'0'
    lcall ?WriteData

    mov a, bcd+0
    swap a
    anl a, #0FH
    orl a, #'0'
    lcall ?WriteData

    mov a, bcd+0
    anl a, #0FH
    orl a, #'0'
    lcall ?WriteData

    ret

Display_Voltage_LCD:
	Set_Cursor(2,1)
	mov a, #'V'
	lcall ?WriteData
	mov a, #'='
	lcall ?WriteData

	mov a, bcd+1
	swap a
	anl a, #0FH
	orl a, #'0'
	lcall ?WriteData
	
	mov a, #'.'
	lcall ?WriteData
	
	mov a, bcd+1
	anl a, #0FH
	orl a, #'0'
	lcall ?WriteData

	mov a, bcd+0
	swap a
	anl a, #0FH
	orl a, #'0'
	lcall ?WriteData
	
	mov a, bcd+0
	anl a, #0FH
	orl a, #'0'
	lcall ?WriteData
	
	ret
	
Display_Voltage_Serial:
	mov a, #'T'
	lcall putchar
	mov a, #'='
	lcall putchar

	mov a, bcd+1
	swap a
	anl a, #0FH
	orl a, #'0'
	lcall putchar
	
	
	mov a, bcd+1
	anl a, #0FH
	orl a, #'0'
	lcall putchar

	mov a, bcd+0
	swap a
	anl a, #0FH
	orl a, #'0'
	lcall putchar
	
	mov a, bcd+0
	anl a, #0FH
	orl a, #'0'
	lcall putchar

	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	
	ret

Initial_Message:  db 'Voltmeter test', 0

Temperature_Check:

;mov a, SWA ; The first three switches select the channel to read
	mov a, #0x00 ;get the reference voltage
	
	mov ADC_C, a
	lcall Wait50ms
	lcall Wait50ms
	
	
	mov  a, ADC_H
	anl  a, #0x0F
	mov ADCREF_H, a
	mov ADCREF_L, ADC_L
	
	mov a, #0x01 ; get the lm335 voltage
	mov ADC_C, a
	
	lcall Wait50ms
	lcall Wait50ms
	
	; Load 32-bit 'x' with 12-bit adc result
	
	mov  a, ADC_H
	anl  a, #0x0F	
	mov x+3, #0
	mov x+2, #0
	mov x+1, a
	mov x+0, ADC_L
	
	; LM335 calculations
	Load_y(4096)
	lcall mul32
	
	
	mov y+3, #0
	mov y+2, #0
	mov y+1, ADCREF_H
	mov y+0, ADCREF_L
	
	lcall div32
	
	Load_y(2730)
	lcall sub32
	
	Load_y(10)
	lcall div32
	
	;store results
	mov TC_H, x+1
	mov TC_L, x+0
	
	;get op amp voltage
	
	mov a, #0x02
	mov ADC_C, a
	lcall Wait50ms
	lcall Wait50ms

	
	mov  a, ADC_H
	anl  a, #0x0F
	
	mov x+3, #0
	mov x+2, #0
	mov x+1, a
	mov x+0, ADC_L
	
	Load_y(333)
	lcall mul32
	
	mov y+3, #0
	mov y+2, #0
	mov y+1, ADCREF_H
	mov y+0, ADCREF_L
	
	lcall div32
	;store temp 
	mov TH_H, x+1
	mov TH_L, x+0
	
	
	mov y+3, #0
	mov y+2, #0
	mov y+1, TC_H
	mov y+0, TC_L
	
	lcall add32
	
	mov TOVEN_H, x+1
	mov TOVEN_L, x+0
	
	
	
	mov x+3, #0
	mov x+2, #0
	mov x+1, TOVEN_H
	mov x+0, TOVEN_L
	lcall Wait50ms
	
ret



mycode:
	mov SP, #7FH
	clr a
	mov LEDRA, a
	mov LEDRB, a
	
	lcall InitSerialPort
	lcall Timer0_Init
	
	
	; COnfigure the pins connected to the LCD as outputs
	mov P0MOD, #10101010b ; P0.1, P0.3, P0.5, P0.7 are outputs.  ('1' makes the pin output)
    mov P1MOD, #10000010b ; P1.7 and P1.1 are outputs
    mov p2MOD, #11111111b
    mov P3MOD, #11111111b

    lcall ELCD_4BIT ; Configure LCD in four bit mode
    ; For convenience a few handy macros are included in 'LCD_4bit_DE1Lite.inc':
	Set_Cursor(1, 1)
    Send_Constant_String(#Initial_Message)
	
	mov dptr, #Initial_Message
	lcall SendString
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	

	mov ADC_C, #0x80 ; Reset ADC
	lcall Wait50ms
	
	lcall Temperature_Check

forever:
	
	
	jnb onesec, no_send
    clr onesec
    
	mov a, Timer
	add a, #0x99
	mov Timer, a
	
	cjne a, #0, moveon
    setb timeend
    
    moveon:
    lcall Temperature_Check
    
    mov x+0, TOVEN_L
    mov x+1, TOVEN_H
    mov x+2, #0
    mov x+3, #0
    
   	mov y+0, Target_Temp+0
   	mov y+1, Target_Temp+1
   	mov y+2, #0
   	mov y+3, #0
   	
   	clr mf
   	lcall x_gteq_y
   	
    
    
    lcall hex2bcd
	lcall Display_Temp_LCD
	;xlcall Display_Voltage_LCD
	lcall Display_Voltage_Serial
	no_send:
	
	ljmp forever
	
end
