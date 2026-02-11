$MODMAX10

;============================================================
; 1. INTERRUPT VECTORS
;============================================================
    CSEG at 0
    ljmp main_code

org 0x0003
    reti

org 0x000B
    ljmp Timer0_ISR

org 0x0013
    reti
    
org 0x001B
    ljmp Timer1_ISR

org 0x0023
    reti

;============================================================
; 2. RAM VARIABLES
;============================================================
ADCREF_L      DATA 30H
ADCREF_H      DATA 31H

ADCLM_L       DATA 32H
ADCLM_H       DATA 33H

ADCOP_L       DATA 34H
ADCOP_H       DATA 35H

TC_L          DATA 36H
TC_H          DATA 37H

TH_L          DATA 38H
TH_H          DATA 39H

TOVEN_L       DATA 3AH
TOVEN_H       DATA 3BH

TMP0          DATA 3CH
TMP1          DATA 3DH

    DSEG at 40H
x:              ds 4
y:              ds 4
Count10ms:      ds 1
PWM_Phase:      ds 1
PWM_Duty:       ds 1
bcd:            ds 5
Controller_Mode: ds 1
Soak_temp:      ds 1
Soak_Minute:    ds 1
Soak_Second:    ds 1
Reflow_temp:    ds 1
Reflow_Minute:  ds 1
Reflow_Second:  ds 1 
Soak_Time_State: ds 1 
Reflow_Time_State: ds 1

Melody_Ptr:      ds 2
Melody_Index:    ds 1
Melody_Duration: ds 1
Melody_Playing:  ds 1

;---- FSM variables
Stage:          ds 1
PrevStage:      ds 1
StageSec:       ds 1 
ErrorCode:      ds 1
Temperature:    ds 2 ; 16-bit
Timer_Run:      ds 1
UI_State:       ds 1

Soak_Temp16:    ds 2
Soak_TimeSec:   ds 1
Reflow_Temp16:  ds 2
Reflow_TimeSec: ds 1

;---- progress bar variables
t_start:       DS 1    ; Starting temperature
t_target:      DS 1    ; Target temperature
t_current:     DS 1    ; Current temperature
total_range:   DS 1    ; Total temperature range
curr_range:    DS 1    ; Current progress range
bar_count:     DS 1    ; Number of filled bars (0-12)
percent:       DS 1    ; Progress percentage (0-100)
bar_idx:           DS 1    ; Loop index
Ramp1_start_temp: DS 1 ; Initial temperature when Stage 1 starts
Ramp3_start_temp: DS 1  ;

;---- Sound Variables (NEW)
Beep_Count:     ds 1
Beep_Timer:     ds 1
Current_Note_H: ds 1   
Current_Note_L: ds 1  
Cooling5_start_temp: DS 1

bseg

mf:             dbit 1
onesec:         dbit 1
timeend:        dbit 1

EmergencyLatched: dbit 1

;============================================================
; 3. CONSTANTS & STRINGS
;============================================================
    CSEG

Soak_Temp_Message:    db 'STemp:       ___', 0
Soak_Time_Message:    db 'STime:     __:__', 0 
Reflow_Temp_Message:  db 'RTemp:       ___', 0
Reflow_Time_Message:  db 'RTime:     __:__', 0 
Stats_Message_Top:    db 'T:XXX   M:    ', 0
Stats_Message_Bottom: db 'S: XX:XX        ', 0
Ready_Message:        db 'READY TO START  ', 0
Press_Star_Msg:       db 'PRESS * TO RUN  ', 0
Blank:                db '                ', 0
Ramp_To_Soak:         db 'RPTOSK', 0
Soak:                 db '  SOAK', 0
Ramp_To_Reflow:       db 'RPTORW', 0
Reflow:               db 'REFLOW', 0
Cooling:              db 'COOLIN', 0
Done_Msg:             db '  DONE', 0
Error_Line2_Msg:      db 'EMERGENCY STOP! ', 0
Error_Word:           db 'Error_Word', 0

; New Strings for layout
Msg_Target_Soak:      db 'Stemp: ', 0
Msg_Target_Reflow:    db 'Rtemp: ', 0
Msg_Time_Rem:         db ' t:   ', 0

FREQ            EQU 33333333
BAUD            EQU 115200
T2LOAD          EQU 65536-(FREQ/(32*BAUD))

TIMER0_RATE     EQU 100
TIMER0_RELOAD   EQU (65536-(FREQ/(12*TIMER0_RATE)))

MAX_STAGE_TIME EQU 240
COOLING_TEMP   EQU 60

SOAK_DEADBAND      EQU 3     ; ?3?C
REFLOW_DEADBAND    EQU 3

SOAK_PWM_HIGH       EQU 60
SOAK_PWM_MAINTAIN   EQU 20
SOAK_PWM_LOW        EQU 0 

REFLOW_PWM_HIGH     EQU 50
REFLOW_PWM_MAINTAIN EQU 20
REFLOW_PWM_LOW      EQU 0

PWM_PIN  equ P3.7
Test_PIN equ P3.6
SSR_OUT  equ PWM_PIN   
SPEAKER  equ P2.1

EMERG_STOP  BIT SWA.0 

; Keypad Pins
ROW1 EQU P1.2
ROW2 EQU P1.4
ROW3 EQU P1.6
ROW4 EQU P2.0
COL1 EQU P2.2
COL2 EQU P2.4
COL3 EQU P2.6
COL4 EQU P3.0

; LCD Pins
ELCD_RS  EQU P1.7
ELCD_E   EQU P1.1
ELCD_D4  EQU P0.7
ELCD_D5  EQU P0.5
ELCD_D6  EQU P0.3
ELCD_D7  EQU P0.1

TONE0P5_RELOAD  EQU 64857 ;x

;============================================================
; MUSICAL NOTE DEFINITIONS (Timer1 Reload Values)
;============================================================
; Formula: 65536 - (FREQ / (12 * 2 * Note_Frequency))
; For 33.333MHz crystal
NOTE_C6  EQU 64208
NOTE_D6  EQU 64414
NOTE_E6  EQU 64610
NOTE_F6  EQU 64712
NOTE_G6  EQU 64864
NOTE_A6  EQU 64998
NOTE_B6  EQU 65119
NOTE_C7  EQU 65183
NOTE_D7  EQU 65271
NOTE_E7  EQU 65351
NOTE_G7  EQU 65446
NOTE_REST EQU 0




;============================================================
; 4. INITIALIZATION & UTILS
;============================================================

InitSerialPort:
    clr TR2
    mov T2CON, #30H
    mov RCAP2H, #high(T2LOAD)
    mov RCAP2L, #low(T2LOAD)
    setb TR2
    mov SCON, #52H
    setb TI
    ret

putchar:
    jnb TI, putchar
    clr TI
    mov SBUF, a
    ret

UART_CRLF:
    mov a, #'\r'
    lcall putchar
    mov a, #'\n'
    lcall putchar
    ret

UART_PrintU8_Dec:
    ; Input: A = 0..255
    ; Output: ASCII decimal via putchar
    push acc
    push b
    push psw

    mov b, #100
    div ab              ; A=hundreds, B=remainder
    jz  UP8_TENS        ; if hundreds==0 skip
    add a, #'0'
    lcall putchar

UP8_TENS:
    mov a, b            ; remainder
    mov b, #10
    div ab              ; A=tens, B=ones

    jz  UP8_ONES
    add a, #'0'
    lcall putchar

UP8_ONES:
    mov a, b
    add a, #'0'
    lcall putchar

    pop psw
    pop b
    pop acc
    ret

UART_SendTemperature:
    push acc
    push psw

    mov a, Temperature+1
    jz  UST_low_ok
    mov a, #255
    sjmp UST_print
UST_low_ok:
    mov a, Temperature+0

UST_print:
    lcall UART_PrintU8_Dec
    lcall UART_CRLF

    pop psw
    pop acc
    ret

SendString:
    clr A
    movc A, @A+DPTR
    jz SSDone
    lcall putchar
    inc DPTR
    sjmp SendString
SSDone:
    ret

$include(math32.asm)
$include(LCD_4bit_DE10Lite_no_RW.inc)

; 7-Segment Lookup Table
myLUT:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99
    DB 0x92, 0x82, 0xF8, 0x80, 0x90
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E
    
;============================================================
; STAGE MELODIES
;============================================================

; Stage 1??2
Melody_ToSoak:
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 12
    
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 12
    
    DB high(NOTE_E6), low(NOTE_E6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_G6), low(NOTE_G6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_C6), low(NOTE_C6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_D6), low(NOTE_D6), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E6), low(NOTE_E6), 30
    DB high(NOTE_REST), low(NOTE_REST), 10
    DB 0xFF

;; Stage 2 -> 3
Melody_SoakDone:
    DB high(NOTE_E7), low(NOTE_E7), 25
    DB high(NOTE_E7), low(NOTE_E7), 25
    DB high(NOTE_REST), low(NOTE_REST), 7
    DB high(NOTE_E7), low(NOTE_E7), 25
    DB high(NOTE_REST), low(NOTE_REST), 20
    DB high(NOTE_C7), low(NOTE_C7), 20
    DB high(NOTE_E7), low(NOTE_E7), 40
    DB high(NOTE_G7), low(NOTE_G7), 60
    DB 0xFF

; Stage 3 -> 4
Melody_ToReflow:
    DB high(NOTE_C7), low(NOTE_C7), 12
    DB high(NOTE_E7), low(NOTE_E7), 12
    DB high(NOTE_C7), low(NOTE_C7), 12
    DB high(NOTE_E7), low(NOTE_E7), 20
    DB 0xFF

; Stage 4 -> 5
Melody_ReflowDone:
    DB high(NOTE_E7), low(NOTE_E7), 15
    DB high(NOTE_C7), low(NOTE_C7), 15
    DB high(NOTE_G6), low(NOTE_G6), 25
    DB 0xFF

; Stage 5 -> 6
Melody_Complete:
    DB high(NOTE_C6), low(NOTE_C6), 10
    DB high(NOTE_C6), low(NOTE_C6), 10
    DB high(NOTE_C6), low(NOTE_C6), 10
    DB high(NOTE_REST), low(NOTE_REST), 5
    DB high(NOTE_C7), low(NOTE_C7), 30
    DB 0xFF

Melody_Error:
    DB high(NOTE_E7), low(NOTE_E7), 8
    DB high(NOTE_C6), low(NOTE_C6), 8
    DB high(NOTE_E7), low(NOTE_E7), 8
    DB high(NOTE_C6), low(NOTE_C6), 8
    DB high(NOTE_E7), low(NOTE_E7), 8
    DB high(NOTE_C6), low(NOTE_C6), 8
    DB 0xFF


showBCD MAC
    mov A, %0
    anl a, #0fh
    movc A, @A+dptr
    mov %1, A
    mov A, %0
    swap a
    anl a, #0fh
    movc A, @A+dptr
    mov %2, A
ENDMAC

Display:
    push psw                ; SAVE FLAGS (Crucial: Protects the "Key Pressed" Carry flag)
    push acc                ; Save Accumulator
    push b                  ; Save B register

    mov dptr, #myLUT        ; Point to 7-segment lookup table

    ; Check if High Byte exists (Over 255 degrees)
    mov a, Temperature+1
    jnz Display_Overflow

    mov a, Temperature+0    ; Load Low Byte
    sjmp Display_Convert

Display_Overflow:
    mov a, #255             ; Cap at 255 for display

Display_Convert:
    ;--- Hundreds Digit ---
    mov b, #100
    div ab                  ; A = Hundreds, B = Remainder
    movc a, @a+dptr
    mov HEX2, a

    ;--- Tens Digit ---
    mov a, b
    mov b, #10
    div ab                  ; A = Tens, B = Ones
    movc a, @a+dptr
    mov HEX1, a

    ;--- Ones Digit ---
    mov a, b
    movc a, @a+dptr
    mov HEX0, a

    ;--- Turn off Unused Displays ---
    mov HEX3, #0FFh
    mov HEX4, #0FFh
    mov HEX5, #0FFh

    pop b                   ; Restore B
    pop acc                 ; Restore Accumulator
    pop psw                 ; RESTORE FLAGS (Carry flag is back to how Keypad left it)
    ret

MYRLC MAC
    mov a, %0
    rlc a
    mov %0, a
ENDMAC

Shift_Digits_Left:
    mov R0, #4
    clr c
Shift_Digits_Left_L0:
    MYRLC(bcd+0)
    MYRLC(bcd+1)
    MYRLC(bcd+2)
    MYRLC(bcd+3)
    MYRLC(bcd+4)
    djnz R0, Shift_Digits_Left_L0
    mov a, R7
    anl a, #0Fh 
    orl a, bcd+0
    mov bcd+0, a
    ret
    
MYRRC MAC
    mov a, %0
    rrc a
    mov %0, a
ENDMAC

Shift_Digits_Right:
    mov R0, #4 
Shift_Digits_Right_L0:
    clr c
    MYRRC(bcd+4)
    MYRRC(bcd+3)
    MYRRC(bcd+2)
    MYRRC(bcd+1)
    MYRRC(bcd+0)
    djnz R0, Shift_Digits_Right_L0
    ret

Wait25ms:
    mov R0, #15
L3_25: mov R1, #74
L2_25: mov R2, #250
L1_25: djnz R2, L1_25
       djnz R1, L2_25
       djnz R0, L3_25
    ret

Wait50ms:
    mov R0, #30
W3_50: mov R1, #74
W2_50: mov R2, #250
W1_50: djnz R2, W1_50
       djnz R1, W2_50
       djnz R0, W3_50
    ret

CHECK_COLUMN MAC
    jb %0, CHECK_COL_%M
    mov R7, %1
    jnb %0, $ 
    setb c
    ret
CHECK_COL_%M:
ENDMAC

Configure_Keypad_Pins:
    orl P1MOD, #0b_01010100 
    orl P2MOD, #0b_00000001 
    anl P2MOD, #0b_10101011 
    anl P3MOD, #0b_11111110 
    ret

Keypad:
    jb KEY.1, keypad_L0
    lcall Wait25ms 
    jb KEY.1, keypad_L0
    jnb KEY.1, $ 
    lcall Shift_Digits_Right
    clr c
    ret
keypad_L0:
    clr ROW1
    clr ROW2
    clr ROW3
    clr ROW4
    mov c, COL1
    anl c, COL2
    anl c, COL3
    anl c, COL4
    jnc Keypad_Debounce
    clr c
    ret
Keypad_Debounce:
    lcall Wait25ms 
    mov c, COL1
    anl c, COL2
    anl c, COL3
    anl c, COL4
    jnc Keypad_Key_Code
    clr c
    ret
Keypad_Key_Code:    
    setb ROW1
    setb ROW2
    setb ROW3
    setb ROW4
keypad_default:     
    clr ROW1
    CHECK_COLUMN(COL1, #01H)
    CHECK_COLUMN(COL2, #02H)
    CHECK_COLUMN(COL3, #03H)
    CHECK_COLUMN(COL4, #0AH)
    setb ROW1
    clr ROW2
    CHECK_COLUMN(COL1, #04H)
    CHECK_COLUMN(COL2, #05H)
    CHECK_COLUMN(COL3, #06H)
    CHECK_COLUMN(COL4, #0BH)
    setb ROW2
    clr ROW3
    CHECK_COLUMN(COL1, #07H)
    CHECK_COLUMN(COL2, #08H)
    CHECK_COLUMN(COL3, #09H)
    CHECK_COLUMN(COL4, #0CH)
    setb ROW3
    clr ROW4
    CHECK_COLUMN(COL1, #0EH)
    CHECK_COLUMN(COL2, #00H)
    CHECK_COLUMN(COL3, #0FH)
    CHECK_COLUMN(COL4, #0DH)
    setb ROW4
    clr c
    ret

;============================================================
; 5. TIMER & ISR (MODIFIED FOR SOUND)
;============================================================
Timer0_Init:
    mov a, TMOD
    anl a, #0F0h
    orl a, #01h
    mov TMOD, a
    mov TH0, #high(TIMER0_RELOAD)
    mov TL0, #low(TIMER0_RELOAD)
    mov Count10ms, #0
    mov PWM_Phase, #0
    mov PWM_Duty,  #0
    ; Init Sound
    mov Beep_Count, #0
    mov Beep_Timer, #0
    setb ET0
    setb EA
    setb TR0
    ret

Timer0_ISR:
    mov TH0, #high(TIMER0_RELOAD)
    mov TL0, #low(TIMER0_RELOAD)
    push acc
    push psw
    jb  EmergencyLatched, ES_Done      ; already latched -> do nothing
    jnb EMERG_STOP, ES_Done            ; not pressed -> skip

    ; pressed -> latch and force ERROR NOW
    setb EmergencyLatched

    mov PWM_Duty, #0
    clr PWM_PIN                        ; immediate SSR OFF (don?t wait for PWM compare)
    mov Timer_Run, #0                  ; stop FSM running in main loop

    mov Stage, #7
    mov StageSec, #0
    mov ErrorCode, #9                  ; pick a code: 9 = emergency stop

    mov Beep_Count, #10                ; optional alarm beeps
    mov Beep_Timer, #0

ES_Done:

    inc count10ms
    inc PWM_Phase
    
    ; --------------------------------------
    ; SPEAKER LOGIC (Non-blocking)
    ; --------------------------------------
    mov a, Beep_Count
    jz Stop_Speaker     ; If no beeps left, ensure speaker OFF

    inc Beep_Timer
    mov a, Beep_Timer
    
    ; Logic: ON for 10 ticks (100ms), OFF for 10 ticks (100ms)
    ; Total Period = 20 ticks (200ms)
    
    cjne a, #10, Check_Phase
    ; Exact transition point, handled by logic below
Check_Phase:
    jc Turn_Speaker_On      ; Beep_Timer < 10  => ON window

    ; OFF window (10..19)
    lcall StopTone
    sjmp Check_Cycle_End

Turn_Speaker_On:
    mov Current_Note_H, #high(TONE0P5_RELOAD)
    mov Current_Note_L, #low(TONE0P5_RELOAD)
    lcall StartTone
    sjmp PWM_Logic

Check_Cycle_End:
    mov a, Beep_Timer
    cjne a, #20, PWM_Logic
    ; Cycle complete
    mov Beep_Timer, #0
    dec Beep_Count      ; One beep done
    sjmp PWM_Logic

Stop_Speaker:
    lcall StopTone
    sjmp Check_Melody    ; ?? ??????????????

;----------------------------------------------------
; MELODY PLAYBACK LOGIC (NEW)
;----------------------------------------------------
Check_Melody:
    mov a, Melody_Playing
    jz PWM_Logic
    
    mov a, Melody_Duration
    jz Load_Next_Note
    dec Melody_Duration
    sjmp PWM_Logic

Load_Next_Note:

    lcall StopTone
    
    mov dph, Melody_Ptr+1
    mov dpl, Melody_Ptr+0
    mov a, Melody_Index
    add a, dpl
    mov dpl, a
    mov a, #0
    addc a, dph
    mov dph, a
    
    clr a
    movc a, @a+dptr
    cjne a, #0xFF, Play_Note
    
    mov Melody_Playing, #0
    sjmp PWM_Logic

Play_Note:
    clr a
    movc a, @a+dptr
    mov R6, a
    
    inc dptr
    clr a
    movc a, @a+dptr
    mov R7, a
    
    inc dptr
    clr a
    movc a, @a+dptr
    mov Melody_Duration, a
    
    mov a, Melody_Index
    add a, #3
    mov Melody_Index, a
    
    mov a, R6
    orl a, R7
    jz PWM_Logic
    
    mov Current_Note_H, R6
    mov Current_Note_L, R7
    
    mov TH1, R6
    mov TL1, R7
    setb ET1
    setb TR1

    sjmp PWM_Logic
    
    ; --------------------------------------
    ; PWM LOGIC
    ; --------------------------------------
PWM_Logic:
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

;============================================================
; 6. TEMPERATURE LOGIC
;============================================================
Temperature_Check:
    ; --- 1. Read Reference ---
    mov a, #0x00
    mov ADC_C, a
    lcall Wait50ms
    mov  a, ADC_H
    anl  a, #0x0F
    mov ADCREF_H, a
    mov ADCREF_L, ADC_L

    ; --- 2. Read LM335 (Kelvin) ---
    mov a, #0x01
    mov ADC_C, a
    lcall Wait50ms
    mov  a, ADC_H
    anl  a, #0x0F
    mov x+3, #0
    mov x+2, #0
    mov x+1, a
    mov x+0, ADC_L

    Load_y(4096)
    lcall mul32
    mov y+3, #0
    mov y+2, #0
    mov y+1, ADCREF_H
    mov y+0, ADCREF_L
    lcall div32
    ; X now contains Temperature in Kelvin * 10 (e.g., 2980 for 25C)

    ; --- FIX: Check for Underflow before subtracting ---
    ; We check if Kelvin (x) >= 2730 (y). 
    ; If Kelvin < 2730, we are below 0C. Clamp to 0 to avoid wrap-around.

    Load_y(2730) 
    lcall x_gteq_y     ; Checks if x >= 2730
    jnb mf, TC_Underflow ; If mf=0, x is less than 2730. Jump to clamp.

    ; If we are here, Temp is >= 0C. Safe to subtract.
    Load_y(2730)
    lcall sub32        ; x = x - 2730

    Load_y(10)
    lcall div32
    mov TC_H, x+1
    mov TC_L, x+0
    sjmp TC_Calc_Done

TC_Underflow:
    ; Sensor reading is impossibly low (or 0V). Force 0C.
    mov TC_H, #0
    mov TC_L, #0

TC_Calc_Done:
    ; --- 3. Read OpAmp (Ambient) ---
    mov a, #0x02
    mov ADC_C, a
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
    mov TH_H, x+1
    mov TH_L, x+0

    ; --- 4. Sum (Probe Temp + Ambient Temp) ---
    mov y+3, #0
    mov y+2, #0
    mov y+1, TC_H
    mov y+0, TC_L
    lcall add32
    mov TOVEN_H, x+1
    mov TOVEN_L, x+0

    ; Store Global
    mov Temperature+0, TOVEN_L
    mov Temperature+1, TOVEN_H
    ret

;============================================================
; 7. FSM LOGIC
;============================================================
; UPDATED: Sets the beep count for the ISR to handle
StartBuzzer:
    mov Beep_Count, a   ; Load number of beeps requested
    mov Beep_Timer, #0  ; Reset the duration timer
    ret
    
;============================================================
; Play Melody Function
;============================================================
PlayMelody:
    push acc
    push psw
    
    mov Melody_Ptr+1, dph
    mov Melody_Ptr+0, dpl
    
    mov Melody_Index, #0
    mov Melody_Duration, #0
    mov Melody_Playing, #1
    
    pop psw
    pop acc
    ret

StopMelody:
    mov Melody_Playing, #0
    lcall StopTone
    ret

SAFETY_CHECK_ALL:
    push acc
    push psw

    mov ErrorCode, #0

    mov a, Stage
    cjne a, #1, safety_done

    ; StageSec < 60
    mov a, StageSec
    clr c
    subb a, #60
    jc  safety_done

    mov a, Temperature+1
    jnz safety_done  

    mov a, Temperature+0 
    clr c
    subb a, #50
    jnc safety_done            ; Temperature >= 50 -> OK

    ; Temperature < 50 -> Error
    mov ErrorCode, #5

safety_done:
    pop psw
    pop acc
    ret

FSM1_Reflow:
    mov a, Stage
    cjne a, PrevStage, FSM_state_changed
    sjmp FSM_no_change

FSM_state_changed:
    mov PrevStage, a
    ; If error, beep lots, else beep once
    cjne a, #7, FSM_beep_change
    mov dptr, #Melody_Error
    lcall PlayMelody
    sjmp FSM_no_change
FSM_beep_change:
    lcall UpdateStage_LEDs ; Update LEDs immediately on change

FSM_no_change:
    mov a, Stage
    
    ; Stage 0: Idle
    cjne a, #0, FSM_s1
    mov PWM_Duty, #0
    mov StageSec, #0
    ret

    ; Stage 1: Heat to Soak
FSM_s1:
    cjne a, #1, FSM_s2
    lcall SAFETY_CHECK_ALL
    mov a, ErrorCode
    jz FSM_s1_go
    ljmp FSM_enter_error
FSM_s1_go:
    mov PWM_Duty, #100
    
    ; Timeout Check
    mov a, StageSec
    clr c
    subb a, #MAX_STAGE_TIME
    jnc FSM_timeout_error_jmp

    ; Check Temp >= Soak_Temp16
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0
    mov y+0, Soak_Temp16+0
    mov y+1, Soak_Temp16+1
    mov y+2, #0
    mov y+3, #0
    clr mf
    lcall x_gteq_y
    jnb mf, FSM_s1_stay
    
    ; Next Stage
    mov Stage, #2
    mov StageSec, #0
    mov PWM_Duty, #20
    mov dptr, #Melody_ToSoak
    lcall PlayMelody
    ret

FSM_s1_stay:
    inc StageSec
    ret
FSM_timeout_error_jmp:
    ljmp FSM_timeout_error

    ; Stage 2: Soak Hold
FSM_s2_not2:
    ljmp FSM_s3  
    
FSM_s2:
    cjne a, #2, FSM_s2_not2
    lcall SAFETY_CHECK_ALL

     ; If Temperature >= Soak_Temp + deadband => LOW PWM
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0
    mov y+0, Soak_Temp16+0
    mov y+1, Soak_Temp16+1
    mov y+2, #0
    mov y+3, #0
    Load_x(0)                 ; (we'll reuse x/y, so don't care)
    ; build y = Soak_Temp + SOAK_DEADBAND
    mov a, Soak_Temp16+0
    add a, #SOAK_DEADBAND
    mov y+0, a
    mov a, Soak_Temp16+1
    addc a, #0
    mov y+1, a
    mov y+2, #0
    mov y+3, #0
    ; x = Temperature
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0
    clr mf
    lcall x_gteq_y             ; mf=1 if Temp >= (Soak+db)
    jb  mf, SOAK_low
    ; If Temperature <= Soak_Temp - deadband => HIGH PWM
    ; Build y = Soak_Temp - SOAK_DEADBAND  (clamp at 0)
    mov a, Soak_Temp16+0
    clr c
    subb a, #SOAK_DEADBAND
    mov y+0, a
    mov a, Soak_Temp16+1
    subb a, #0
    mov y+1, a
    mov y+2, #0
    mov y+3, #0
    ; Check: Temp >= (Soak-db) ?
    ; if NOT, then Temp < (Soak-db) => too cold => HIGH PWM
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0

    clr mf
    lcall x_gteq_y
    jnb mf, SOAK_high

    ; Otherwise within band => maintain
SOAK_maintain:
    mov PWM_Duty, #SOAK_PWM_MAINTAIN
    sjmp SOAK_done

SOAK_high:
    mov PWM_Duty, #SOAK_PWM_HIGH
    sjmp SOAK_done

SOAK_low:
    mov PWM_Duty, #SOAK_PWM_LOW

SOAK_done:
; pwm change done    
    ; Calculate remaining time: Soak_TimeSec - StageSec
    mov a, Soak_TimeSec
    clr c
    subb a, StageSec
    
    ; If result is 0 or negative (carry set), move next
    jc FSM_s2_done 
    jz FSM_s2_done

    ; Display Countdown on Line 2, position 12 (Layout: "S:XXX t:MM:SS")
    ; We need to call LCD print here because the time changes every second
    push acc
    Set_Cursor(2, 12) 
    pop acc
    lcall LCD_Print_Countdown_A ; Print Remaining Time
    inc StageSec
    ret



FSM_s2_done:
    mov Stage, #3
    mov StageSec, #0
    mov PWM_Duty, #100
    
    mov a, Temperature+0
    
    mov Ramp3_start_temp, a
    
    mov dptr, #Melody_SoakDone
    lcall PlayMelody
    
    lcall UpdateStage_LEDs
    ret

    ; Stage 3: Heat to Reflow
FSM_s3:
    cjne a, #3, FSM_s4
    lcall SAFETY_CHECK_ALL
    mov a, ErrorCode
    jz FSM_s3_go
    ljmp FSM_enter_error
FSM_s3_go:
    mov PWM_Duty, #100
    mov a, StageSec
    clr c
    subb a, #MAX_STAGE_TIME
    jnc FSM_s3_timeout_near 
    
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0
    mov y+0, Reflow_Temp16+0
    mov y+1, Reflow_Temp16+1
    mov y+2, #0
    mov y+3, #0
    clr mf
    lcall x_gteq_y
    jnb mf, FSM_s3_stay

    mov Stage, #4
    mov StageSec, #0
    mov PWM_Duty, #20
    mov dptr, #Melody_ToReflow
    lcall PlayMelody 
    ret
    
FSM_s3_timeout_near:
    ljmp FSM_timeout_error

FSM_s3_stay:
    inc StageSec
    ret

    ; Stage 4: Reflow Hold
FSM_s4:
    cjne a, #4, FSM_s4_not4
    lcall SAFETY_CHECK_ALL

    ; If Temp >= Reflow + db => LOW
    mov a, Reflow_Temp16+0
    add a, #REFLOW_DEADBAND
    mov y+0, a
    mov a, Reflow_Temp16+1
    addc a, #0
    mov y+1, a
    mov y+2, #0
    mov y+3, #0

    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0

    clr mf
    lcall x_gteq_y
    jb  mf, REFLOW_low

    ; If Temp < Reflow - db => HIGH
    mov a, Reflow_Temp16+0
    clr c
    subb a, #REFLOW_DEADBAND
    mov y+0, a
    mov a, Reflow_Temp16+1
    subb a, #0
    mov y+1, a
    mov y+2, #0
    mov y+3, #0

    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0

    clr mf
    lcall x_gteq_y
    jnb mf, REFLOW_high

REFLOW_maintain:
    mov PWM_Duty, #REFLOW_PWM_MAINTAIN
    sjmp REFLOW_done

REFLOW_high:
    mov PWM_Duty, #REFLOW_PWM_HIGH
    sjmp REFLOW_done

REFLOW_low:
    mov PWM_Duty, #REFLOW_PWM_LOW

REFLOW_done:
; pwm change done  
    ; Calculate remaining time: Reflow_TimeSec - StageSec
    mov a, Reflow_TimeSec
    clr c
    subb a, StageSec
    
    ; If result is 0 or negative, move next
    jc FSM_s4_done
    jz FSM_s4_done

    ; Display Countdown
    push acc
    Set_Cursor(2, 12)
    pop acc
    lcall LCD_Print_Countdown_A
    
    inc StageSec
    ret

FSM_s4_not4:
    ljmp FSM_s5 

FSM_s4_done:
    mov Stage, #5
    mov StageSec, #0
    mov PWM_Duty, #0
    clr SSR_OUT
    
    mov a, Temperature+0
    mov Cooling5_start_temp, a
    
    mov dptr, #Melody_ReflowDone
    lcall PlayMelody
    lcall UpdateStage_LEDs
    ret

    ; Stage 5: Cooling
FSM_s5:
    cjne a, #5, FSM_s6
    mov PWM_Duty, #0
    
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov x+2, #0
    mov x+3, #0
    mov y+0, #low(COOLING_TEMP)
    mov y+1, #high(COOLING_TEMP)
    mov y+2, #0
    mov y+3, #0
    clr mf
    lcall x_gteq_y
    
    jb mf, FSM_s5_stay ; if temp >= cooling, stay

    ; Done
    mov Stage, #6
    mov StageSec, #0
    mov dptr, #Melody_Complete
    lcall PlayMelody
    ret
FSM_s5_stay:
    inc StageSec
    ret

    ; Stage 6: Done
FSM_s6:
    cjne a, #6, FSM_s7
    mov PWM_Duty, #0
    ret

    ; Stage 7: Error
FSM_s7:
    mov PWM_Duty, #0
    ret

FSM_enter_error:
    mov Stage, #7
    mov StageSec, #0
    mov PWM_Duty, #0
    ret

FSM_timeout_error:
    mov ErrorCode, #4
    ljmp FSM_enter_error

;============================================================
; 8. DISPLAY LOGIC (STATS) - WITH PROGRESS BAR
;============================================================

calc_progress:
    ; total_range = |t_target - t_start|
    mov   a, t_target
    clr   c
    subb  a, t_start
    jnc   tr_ok
    cpl   a
    inc   a
tr_ok:
    mov   total_range, a

    ; curr_range = |t_current - t_start|
    mov   a, t_current
    clr   c
    subb  a, t_start
    jnc   cr_ok
    cpl   a
    inc   a
cr_ok:
    mov   curr_range, a

    ; ===== bar_count = (curr_range * 12) / total_range =====
    mov   x+0, curr_range
    mov   x+1, #0
    mov   x+2, #0
    mov   x+3, #0         ; x = curr_range
    
    mov   y+0, #12
    mov   y+1, #0
    mov   y+2, #0
    mov   y+3, #0         ; y = 12
    
    lcall mul32           ; x = curr_range * 12
    
    mov   y+0, total_range
    mov   y+1, #0
    mov   y+2, #0
    mov   y+3, #0         ; y = total_range
    
    lcall div32           ; x = (curr_range * 12) / total_range
    
    mov   a, x+0
    mov   bar_count, a 
    
    ; ===== percent = (curr_range * 100) / total_range =====
    mov   x+0, curr_range
    mov   x+1, #0
    mov   x+2, #0
    mov   x+3, #0
    
    mov   y+0, #100
    mov   y+1, #0
    mov   y+2, #0
    mov   y+3, #0
    
    lcall mul32
    
    mov   y+0, total_range
    mov   y+1, #0
    mov   y+2, #0
    mov   y+3, #0
    
    lcall div32
    
    mov   a, x+0
    mov   percent, a

    ret

; Draw progress bar at current cursor position
draw_progress_bar:
    mov   bar_idx, #0

draw_bar_loop:
    mov   a, bar_idx
    clr   c
    subb  a, bar_count
    jc    bar_full

bar_empty:
    mov   a, #' '
    lcall ?WriteData
    sjmp  bar_next

bar_full:
    mov   a, #0FFh        ; Full block character
    lcall ?WriteData

bar_next:
    inc   bar_idx
    mov   a, bar_idx
    cjne  a, #12, draw_bar_loop
    ret

; Draw percent at current cursor position
draw_percent:
    mov   a, percent
    cjne  a, #100, not_100

    ; "100%"
    mov   a, #'1'
    lcall ?WriteData
    mov   a, #'0'
    lcall ?WriteData
    mov   a, #'0'
    lcall ?WriteData
    mov   a, #'%'
    lcall ?WriteData
    ret

not_100:
    ; " xx%"
    mov   a, #' '
    lcall ?WriteData
    mov   a, percent
    lcall LCD_Print2Dec_A  ; Print 2-digit number
    mov   a, #'%'
    lcall ?WriteData
    ret

;------------------------------------------------------------
; Original Display Functions
;------------------------------------------------------------

LCD_Print3Temp:
    mov b, #100
    div ab
    add a, #'0'
    lcall ?WriteData
    mov a, b
    mov b, #10
    div ab
    add a, #'0'
    lcall ?WriteData
    mov a, b
    add a, #'0'
    lcall ?WriteData
    ret

LCD_Print_Countdown_A:
    ; Input: A = Seconds remaining
    ; Output: MM:SS at current cursor
    mov b, #60
    div ab    ; A = Minutes, B = Seconds
    push b    ; Save seconds
    
    ; Print Minutes (2 digits)
    mov b, #10
    div ab
    add a, #'0'
    lcall ?WriteData
    mov a, b
    add a, #'0'
    lcall ?WriteData
    
    ; Separator
    mov a, #':'
    lcall ?WriteData
    
    ; Print Seconds
    pop acc
    mov b, #10
    div ab
    add a, #'0'
    lcall ?WriteData
    mov a, b
    add a, #'0'
    lcall ?WriteData
    ret

Refresh_Stats_LCD:
    mov a, Stage
    cjne a, #0, RS_1
    ; Stage 0 is IDLE
    Set_Cursor(1,1)
    Send_Constant_String(#Ready_Message)
    Set_Cursor(2,1)
    Send_Constant_String(#Press_Star_Msg)
    ret 
    
RS_1:
    cjne a, #1, RS_2
    ljmp FSM_Display_1
RS_2:
    cjne a, #2, RS_3
    ljmp FSM_Display_2
RS_3:
    cjne a, #3, RS_4
    ljmp FSM_Display_3
RS_4:
    cjne a, #4, RS_5
    ljmp FSM_Display_4
RS_5:
    cjne a, #5, RS_6
    ljmp FSM_Display_5
RS_6:
    cjne a, #6, RS_7
    ljmp FSM_Display_6
RS_7:
    ljmp FSM_Display_7

; Generic Display Header used by Cooling/Done
FSM_Display_Stats:
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top)
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp
    Set_Cursor(2,1)
    Send_Constant_String(#Stats_Message_Bottom)
    
    Set_Cursor(2,4)
    mov a, StageSec
    lcall LCD_Print2Dec_A
    ret

FSM_Display_1: ; Ramp to Soak
    ; Line 1: T:XXX      RPTOSK
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top) ; "T:XXX M:      "
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp

    Set_Cursor(1,11)
    Send_Constant_String(#Ramp_To_Soak)

    ; Line 2: Progress Bar + Percentage
    ; Setup progress calculation
    mov a, Ramp1_start_temp   ; Starting temperature
    mov t_start, a
    mov a, Soak_temp          ; Target temperature
    mov t_target, a
    mov x+0, Temperature+0    ; Current temperature
    mov a, x+0
    mov t_current, a
    
    ; Calculate and draw progress
    lcall calc_progress
    
    Set_Cursor(2,1)
    lcall draw_progress_bar   ; Draw 12-char bar at column 1-12
    
    Set_Cursor(2,13)
    lcall draw_percent        ; Draw " xx%" at column 13-16
    ret

FSM_Display_2: ; Soak Hold
    ; Line 1: T:XXX      SOAK
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top)
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp
    Set_Cursor(1,11)
    Send_Constant_String(#Soak)

    ; Line 2: S:XXX t:MM:SS (Target + Countdown)
    Set_Cursor(2,1)
    mov a, #'S'
    lcall ?WriteData
    mov a, #':'
    lcall ?WriteData
    mov a, Soak_temp
    lcall LCD_Print3Temp
    
    Send_Constant_String(#Msg_Time_Rem) ; " t:"
    
    ; Time is printed by FSM loop (FSM_s2)
    ret

FSM_Display_3: ; Ramp to Reflow
    ; Line 1: T:XXX      RPTORW
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top)
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp
    Set_Cursor(1,11)
    Send_Constant_String(#Ramp_To_Reflow)

    ; Line 2: Progress Bar + Percentage
    ; Setup progress calculation
    mov a, Ramp3_start_temp          ; Starting temperature
    mov t_start, a
    mov a, Reflow_temp        ; Target temperature
    mov t_target, a
    mov x+0, Temperature+0    ; Current temperature
    mov a, x+0
    mov t_current, a
    
    ; Calculate and draw progress
    lcall calc_progress
    
    Set_Cursor(2,1)
    lcall draw_progress_bar   ; Draw 12-char bar
    
    Set_Cursor(2,13)
    lcall draw_percent        ; Draw percentage
    ret

FSM_Display_4: ; Reflow Hold
    ; Line 1: T:XXX      REFLOW
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top)
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp
    Set_Cursor(1,11)
    Send_Constant_String(#Reflow)

    ; Line 2: R:XXX t:MM:SS
    Set_Cursor(2,1)
    mov a, #'R'
    lcall ?WriteData
    mov a, #':'
    lcall ?WriteData
    mov a, Reflow_temp
    lcall LCD_Print3Temp
    
    Send_Constant_String(#Msg_Time_Rem) ; " t:"
    
    ; Time is printed by FSM loop (FSM_s4)
    ret

FSM_Display_5: ; Cooling with countdown style
    ; Line 1: T:XXX      COOLIN
    Set_Cursor(1,1)
    Send_Constant_String(#Stats_Message_Top)
    Set_Cursor(1,3)
    mov x+0, Temperature+0
    mov x+1, Temperature+1
    mov a, x+0
    lcall LCD_Print3Temp
    Set_Cursor(1,11)
    Send_Constant_String(#Cooling)

    ; Line 2 Option A: Progress Bar (recommended)
    mov a, Cooling5_start_temp
    mov t_start, a
    mov a, #COOLING_TEMP
    mov t_target, a
    mov x+0, Temperature+0
    mov a, x+0
    mov t_current, a
    
    lcall calc_progress
    Set_Cursor(2,1)
    lcall draw_progress_bar
    Set_Cursor(2,13)
    lcall draw_percent
    ret
    
    ; Line 2 Option B: Show target temp + time elapsed
    ; Set_Cursor(2,1)
    ; mov a, #'C'
    ; lcall ?WriteData
    ; mov a, #':'
    ; lcall ?WriteData
    ; mov a, #COOLING_TEMP
    ; lcall LCD_Print3Temp
    ; Send_Constant_String(#Msg_Time_Rem) ; " t:"
    ; mov a, StageSec
    ; lcall LCD_Print_Countdown_A  ; Shows elapsed time
    ; ret
    
FSM_Display_6:
    lcall FSM_Display_Stats
    Set_Cursor(1,11)
    Send_Constant_String(#Done_Msg)
    ret

FSM_Display_7:
    Set_Cursor(1,1)
    mov a, #'E'
    lcall ?WriteData
    mov a, #'R'
    lcall ?WriteData
    mov a, #'R'
    lcall ?WriteData
    mov a, #':'
    lcall ?WriteData
    ; xx = ErrorCode (2-digit)
    mov a, ErrorCode
    lcall LCD_Print2Dec_A
    ; pad spaces until column 11
    mov a, #' '
    lcall ?WriteData
    lcall ?WriteData
    lcall ?WriteData
    lcall ?WriteData
    lcall ?WriteData
    Set_Cursor(1,11)
    Send_Constant_String(#Error_Word)
    Set_Cursor(2,1)
    Send_Constant_String(#Error_Line2_Msg)
    ret

;------------------------------------------------------------
; random functoins
;------------------------------------------------------------
LCD_Print3_From_BCD:
    mov a, bcd+1
    anl a, #0Fh
    add a, #'0'
    lcall ?WriteData
    mov a, bcd+0
    swap a
    anl a, #0Fh
    add a, #'0'
    lcall ?WriteData
    mov a, bcd+0
    anl a, #0Fh
    add a, #'0'
    lcall ?WriteData
    ret

Store_Soak_Temp:
    mov a, bcd+0
    anl a, #0Fh
    mov R0, a
    mov a, bcd+0
    swap a
    anl a, #0Fh
    mov R1, a
    mov a, bcd+1
    anl a, #0Fh
    mov R2, a
    mov a, R2
    mov b, #10
    mul ab
    mov b, #10
    mul ab
    mov R2, a
    mov a, R1
    mov b, #10
    mul ab
    mov R1, a
    mov a, R2
    add a, R1
    add a, R0
    clr c
    subb a, #241
    jc  Add_Soak_Temp
    mov a, #240
    sjmp Save_Soak_Temp
Add_Soak_Temp:
    add a, #241
Save_Soak_Temp:
    mov Soak_temp, a
    mov Soak_Temp16+0, a
    mov Soak_Temp16+1, #0
    ret

LCD_Show_Soak_Temp:
    Set_Cursor(1,1)
    Send_Constant_String(#Soak_Temp_Message)
    Set_Cursor(1,14)
    mov a, Soak_temp
    lcall LCD_Print3Temp
    ret

Store_Reflow_Temp:
    mov a, bcd+0
    anl a, #0Fh
    mov R0, a
    mov a, bcd+0
    swap a
    anl a, #0Fh
    mov R1, a
    mov a, bcd+1
    anl a, #0Fh
    mov R2, a
    mov a, R2
    mov b, #10
    mul ab
    mov b, #10
    mul ab
    mov R2, a
    mov a, R1
    mov b, #10
    mul ab
    mov R1, a
    mov a, R2
    add a, R1
    add a, R0
    clr c
    subb a, #241
    jc  Add_Reflow_Temp
    mov a, #240
    sjmp Save_Reflow_Temp
Add_Reflow_Temp:
    add a, #241
Save_Reflow_Temp:
    mov Reflow_temp, a
    mov Reflow_Temp16+0, a
    mov Reflow_Temp16+1, #0
    ret
    
LCD_Show_Reflow_Temp:
    Set_Cursor(1,1)
    Send_Constant_String(#Reflow_Temp_Message)
    Set_Cursor(1,14)
    mov a, Reflow_temp
    lcall LCD_Print3Temp
    ret

Get_2Digits_BCD0:
    mov a, bcd+0
    anl a, #0Fh
    mov R0, a
    mov a, bcd+0
    swap a
    anl a, #0Fh
    mov R1, a
    mov a, R1
    mov b, #10
    mul ab
    add a, R0
    ret

Clamp_59:
    clr c
    subb a, #60
    jc  C59_ok
    mov a, #59
    ret
C59_ok:
    add a, #60
    ret

LCD_Print2Dec_A:
    mov b, #10
    div ab
    add a, #'0'
    lcall ?WriteData
    mov a, b
    add a, #'0'
    lcall ?WriteData
    ret

Store_Soak_Minute:
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    mov Soak_Minute, a
    ret

Store_Soak_Second:
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    mov Soak_Second, a
    ; Update FSM Time
    mov a, Soak_Minute
    mov b, #60
    mul ab
    add a, Soak_Second
    mov Soak_TimeSec, a
    ret

LCD_Show_Soak_Time:
    Set_Cursor(1,1)
    Send_Constant_String(#Soak_Time_Message)
    Set_Cursor(1,12)
    mov a, Soak_Minute
    lcall LCD_Print2Dec_A
    Set_Cursor(1,15)
    mov a, Soak_Second
    lcall LCD_Print2Dec_A
    ret

Commit_Soak_Time_Step:
    mov a, Soak_Time_State
    cjne a, #0, CST_Seconds
CST_Minutes:
    lcall Store_Soak_Minute
    mov Soak_Time_State, #1
    Set_Cursor(1,1)
    Send_Constant_String(#Soak_Time_Message)
    Set_Cursor(1,12)
    mov a, Soak_Minute
    lcall LCD_Print2Dec_A
    Set_Cursor(1,15)
    ljmp ClearEntryAndLoop
CST_Seconds:
    lcall Store_Soak_Second
    mov Soak_Time_State, #0
    lcall LCD_Show_Soak_Time
    ljmp ClearEntryAndLoop
    
Store_Reflow_Minute:
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    mov Reflow_Minute, a
    ret

Store_Reflow_Second:
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    mov Reflow_Second, a
    ; Update FSM
    mov a, Reflow_Minute
    mov b, #60
    mul ab
    add a, Reflow_Second
    mov Reflow_TimeSec, a
    ret
    
LCD_Show_Reflow_Time:
    Set_Cursor(1,1)
    Send_Constant_String(#Reflow_Time_Message)
    Set_Cursor(1,12)
    mov a, Reflow_Minute
    lcall LCD_Print2Dec_A
    Set_Cursor(1,15)
    mov a, Reflow_Second
    lcall LCD_Print2Dec_A
    ret
    
Commit_Reflow_Time_Step:
    mov a, Reflow_Time_State
    cjne a, #0, CRT_Seconds
CRT_Minutes:
    lcall Store_Reflow_Minute
    mov Reflow_Time_State, #1
    Set_Cursor(1,1)
    Send_Constant_String(#Reflow_Time_Message)
    Set_Cursor(1,12)
    mov a, Reflow_Minute
    lcall LCD_Print2Dec_A
    Set_Cursor(1,15)
    ljmp ClearEntryAndLoop
CRT_Seconds:
    lcall Store_Reflow_Second
    mov Reflow_Time_State, #0
    lcall LCD_Show_Reflow_Time
    ljmp ClearEntryAndLoop

Commit_Soak:
    lcall Store_Soak_Temp
    lcall LCD_Show_Soak_Temp
    sjmp ClearEntryAndLoop

Commit_Reflow:
    lcall Store_Reflow_Temp
    lcall LCD_Show_Reflow_Temp
    sjmp ClearEntryAndLoop

ClearEntryAndLoop:
    clr a
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a
    mov bcd+4, a
    ljmp forever
    
UpdateStage_LEDs:
    mov a, Stage
    jz  USL_Off       ; If Stage 0, LEDs off
    
    ; Create a bitmask based on stage
    mov b, a          
    mov a, #01H       
    dec b             
    
    mov R0, b         
    cjne R0, #0, USL_Loop
    sjmp USL_Write

USL_Loop:
    rl a              
    djnz R0, USL_Loop

USL_Write:
    mov LEDRA, a      
    ret

USL_Off:
    mov LEDRA, #00H
    ret
    
FSM_Display_Stats_Blank:
    Set_Cursor(1, 1)
    Send_Constant_String(#Stats_Message_Top)    ; Prints "T:XXX M:      "
    Set_Cursor(2, 1)
    Send_Constant_String(#Stats_Message_Bottom) ; Prints "S: XX:XX        "
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

;------------------------------------------------------------
; buzzer shit
;------------------------------------------------------------

Timer1_Init_Tone:
    ; Timer1 mode 1 (16-bit)
    mov a, TMOD
    anl a, #0Fh
    orl a, #10h
    mov TMOD, a

    mov TH1, #high(TONE0P5_RELOAD)
    mov TL1, #low(TONE0P5_RELOAD)

    clr TR1
    clr ET1
    ret

StartTone:
    mov TH1, Current_Note_H
    mov TL1, Current_Note_L
    setb ET1
    setb TR1
    ret

StopTone:
    clr TR1
    clr ET1
    clr SPEAKER
    ret

Timer1_ISR:
    push acc
    push psw

    mov TH1, Current_Note_H
   
    mov TL1, Current_Note_L
    cpl SPEAKER          ; toggle every 0.5ms -> 1kHz square wave

    pop psw
    pop acc
    reti

;------------------------------------------------------------
; main
;------------------------------------------------------------
main_code:
    mov SP, #7FH
    clr a
    mov LEDRA, a
    mov LEDRB, a
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a
    mov bcd+4, a
    mov UI_State, #0
    mov Soak_Time_State, a
    mov Reflow_Time_State, a
    mov Controller_Mode, #0
    mov P0MOD, #10101010b
    mov P1MOD, #10000010b
    mov P2MOD, #11111111b
    mov P3MOD, #11111111b
    mov Stage,     #0
    mov PrevStage, #0FFh
    mov StageSec,  #0
    mov ErrorCode, #0
    mov Timer_Run, #1
    clr EmergencyLatched
    mov Melody_Playing, #0
    mov Melody_Index, #0

    ; --- FIX: INITIALIZE DEFAULTS CORRECTLY ---
    ; Default Soak: 150C, 1min 30sec (90s)
    mov Soak_temp, #150
    mov Soak_Minute, #1
    mov Soak_Second, #30
    
    mov Soak_Temp16+0, #low(150)
    mov Soak_Temp16+1, #high(150)
    mov Soak_TimeSec,  #90

    ; Default Reflow: 220C, 0min 45sec (45s)
    mov Reflow_temp, #220
    mov Reflow_Minute, #0
    mov Reflow_Second, #45
    
    mov Reflow_Temp16+0, #low(220)
    mov Reflow_Temp16+1, #high(220)
    mov Reflow_TimeSec,  #45
    ; ------------------------------------------
    
    lcall ELCD_4BIT 
    lcall Configure_Keypad_Pins     
    lcall Timer0_Init
	lcall Timer1_Init_Tone
    lcall InitSerialPort
    
    ; Initial Read to stabilize ADC
    lcall Temperature_Check
    lcall Wait50ms
    lcall Temperature_Check
    
    ; Show Default Screen
    Set_Cursor(1,1)
    Send_Constant_String(#Ready_Message)
    Set_Cursor(2,1)
    Send_Constant_String(#Press_Star_Msg)
    
forever:
    ; 1. HANDLE PERIODIC TASKS (1Hz)
    jnb onesec, handle_keys
    clr onesec
    
    ; Update Temperature
    lcall Temperature_Check
    
    lcall UART_SendTemperature
    
    ; Run State Machine ONLY if not paused
    mov a, Timer_Run
    jz  skip_fsm
    lcall FSM1_Reflow
skip_fsm:
    
    ; Update LCD (Only if in Run/Stats mode)
    mov a, UI_State
    cjne a, #1, handle_keys
    lcall Refresh_Stats_LCD

handle_keys:
    jb EmergencyLatched, NoKeyPressed
    ; 2. SCAN KEYPAD
    lcall Keypad
    lcall Display
    jnc NoKeyPressed
    sjmp GotKey
NoKeyPressed:
    ljmp forever

GotKey:
	mov a, #1                    ; 1¸öbeep
  
    lcall StartBuzzer
    ; --- STAR '*' (0x0E) : Start / Pause FSM ---
    mov a, R7
    cjne a, #0EH, GK_NotStar

    ; ???? Stage=0(Idle),? * ???
    mov a, Stage
    jnz Star_TogglePause

Star_Start:
    mov Stage,    #1
    mov StageSec, #0
    mov PrevStage,#0FFh
    mov Timer_Run,#1        ; ??????
    
    mov a, Temperature+0
    mov Ramp1_start_temp, a

    ; ??:??????? stats ???(?????????)
    mov UI_State, #1

    lcall Refresh_Stats_LCD
    ljmp ClearEntryAndLoop

Star_TogglePause:
    ; Timer_Run = Timer_Run XOR 1 (? CPL + mask ??)
    mov a, Timer_Run
    cpl a
    anl a, #01h
    mov Timer_Run, a

    ; ???:??? SSR/PWM
    mov a, Timer_Run
    jnz Star_Resume

Star_Pause:
    mov PWM_Duty, #0
    clr SSR_OUT            ; ???? SSR
    lcall Refresh_Stats_LCD
    ljmp ClearEntryAndLoop

Star_Resume:
    mov UI_State, #1

    lcall Refresh_Stats_LCD
    ljmp ClearEntryAndLoop

GK_NotStar:
    ; Ignore other keys if in Stats Mode
    mov a, UI_State
    cjne a, #1, GK_NotStats
    ljmp forever

GK_NotStats:
    ; --- HASH '#' (0x0F) : Commit ---
    mov a, R7
    cjne a, #0FH, GK_NotHash
    mov a, Controller_Mode
    cjne a, #0, GK_H1
    ljmp Commit_Soak
GK_H1:
    cjne a, #1, GK_H2
    ljmp Commit_Reflow
GK_H2:
    cjne a, #2, GK_H3
    ljmp Commit_Soak_Time_Step
GK_H3:
    cjne a, #3, GK_HDone
    ljmp Commit_Reflow_Time_Step
GK_HDone:
    ljmp forever

GK_NotHash:
    ; --- MODES (A,B,C,D) ---
    ljmp Key_B

Key_B:
    mov a, R7
    cjne a, #0BH, Key_C
    mov Controller_Mode, #2
    lcall LCD_Show_Soak_Time
    
    ; Logic to place cursor
    mov a, Soak_Time_State
    jz  KB_Min
KB_Sec:
    Set_Cursor(1,15)
    sjmp KB_Done
KB_Min:
    Set_Cursor(1,12)
KB_Done:
    ljmp ClearEntryAndLoop

Key_C:
    mov a, R7
    cjne a, #0CH, Key_A
    mov Controller_Mode, #1
    lcall LCD_Show_Reflow_Temp
    ljmp ClearEntryAndLoop
    
Key_A:
    mov a, R7
    cjne a, #0AH, Key_D        
    mov Controller_Mode, #0
    lcall LCD_Show_Soak_Temp
    ljmp ClearEntryAndLoop

Key_D:
    mov a, R7
    cjne a, #0DH, Key_Digit
    mov Controller_Mode, #3
    lcall LCD_Show_Reflow_Time
    
    mov a, Reflow_Time_State
    jz  KD_Min
KD_Sec:
    Set_Cursor(1,15)
    sjmp KD_Done
KD_Min:
    Set_Cursor(1,12)
KD_Done:
    ljmp ClearEntryAndLoop

Key_Digit:
    mov a, R7
    clr c
    subb a, #0Ah
    jc  KD_IsDigit
    ljmp forever ; Not a digit

KD_IsDigit:
    lcall Shift_Digits_Left
    mov a, Controller_Mode
    
    ; Live Typing Display Logic
    cjne a, #0, TD1
        Set_Cursor(1,14)
        lcall LCD_Print3_From_BCD
        ljmp forever
TD1:
    cjne a, #1, TD2
        Set_Cursor(1,14)
        lcall LCD_Print3_From_BCD
        ljmp forever
TD2:
    cjne a, #2, TD3
    mov a, Soak_Time_State
    jz  TD2_Minutes
TD2_Seconds:
    Set_Cursor(1,15)
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    lcall LCD_Print2Dec_A
    ljmp forever
TD2_Minutes:
    Set_Cursor(1,12)
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    lcall LCD_Print2Dec_A
    ljmp forever
TD3:
    cjne a, #3, TD_End    
    mov a, Reflow_Time_State
    jz  TD3_Minutes
TD3_Seconds:
    Set_Cursor(1,15)
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    lcall LCD_Print2Dec_A
    ljmp forever
TD3_Minutes:
    Set_Cursor(1,12)
    lcall Get_2Digits_BCD0
    lcall Clamp_59
    lcall LCD_Print2Dec_A
    ljmp forever
TD_End:
    ljmp forever

end