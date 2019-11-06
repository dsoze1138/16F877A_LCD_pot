 list n=0,c=255,r=dec
 errorlevel -302 ; Suppress Register in operand not in bank 0. warning
;
; File: main.asm
; Target: PIC16F877A
;
; Description:
;
;  Homework Problem: Keep everything the same as in Homework #1,
;  design a system that can read the potentiometer's settings and
;  display the real-time voltage ranges of High, Medium, and Low
;  on the LCD module. Your program should display the real-time
;  voltage of High=4.5 V and above, Medium=between 2.0V and 3.0V,
;  and Low=1.5V and below and update the reading and display
;  every 5 seconds to reflect the changes on the potentiometer.
;  If the voltage in not in the ranges, then display "Unknown" on the LCD.
;
;  I've gotten the ASM to do everything but to be able to do
;  "greater than," "less than," for the ranges. I can see how to
;  do this in C-code for the PIC, but not in ASM. Any help would
;  be appreciated.
;
;  Board: uC Training System Manual Rev. 3 from ODU
;  Microchip: PIC16F877A
;  Coding Language: ASM within MPLAB (MPASM Assembler v5.50)
;
; https://stackoverflow.com/questions/58376434/eet-470-asm-code-designs-adc-conversions-and-voltage-ranges-display-on-a-lcd
;
;                         PIC16F877A
;                 +----------:_:----------+
;        S3 ->  1 : MCLR/VPP      PGD/RB7 : 40 <> LCD_D7
;           ->  2 : RA0/AN0       PGC/RB6 : 39 <> LCD_D6
;           ->  3 : RA1/AN1           RB5 : 38 <> LCD_D5
;           ->  4 : RA2/AN2           RB4 : 37 <> LCD_D4
;           ->  5 : RA3/AN3       PGM/RB3 : 36 <> LCD_D3
;           <>  6 : RA4               RB2 : 35 <> LCD_D2
;           ->  7 : RA5/AN4           RB1 : 34 <> LCD_D1
;           ->  8 : RE0/AN5           RB0 : 33 <> LCD_D0
;           ->  9 : RE1/AN6           VDD : 32 <- 5v0
;      POT1 -> 10 : RE2/AN7           VSS : 31 <- GND
;       5v0 -> 11 : VDD               RD7 : 30 ->
;       GND -> 12 : VSS               RD6 : 29 ->
;    20 MHz -> 13 : OSC1              RD5 : 28 ->
;    20 MHz <- 14 : OSC2              RD4 : 27 ->
;    LCD_RS <> 15 : RC0/SOSCO   RX/DT/RC7 : 26 <>
;    LCD_E  <> 16 : RC1/SOSCI   TX/CK/RC6 : 25 <>
;           <> 17 : RC2/CCP1          RC5 : 24 <>
;           <> 18 : RC3/SCL       SDA/RC4 : 23 <>
;           <> 19 : RD0               RD3 : 22 <>
;           <> 20 : RD1               RD2 : 21 <>
;                 +-----------------------:
;                          DIP-40
;
;*****************************************************************
;
; Include names of the Special Function Registers(SFR)
; Set the configuration word bits for this application
;
;*****************************************************************
    #include <P16F877A.INC>

    __CONFIG  ( _FOSC_HS & _WDTE_OFF & _PWRTE_ON & _BOREN_OFF & _LVP_OFF & _CPD_OFF & _WRT_OFF & _CP_OFF)
;
;*****************************************************************
;
; Application constants
;
;*****************************************************************
#define FSYS (20000000)
#define FCYC ( 5000000)
#define TMR0_TICKS_PER_SECOND (FCYC/65536)
#define DISPLAY_UPDATE_SECONDS (5)
#define VREF_IN_TENTHS_OF_VOLTS (50)
#define LCD_LINE_ONE    0x80
#define LCD_LINE_TWO    0xC0
#define LCD_LINE_THREE  0x94
#define LCD_LINE_FOUR   0xD4
;
;*****************************************************************
;
; Define macros to help with bank selection
;
;*****************************************************************
#define BANK0  (h'000')
#define BANK1  (h'080')
#define BANK2  (h'100')
#define BANK3  (h'180')
;
;*****************************************************************
;
; Declare the symbols used for read/write data
;
;*****************************************************************
  cblock 0x20       ; memory present in bank 0
    TMR0_Ticks: 1
    SecondsUntilUpdate:1
    LCD_Temp:2
  endc

  cblock 0x70       ; memory present in all banks
    WREG_save   : 1
    STATUS_save : 1
    PCLATH_save : 1
    Product_8x8 : 2
    Bits_8x8    : 1
  endc
;
;*****************************************************************
;
; Power on reset vector
;
;*****************************************************************
    ORG     0x0000                  ;RESET or WDT reset vector
    GOTO    START
;
;*****************************************************************
;
; Interrupt vector
;
;*****************************************************************
    ORG     0x0004                  ;Regular INT vector
    movwf   WREG_save               ; Save register that
    movf    STATUS,W                ; must be restored to
    movwf   STATUS_save             ; preserve the context
    movf    PCLATH,W                ; of the interrupted
    movwf   PCLATH_save             ; code.
    clrf    STATUS                  ; Use BANK 0 as default
    clrf    PCLATH                  ; Use PAGE 0 as default

;This is the area where your interrupt service routine goes

    movf    PCLATH_save,W
    movwf   PCLATH
    movf    STATUS_save,W
    movwf   STATUS
    swapf   WREG_save,F
    swapf   WREG_save,W
    retfie
;
;*****************************************************************
;
; Main application start
;
;*****************************************************************
START:                              ; Initialize the PIC hardware
    clrf    INTCON                  ; Disable all interrupt sources

    banksel BANK1
    clrf    PIE1
    clrf    PIE2
    movlw   0x07                    ;
    movwf   TRISE                   ; Set RE0/AN5,RE1/AN6 and RE2/AN7 as inputs
    movlw   0x2F
    movwf   TRISA                   ; Set RA0/AN0,RA1/AN1,RA2/AN2,RA3/AN3 and RA5/AN4 as inputs, RA4 as output
    clrf    TRISB                   ; Clear TRISB, set PORTB as all outputs
    clrf    TRISC                   ; Clear TRISC, set PORTC as all outputs
    clrf    TRISD                   ; Clear TRISD, set PORTD as all outputs
    movlw   0x07
    movwf   CMCON                   ; Turn off comparators
    clrf    ADCON1                  ; Make AN0-AN7 analog inputs, Left justified result.
    movlw   0xC7                    ; TIMER0 clock source FOSC/4, prescale 1:256
    movwf   OPTION_REG

    banksel BANK0
    clrf    PORTA                   ; Make all outputs low
    clrf    PORTB
    clrf    PORTC
    clrf    PORTD
    movlw   0xB9
    movwf   ADCON0                  ; Set ADC clock to FOSC/32, AN7 as input, turn ADC on.

    clrf    TMR0_Ticks              ; Setup TIMER0 to count seconds
    clrf    SecondsUntilUpdate
    clrf    TMR0
    bcf     INTCON,T0IF
;
;*****************************************************************
;
; Main application processing loop initialization
;
;*****************************************************************
    call    LCD_Initialize

    movlw   LCD_LINE_ONE
    call    LCD_WriteCommand        ; Set LCD at start of line one

    banksel EEADR
    movlw   LOW(LCD_StartMessage)
    movwf   EEADR
    movlw   HIGH(LCD_StartMessage)
    movwf   EEADRH
    call    LCD_WriteStringFromROM  ; Display application title text
;
;*****************************************************************
;
; Main application processing loop
;
;*****************************************************************
main:
    banksel BANK0

    btfsc   INTCON,T0IF             ; Check elapse time and update
    call    Update_TMR0_Ticks       ; timeouts that take seconds.

    movf    SecondsUntilUpdate,F    ; Check if time to refresh display.
    btfss   STATUS,Z                ; Skip when display refresh is required.
    goto    main
;
;*****************************************************************
;
; Update LCD character display module
;
;*****************************************************************
    movlw   DISPLAY_UPDATE_SECONDS  ; set delay for next update
    movwf   SecondsUntilUpdate

    movlw   LCD_LINE_TWO
    call    LCD_WriteCommand        ; Set LCD at start of line two
    banksel EEADR
    movlw   LOW(LCD_Pot_Value)
    movwf   EEADR
    movlw   HIGH(LCD_Pot_Value)
    movwf   EEADRH
    call    LCD_WriteStringFromROM  ; Display type of output text
    banksel BANK0
;
;*****************************************************************
;
; Read voltage of POT1 and scale for a 0 to VREF values in
; tenths of volts. When VREF is 5.0 volts the value is 0 to 50.
;
;*****************************************************************
    call    ADC_Read                ; Read voltage of POT1
    movwf   Product_8x8
    movlw   VREF_IN_TENTHS_OF_VOLTS
    call    Mul_8x8                 ; Scale to the ADC VREF in tenths of volts
    movlw   d'128'
    addwf   Product_8x8,F           ; Round up
    btfsc   STATUS,C
    incf    Product_8x8+1,F         ; Product_8x8+1 = POT1 voltage in range 0 to 50 tenths of volts

    movlw   LCD_LINE_TWO+2
    call    LCD_WriteCommand        ; Set LCD to where volts are shown
    movf    Product_8x8+1,W
    call    LCD_WriteVolts          ; Show voltage from POT1

    movlw   LCD_LINE_TWO+8
    call    LCD_WriteCommand        ; Set LCD to where results are shown
;
;*****************************************************************
;
; Display the required outputs of High, Mid, Low and Unknown
;
;*****************************************************************
    movf    Product_8x8+1,W
    sublw   d'45'
    btfss   STATUS,Z                ; ZERO  is set when [Product_8x8+1] == CONSTANT
    btfss   STATUS,C                ; CARRY is set when [Product_8x8+1] <  CONSTANT
    goto    Display_High

    movf    Product_8x8+1,W
    sublw   d'30'                   ; CARRY is set when [Product_8x8+1] <  CONSTANT
    btfss   STATUS,C
    goto    Display_Unknown

    movf    Product_8x8+1,W
    sublw   d'20'
    btfss   STATUS,Z                ; ZERO  is set when [Product_8x8+1] == CONSTANT
    btfss   STATUS,C                ; CARRY is set when [Product_8x8+1] <  CONSTANT
    goto    Display_Mid

    movf    Product_8x8+1,W
    sublw   d'15'
    btfss   STATUS,C                ; CARRY is set when [Product_8x8+1] <  CONSTANT
    goto    Display_Unknown
    goto    Display_Low

Display_Low:
    banksel EEADR
    movlw   LOW(LCD_Low)
    movwf   EEADR
    movlw   HIGH(LCD_Low)
    movwf   EEADRH
    goto    Show_POT1_state

Display_Mid:
    banksel EEADR
    movlw   LOW(LCD_Mid)
    movwf   EEADR
    movlw   HIGH(LCD_Mid)
    movwf   EEADRH
    goto    Show_POT1_state

Display_High:
    banksel EEADR
    movlw   LOW(LCD_High)
    movwf   EEADR
    movlw   HIGH(LCD_High)
    movwf   EEADRH
    goto    Show_POT1_state

Display_Unknown:
    banksel EEADR
    movlw   LOW(LCD_Unknown)
    movwf   EEADR
    movlw   HIGH(LCD_Unknown)
    movwf   EEADRH
Show_POT1_state:
    call    LCD_WriteStringFromROM
    goto    main
;
;*****************************************************************
; Subroutine Name: Update_TMR0_Ticks
; Function: Update tick count of TIMER0 events to count elapse seconds.
;           Decrement count in seconds for the display update.
;
;*****************************************************************
Update_TMR0_Ticks:
    bcf     INTCON,T0IF
    banksel TMR0_Ticks
    incf    TMR0_Ticks,F
    movlw   TMR0_TICKS_PER_SECOND   ; Calculate: TMR0_Ticks - TMR0_TICKS_PER_SECOND
    subwf   TMR0_Ticks,W            ; CARRY is set a NOT BORROW on subtract
    btfss   STATUS,C                ; So CARRY is set when TMR0_Ticks < TMR0_TICKS_PER_SECOND
    return

    call    ADC_Read                ; Read voltage of POT1
    movwf   Product_8x8
    movlw   VREF_IN_TENTHS_OF_VOLTS
    call    Mul_8x8                 ; Scale to the ADC VREF in tenths of volts
    movlw   d'128'
    addwf   Product_8x8,F           ; Round up
    btfsc   STATUS,C
    incf    Product_8x8+1,F         ; Product_8x8+1 = POT1 voltage in range 0 to 50 tenths of volts

    movlw   LCD_LINE_TWO+2
    call    LCD_WriteCommand        ; Set LCD to where volts are shown
    movf    Product_8x8+1,W
    call    LCD_WriteVolts          ; Show voltage from POT1

    clrf    TMR0_Ticks
    movf    SecondsUntilUpdate,F    ; Count down seconds until a
    btfss   STATUS,Z                ; display refresh is required.
    decf    SecondsUntilUpdate,F
    return
;
;*****************************************************************
; Subroutine Name: ADC_Read
; Function: Select ADC channel to convert
;           Wait for sample and hold setup time
;           Start ADC conversion
;           Wait for conversion to complete
;           Return ADRESH in the WREG
;
; Inputs:  none
;
; Outputs: WREG is 8-bit ADC data
;
; Author:
; Date:2019-10-18
;*****************************************************************
ADC_Read:
    banksel ADCON0
    bsf     ADCON0,CHS2             ; Select RE2/AN7 as ADC input
    bsf     ADCON0,CHS1
    bsf     ADCON0,CHS0
    call    Delay_40us              ; Wait for sample and hold capacitor to charge.
    bsf     ADCON0,GO
    nop
    nop
ADC_Wait:
    btfsc   ADCON0,GO               ; skip when conversion is complete
    goto    ADC_Wait
    movf    ADRESH,W                ; Get ADC conversion value
    return
;
;*****************************************************************
; Subroutine Name: Mul_8x8
; Function: Multiply two 8-bit values and provide a 16-bit product
;
; Inputs:  Product_8x8 (low 8-bits) is multiplier
;          WREG is multiplicand
;
; Outputs: Product_8x8 (16-bits) is the product
;
; Author:
; Date:2019-10-18
;*****************************************************************
Mul_8x8:
    clrf    Bits_8x8
    bsf     Bits_8x8,3
    clrf    Product_8x8+1
    rrf     Product_8x8+0,F
M_8x8_Loop:
    btfsc   STATUS,C
    addwf   Product_8x8+1,F
    rrf     Product_8x8+1,F
    rrf     Product_8x8+0,F
    decfsz  Bits_8x8,F
    goto    M_8x8_Loop
    return
;
;*****************************************************************
; Subroutine Name: Delay_4us, Delay_20us, Delay_40us
; Function: provides a delay for 4, 20 or 40 microseconds
;           Code relies on a 20MHz system oscillator
;
; Inputs:  none
;
; Outputs: none
;
; Author:
; Date:2019-10-18
;*****************************************************************
Delay_4us:
    goto    $+1
    goto    Dly2
Delay_40us:
    call    Delay_20us              ; Wait at least 40 microseconds
Delay_20us:                         ; for command to complete.
    call    Delay_4us
    call    Delay_4us
    call    Delay_4us
    call    Delay_4us
    goto    Delay_4us
;
;*****************************************************************
; Subroutine Name: Delay_5ms
; Function: Provides a delay for at least 5 milliseconds
;           Code relies on a 20MHz system oscillator
;
; Inputs:  none
;
; Outputs: none
;
; Uses:    WREG, STATUS
;
; Author:
; Date:2019-10-18
;*****************************************************************
Delay_5ms:
    call    Dly0
    call    Dly0
    call    Dly0
    call    Dly0
Dly0:
    goto    $+1
    goto    $+1
    movlw   d'249'
Dly1:
    call    Dly2
    addlw   -1
    bnz     Dly1
Dly2:
    goto    $+1
    goto    $+1
    goto    $+1
    goto    $+1
    goto    $+1
    goto    $+1
    return
;
;*****************************************************************
; Subroutine Name: LCD_POR_Delay
; Function: provides a delay for at least 15 milliseconds
;           Code relies on a 20MHz system oscillator
;
; Inputs:  none
;
; Outputs: none
;
; Uses:    WREG, STATUS
;
; Author:
; Date:2019-10-18
;*****************************************************************
LCD_POR_Delay:
    call    Delay_5ms
    call    Delay_5ms
    goto    Delay_5ms
;
;*****************************************************************
; Subroutine Name: LCD_WriteCommand
; Function: Set LCD_RS to command mode
;           Write command byte to PORTB
;           Pulse the LCD_E high for 4 microseconds
;           Wait for 40 microseconds
;
; Inputs:  WREG     Command to be sent to LCD module
;
; Outputs: WREG     Command sent to LCD module
;
; Author:
; Date:2019-10-19
;*****************************************************************
LCD_WriteCommand:
    banksel PORTC
    bcf     PORTC,0                 ; Assert LCD_RS low
    goto    LCD_Write1
;
;*****************************************************************
; Subroutine Name: LCD_WriteData
; Function: Set LCD_RS to data mode
;           Write command byte to PORTB
;           Pulse the LCD_E high for 4 microseconds
;           Wait for 40 microseconds
;
; Inputs:  WREG     Data to be sent to LCD module
;
; Outputs: WREG     Data sent to LCD module
;
; Author:
; Date:2019-10-19
;*****************************************************************
LCD_WriteData:
    banksel PORTC
    bsf     PORTC,0                 ; Assert LCD_RS high
LCD_Write1:
    movwf   PORTB
    bsf     PORTC,1                 ; Assert LCD_E high
    call    Delay_4us
    bcf     PORTC,1                 ; Assert LCD_E low
    goto    Delay_40us
;
;*****************************************************************
; Subroutine Name: LCD_WriteVolts
; Function:  Display dd.d on LCD, range 0.0 to 25.5
;
;
;
; Inputs:  WREG value to be displayed as three decimal digits
;
; Outputs:
;
; Author:
; Date:2019-10-20
;*****************************************************************
LCD_WriteVolts:
    banksel LCD_Temp
    clrf    LCD_Temp
    movwf   LCD_Temp+1
    movlw   d'200'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,1
    btfsc   STATUS,C
    movwf   LCD_Temp+1
    movlw   d'100'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,0
    btfsc   STATUS,C
    movwf   LCD_Temp+1

    movf    LCD_Temp,F
    movlw   ' '
    btfss   STATUS,Z
    movlw   '0'
    iorwf   LCD_Temp,F
    movf    LCD_Temp,W
    call    LCD_WriteData
    movlw   '0'
    andwf   LCD_Temp,F

    bsf     LCD_Temp,5              ; Do not do leading zero blanking from this digit onward.

    movlw   d'80'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,3
    btfsc   STATUS,C
    movwf   LCD_Temp+1

    movlw   d'40'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,2
    btfsc   STATUS,C
    movwf   LCD_Temp+1

    movlw   d'20'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,1
    btfsc   STATUS,C
    movwf   LCD_Temp+1

    movlw   d'10'
    subwf   LCD_Temp+1,W            ; CARRY is set when Input >= CONSTANT
    btfsc   STATUS,C
    bsf     LCD_Temp,0
    btfsc   STATUS,C
    movwf   LCD_Temp+1

    movf    LCD_Temp,F
    movlw   ' '
    btfss   STATUS,Z
    movlw   '0'
    iorwf   LCD_Temp,F
    movf    LCD_Temp,W
    call    LCD_WriteData
    movlw   '0'
    andwf   LCD_Temp,F

    movlw   '.'                     ; Print a decimal point
    call    LCD_WriteData

    movlw   '0'
    addwf   LCD_Temp+1,W            ; Restore input value
    call    LCD_WriteData           ; Print last digit

    return
;
;*****************************************************************
; Subroutine Name: LCD_WriteStringFromROM
; Function: Display zero terminated string stored in code space
;           on the LCD module. Use the "compact" ASCII string
;           directive DA available in the MPASMWIN assembler
;           to declare the text.
;
; Inputs:  EEADRH:EEADR pointer to ASCIIZ string in ROM
;
; Outputs: EEADRH:EEADR points to word after end of ASCIIZ string
;
; Author:
; Date:2019-10-19
;*****************************************************************
LCD_WriteStringFromROM:
    banksel EECON1
    bsf     EECON1,EEPGD    ; Read 14-bit CODE space word of ASCII 128 string
    bsf     EECON1,RD
    nop                     ; Requires two NOP's
    nop                     ; to read code space.
    banksel EEADR
    incf    EEADR,F         ; Increment EEROM pointer
    skpnz
    incf    EEADRH,F
    banksel EEDATA
    rlf     EEDATA,W        ; Compose high 7-bit ASCIIZ data
    rlf     EEDATH,W
    andlw   0x7F
    btfsc   STATUS,Z
    return                  ; Exit if End Of String at high byte.
    call    LCD_WriteData
    banksel EEDATA
    movf    EEDATA,W        ; Get low 7-bits of ASCIIZ data
    andlw   0x7F
    btfsc   STATUS,Z
    return                  ; Exit if End Of String at low byte.
    call    LCD_WriteData
    goto    LCD_WriteStringFromROM
;
;*****************************************************************
; Subroutine Name: LCD_Initialize
; Function: Wait for Power-On-Reset time, 30 milliseconds.
;           Send command bytes to initialize the LCD module
;           for a 2-line display, no cursor, left justified.
;
; Inputs:  none
;
; Outputs: none
;
; Author:
; Date:2019-10-19
;*****************************************************************
LCD_Initialize:
    call    LCD_POR_Delay           ; Wait 30 milliseconds for LCD
    call    LCD_POR_Delay           ; to complete power up.
    movlw   0x30                    ; LCD command for 8-bit mode.
    call    LCD_WriteCommand        ; Sending this command
    call    LCD_WriteCommand        ; three times puts the
    call    LCD_WriteCommand        ; module in 8-bit mode.
    movlw   0x38                    ; Set module for 8-bit mode, 2 lines 5x8 font
    call    LCD_WriteCommand
    movlw   0x08                    ; Display off
    call    LCD_WriteCommand
    movlw   0x01                    ; Clear display
    call    LCD_WriteCommand
    call    Delay_5ms               ; This command takes awhile.
    movlw   0x06                    ; Cursor increments, No shift
    call    LCD_WriteCommand
    movlw   0x0C                    ; Display on, Cursor off, Blink off
    call    LCD_WriteCommand
    return
;
;*****************************************************************
;
; Declare constant data in code space that are ASCIIZ string
; messages that are displayed on the LCD module.
;
;*****************************************************************
LCD_StartMessage:
    da  "Homework #2",0
LCD_High:
    da  "High",0
LCD_Mid:
    da  "Mid",0
LCD_Low:
    da  "Low",0
LCD_Unknown:
    da  "Unknown",0
LCD_Pot_Value:
    da  "V=    :         ",0
    END                             ;End program
