
; CC5X Version 3.2, Copyright (c) B Knudsen Data
; C compiler for the PICmicro family
; ************   2. Sep 2011  12:59  *************

	processor  16F627A
	radix  DEC

INDF        EQU   0x00
STATUS      EQU   0x03
FSR         EQU   0x04
PORTA       EQU   0x05
TRISA       EQU   0x85
PORTB       EQU   0x06
TRISB       EQU   0x86
INTCON      EQU   0x0B
Carry       EQU   0
Zero_       EQU   2
RP0         EQU   5
RP1         EQU   6
IRP         EQU   7
OPTION_REG  EQU   0x81
PIR1        EQU   0x0C
T2CON       EQU   0x12
CMCON       EQU   0x1F
PIE1        EQU   0x8C
PCON        EQU   0x8E
PR2         EQU   0x92
RA0         EQU   0
RA1         EQU   1
RA2         EQU   2
RA3         EQU   3
RA6         EQU   6
RA7         EQU   7
RB4         EQU   4
RB5         EQU   5
TMR2IF      EQU   1
FSRTemp     EQU   0x32
T2ms        EQU   0x33
T10ms       EQU   0x34
T100ms      EQU   0x35
BlinkCounter EQU   0x36
BlinkState  EQU   0x37
DigitCounter EQU   0x38
DispValue   EQU   0x39
Digit       EQU   0x3B
SecondsFlag EQU   0x3F
StartButtonState EQU   0x40
StartButtonPrev EQU   0x41
StartButtonCounter EQU   0x42
StopButtonState EQU   0x43
StopButtonPrev EQU   0x44
StopButtonCounter EQU   0x45
State1      EQU   0x46
svrWREG     EQU   0x70
svrSTATUS   EQU   0x20
Seconds     EQU   0x27
Minutes     EQU   0x28
Timing      EQU   0x29
START       EQU   0x2A
STOP        EQU   0x2B
StartLock   EQU   0x2C
StopLock    EQU   0x2D
temp        EQU   0x2E
DigitCounter_2 EQU   0x21
temp_2      EQU   0x21
C1cnt       EQU   0x23
C2tmp       EQU   0x24
C3cnt       EQU   0x23
C4tmp       EQU   0x24
C5rem       EQU   0x26
C6cnt       EQU   0x23
C7tmp       EQU   0x24
C8cnt       EQU   0x23
C9tmp       EQU   0x24
C10cnt      EQU   0x23
C11tmp      EQU   0x24
C12rem      EQU   0x26
C13cnt      EQU   0x23
C14tmp      EQU   0x24
test        EQU   0x2F
state       EQU   0x30
counter     EQU   0x31

	GOTO main

  ; FILE TIMER.c
			;//#define   PIC16F627A
			;
			;//#include "16F627A.H"
			;#include "INT16CXX.H"
			;
			;#pragma origin = 0x0004																	// CHECK THIS IN DATASHEET.  WHAT 0x0004 ???
	ORG 0x0004
			;#pragma config        = 0b0011.1111.0101.1000   	// MCLR Pin is input
			;//#pragma config        = 0b0011.1111.0111.1000   	// MCLR Pin is input
			;
			;#define BCD_A			RB0		// OUTPUT, BIT0 BCD WORD
			;#define BCD_B      		RB1   	// OUTPUT, BIT1 BCD WORD  
			;#define BCD_C			RB2   	// OUTPUT, BIT2 BCD WORD		
			;#define BCD_D	 	 	RB3   	// OUTPUT, BIT3 BCD WORD		
			;#define MC14543_LD_CTRL	RB5		// Rising Edge Load to BCD DRiver
			;
			;#define DISP0			RA6
			;#define DISP1			RB4
			;#define DISP2			RA3	
			;#define DISP3			RA2
			;
			;#define DECIMAL			RA7
			;
			;#define StartButton		RA0
			;#define StopButton		RA1
			;
			;#define MAX_DEBOUNCE_COUNT 	10			// How many times do we need the changed state = 100 ms
			;#define STATE_STOPPED 		0
			;#define STATE_STARTED 		1
			;#define STATE_PAUSED		2
			;
			;#define MaxBlink			100			// blink every 2ms * MaxBlink
			;
			;
			;// Declare Function Prototypes
			;void InitialiseHardware(void);
			;void InitialiseGlobals(void);
			;void DisplayDigit(char DigitCounter);
			;void Dec2BCD(void);
			;void debounce(char test, char *state, char *counter);
			;
			;char FSRTemp;
			;char T2ms;								// 2 millisecond counter
			;char T10ms;								// 10's millisecond counter
			;char T100ms;							// 100's millisecond counter
			;
			;char BlinkCounter;						// increment every 2 millisecond, allows blinking when paused in mutliples of 2ms
			;char BlinkState;						// might be needed
			;
			;char DigitCounter;
			;uns16 DispValue;		// unsigned 16 bit integer
			;char Digit[4];
			;char SecondsFlag;
			;char StartButtonState;
			;char StartButtonPrev;
			;char StartButtonCounter;
			;char StopButtonState;
			;char StopButtonPrev;
			;char StopButtonCounter;
			;char State1;
			;
			;#pragma codepage 0
			;interrupt IntHandler() {
IntHandler
			;   	int_save_registers
	MOVWF svrWREG
	SWAPF STATUS,W
	BCF   0x03,RP0
	BCF   0x03,RP1
	MOVWF svrSTATUS
			;   	FSRTemp = FSR;
	MOVF  FSR,W
	MOVWF FSRTemp
			;	if (TMR2IF == 1) {
	BTFSS 0x0C,TMR2IF
	GOTO  m004
			;		DigitCounter++;				// Every 2ms Update the display digits
	INCF  DigitCounter,1
			;		if (DigitCounter == 4) {	// 0 = Units, 1 = tens, 2 = hundreds, 3 = thousands
	MOVF  DigitCounter,W
	XORLW .4
	BTFSC 0x03,Zero_
			;			DigitCounter = 0;
	CLRF  DigitCounter
			;		}
			;		Dec2BCD();
	CALL  Dec2BCD
			;		DisplayDigit(DigitCounter);
	MOVF  DigitCounter,W
	CALL  DisplayDigit
			;		
			;		BlinkCounter++;
	INCF  BlinkCounter,1
			;		{
			;			if (BlinkCounter > MaxBlink) {
	MOVLW .101
	SUBWF BlinkCounter,W
	BTFSS 0x03,Carry
	GOTO  m002
			;				BlinkCounter = 0;
	CLRF  BlinkCounter
			;				if(BlinkState==0){
	MOVF  BlinkState,1
	BTFSS 0x03,Zero_
	GOTO  m001
			;					BlinkState=1;
	MOVLW .1
	MOVWF BlinkState
			;				} else {
	GOTO  m002
			;					BlinkState =0;
m001	CLRF  BlinkState
			;				}	
			;			}
			;		}		
			;
			;		if (State1 == STATE_STARTED) {
m002	DECFSZ State1,W
	GOTO  m003
			;			T2ms++;
	INCF  T2ms,1
			;			if (T2ms == 5) {
	MOVF  T2ms,W
	XORLW .5
	BTFSS 0x03,Zero_
	GOTO  m003
			;				T10ms++;
	INCF  T10ms,1
			;				T2ms = 0;
	CLRF  T2ms
			;				if (T10ms == 10) {
	MOVF  T10ms,W
	XORLW .10
	BTFSS 0x03,Zero_
	GOTO  m003
			;					T100ms++;
	INCF  T100ms,1
			;					T10ms = 0;
	CLRF  T10ms
			;					if (T100ms == 10) {
	MOVF  T100ms,W
	XORLW .10
	BTFSS 0x03,Zero_
	GOTO  m003
			;						T100ms = 0;
	CLRF  T100ms
			;						SecondsFlag = 1;
	MOVLW .1
	MOVWF SecondsFlag
			;					}
			;				}
			;			}
			;		}
			;		TMR2IF = 0;
m003	BCF   0x0C,TMR2IF
			;	}
			;   	FSR = FSRTemp;
m004	MOVF  FSRTemp,W
	MOVWF FSR
			;   	int_restore_registers
	SWAPF svrSTATUS,W
	MOVWF STATUS
	SWAPF svrWREG,1
	SWAPF svrWREG,W
			;}
	RETFIE
			;
			;void main(void) {
main
			;	int Seconds =00;
	BCF   0x03,RP0
	BCF   0x03,RP1
	CLRF  Seconds
			;	int Minutes =00;
	CLRF  Minutes
			;	char Timing=0;
	CLRF  Timing
			;	char START =0;
	CLRF  START
			;	char STOP =0;
	CLRF  STOP
			;	char StartLock =0;
	CLRF  StartLock
			;	char StopLock =0;
	CLRF  StopLock
			;	char temp =0;
	CLRF  temp
			;
			;	InitialiseHardware();
	BSF   0x03,RP0
	CALL  InitialiseHardware
			;	InitialiseGlobals();
	CALL  InitialiseGlobals
			;
			;	while (1) {
			;		debounce(StartButton, &StartButtonState, &StartButtonCounter);
m005	CLRF  test
	BTFSC 0x05,RA0
	INCF  test,1
	MOVLW .64
	MOVWF state
	MOVLW .66
	MOVWF counter
	CALL  debounce
			;		debounce(StopButton, &StopButtonState, &StopButtonCounter);
	CLRF  test
	BTFSC 0x05,RA1
	INCF  test,1
	MOVLW .67
	MOVWF state
	MOVLW .69
	MOVWF counter
	CALL  debounce
			;		
			;		if(StartButtonState == 0) {
	MOVF  StartButtonState,1
	BTFSC 0x03,Zero_
			;			StartLock = 0;
	CLRF  StartLock
			;		}
			;		if(StopButtonState == 0) {
	MOVF  StopButtonState,1
	BTFSC 0x03,Zero_
			;			StopLock = 0;
	CLRF  StopLock
			;		}
			;
			;		if ((StartButtonState == 1) && (StartLock == 0)) {
	DECFSZ StartButtonState,W
	GOTO  m008
	MOVF  StartLock,1
	BTFSS 0x03,Zero_
	GOTO  m008
			;			StartLock = 1;									// Lock out until button is released
	MOVLW .1
	MOVWF StartLock
			;			if(State1 == STATE_STOPPED  || State1 == STATE_PAUSED) {		
	MOVF  State1,1
	BTFSC 0x03,Zero_
	GOTO  m006
	MOVF  State1,W
	XORLW .2
	BTFSS 0x03,Zero_
	GOTO  m007
			;				State1 = STATE_STARTED;						// Start timing if currently stopped
m006	MOVLW .1
	MOVWF State1
			;			} else {
	GOTO  m008
			;//				State1 = STATE_STOPPED;						// stop timing - doesn't reset anything (Paused)
			;				State1 = STATE_PAUSED;
m007	MOVLW .2
	MOVWF State1
			;			}	
			;		}
			;/*		
			;		if ((StartButtonState == 1) && (StartLock == 1)) {
			;			StartLock = 1;									// Lock out until button is released
			;			if(State1 == STATE_PAUSED) {		
			;				State1 = STATE_STARTED;						// Start timing if currently stopped
			;			}	
			;		}	
			;*/		
			;		    
			;	    if((StopButtonState == 1) && (StopLock == 0)) {
m008	DECFSZ StopButtonState,W
	GOTO  m010
	MOVF  StopLock,1
	BTFSS 0x03,Zero_
	GOTO  m010
			;		    StopLock = 1;									// Lock out until button is released
	MOVLW .1
	MOVWF StopLock
			;			if(State1 == STATE_STARTED) {	
	DECFSZ State1,W
	GOTO  m009
			;//				State1 = STATE_STOPPED;						// Stop timing if currently running but don't reset (Paused)
			;				State1 = STATE_PAUSED;						// When paused, blink display
	MOVLW .2
	MOVWF State1
			;			} else {
	GOTO  m010
			;				DispValue = 0;								// If in routine for 2nd instance clear timers (Stop pressed after being paused)
m009	CLRF  DispValue
	CLRF  DispValue+1
			;				Minutes = 0;
	CLRF  Minutes
			;				Seconds = 0;
	CLRF  Seconds
			;				SecondsFlag = 0;
	CLRF  SecondsFlag
			;				T2ms = 0;
	CLRF  T2ms
			;				T10ms = 0;
	CLRF  T10ms
			;				T100ms = 0;
	CLRF  T100ms
			;				State1 = STATE_STOPPED;
	CLRF  State1
			;			} 
			;		}
			;		
			;		if (State1 == STATE_STARTED) {
m010	DECFSZ State1,W
	GOTO  m005
			;			if (SecondsFlag) {
	MOVF  SecondsFlag,1
	BTFSC 0x03,Zero_
	GOTO  m005
			;				SecondsFlag = 0;
	CLRF  SecondsFlag
			;				Seconds++;
	INCF  Seconds,1
			;				if (Seconds == 60) {
	MOVF  Seconds,W
	XORLW .60
	BTFSS 0x03,Zero_
	GOTO  m011
			;					Minutes++;
	INCF  Minutes,1
			;					Seconds = 0;
	CLRF  Seconds
			;					if(Minutes == 100) {
	MOVF  Minutes,W
	XORLW .100
	BTFSC 0x03,Zero_
			;						Minutes = 0;
	CLRF  Minutes
			;					}
			;				}	
			;				DispValue=Minutes;
m011	MOVF  Minutes,W
	MOVWF DispValue
	CLRF  DispValue+1
	BTFSC DispValue,7
	DECF  DispValue+1,1
			;				DispValue=DispValue<<8;
	MOVWF DispValue+1
	CLRF  DispValue
			;				DispValue=DispValue+Seconds;
	MOVF  Seconds,W
	ADDWF DispValue,1
	BTFSC 0x03,Carry
	INCF  DispValue+1,1
	BTFSC Seconds,7
	DECF  DispValue+1,1
			;			}
			;		}
			;	}
	GOTO  m005
			;}// end main()
			;
			;
			;void InitialiseHardware(void) {
InitialiseHardware
			;  	TRISA  = 0b00100011; 				// PORTA (0 = OUTPUT)
	MOVLW .35
	MOVWF TRISA
			;	PORTA  = 0b11011100;				// Initialise PORTA
	MOVLW .220
	BCF   0x03,RP0
	MOVWF PORTA
			;	TRISB  = 0b11000000;      			// PORTB (0 = OUTPUT)
	MOVLW .192
	BSF   0x03,RP0
	MOVWF TRISB
			;	PORTB  = 0b00000000;				// Initialise PORTB
	BCF   0x03,RP0
	CLRF  PORTB
			;	OPTION = 0b10001000;   				// No weak pull ups, prescaler assigned to WDT
	MOVLW .136
	BSF   0x03,RP0
	MOVWF OPTION_REG
			;   	INTCON = 0b11000000;				// TMR2 used to provide 1ms ticks.
	MOVLW .192
	MOVWF INTCON
			;	CMCON  = 0b00000111;				// Enable RA0:3 as Digital Inputs
	MOVLW .7
	BCF   0x03,RP0
	MOVWF CMCON
			;	PCON   = 0b00001000;				// Set internal osciallator to 4MHz
	MOVLW .8
	BSF   0x03,RP0
	MOVWF PCON
			;	T2CON  = 0b00001101;				// TMR2 on, prescale = 1:4, and postscale = 1:2  (8us ticks with 4MHz oscillator)
	MOVLW .13
	BCF   0x03,RP0
	MOVWF T2CON
			;	PIE1   = 0b00000010;				// Bit 1 enables TMR2 = PR2 interface
	MOVLW .2
	BSF   0x03,RP0
	MOVWF PIE1
			;	PIR1   = 0b00000000;				// Read this to see if TMR2 = PR2 flag is set
	BCF   0x03,RP0
	CLRF  PIR1
			;	PR2 = 250;							// TMR2 match value, for 2ms ticks
	MOVLW .250
	BSF   0x03,RP0
	MOVWF PR2
			;	MC14543_LD_CTRL=0;						// Disable loads to BCD driver
	BCF   0x03,RP0
	BCF   0x06,RB5
			;
			;	DispValue = 0xAAAA;
	MOVLW .170
	MOVWF DispValue
	MOVWF DispValue+1
			;}
	RETURN
			;
			;void InitialiseGlobals(void) {
InitialiseGlobals
			;	T2ms = 0;							// Reset Timers
	CLRF  T2ms
			;	T10ms = 0;
	CLRF  T10ms
			;	T100ms = 0;
	CLRF  T100ms
			;	SecondsFlag=0;
	CLRF  SecondsFlag
			;	DigitCounter =0;
	CLRF  DigitCounter
			;	DispValue = 0;
	CLRF  DispValue
	CLRF  DispValue+1
			;
			;	StartButtonState = StartButton;
	CLRF  StartButtonState
	BTFSC 0x05,RA0
	INCF  StartButtonState,1
			;	StartButtonPrev = StartButtonState;
	MOVF  StartButtonState,W
	MOVWF StartButtonPrev
			;	StartButtonCounter = 0;
	CLRF  StartButtonCounter
			;	StopButtonState =0;
	CLRF  StopButtonState
			;	StopButtonPrev =0;
	CLRF  StopButtonPrev
			;	StopButtonCounter =0;
	CLRF  StopButtonCounter
			;	State1 = STATE_STOPPED;
	CLRF  State1
			;}
	RETURN
			;
			;void DisplayDigit(char DigitCounter){
DisplayDigit
	MOVWF DigitCounter_2
			;
			;	DISP0=0;
	BCF   0x05,RA6
			;	DISP1=0;
	BCF   0x06,RB4
			;	DISP2=0;
	BCF   0x05,RA3
			;	DISP3=0;
	BCF   0x05,RA2
			;
			;	DECIMAL=0;
	BCF   0x05,RA7
			;
			;//	Put current digit on PORTB lower nibble
			;	PORTB=PORTB&0b11110000;					// CLEAR PORTB LOWER NIBBLE
	MOVLW .240
	ANDWF PORTB,1
			;//	PORTB=PORTB|DigitCounter;				// Write current digit to lower nibble of PORTB
			;
			;// Turn on only the digit of interest
			;
			;
			;		switch (DigitCounter) {
	MOVF  DigitCounter_2,W
	BTFSC 0x03,Zero_
	GOTO  m012
	XORLW .1
	BTFSC 0x03,Zero_
	GOTO  m013
	XORLW .3
	BTFSC 0x03,Zero_
	GOTO  m014
	XORLW .1
	BTFSC 0x03,Zero_
	GOTO  m015
	GOTO  m016
			;  			case 0:
			;				PORTB=PORTB|Digit[0];
m012	MOVF  Digit,W
	IORWF PORTB,1
			;				DISP0=1;								
	BSF   0x05,RA6
			;	   			break;
	GOTO  m016
			;  			case 1:
			;				PORTB=PORTB|Digit[1];
m013	MOVF  Digit+1,W
	IORWF PORTB,1
			;				DISP1=1;										
	BSF   0x06,RB4
			;	   			break;
	GOTO  m016
			;  			case 2:
			;				PORTB=PORTB|Digit[2];
m014	MOVF  Digit+2,W
	IORWF PORTB,1
			;				DISP2=1;	
	BSF   0x05,RA3
			;				DECIMAL=1;									
	BSF   0x05,RA7
			;	   			break;
	GOTO  m016
			;  			case 3:
			;				PORTB=PORTB|Digit[3];
m015	MOVF  Digit+3,W
	IORWF PORTB,1
			;				DISP3=1;										
	BSF   0x05,RA2
			;	   			break;
			;			default:
			;				break;
			;		}
			;	MC14543_LD_CTRL=1;						// Load digit into BCD driver.
m016	BSF   0x06,RB5
			;    MC14543_LD_CTRL=0;						// Disable loads to BCD driver
	BCF   0x06,RB5
			;}	
	RETURN
			;
			;void Dec2BCD() {
Dec2BCD
			;	uns16 temp =0;
	CLRF  temp_2
	CLRF  temp_2+1
			;	//THIS CONVERTS 16BIT NO to 4 BCD bytes
			;	/*
			;		temp=DispValue%10;
			;		Digit[0]=temp;			
			;		temp=DispValue/10;		
			;		Digit[1]=temp%10;			
			;		temp=DispValue/100;	
			;		Digit[2]=temp%10;			
			;		temp=DispValue/1000;	
			;		Digit[3]=temp;
			;	*/
			;
			;	if (BlinkState == 1 && State1== STATE_PAUSED) {					// if paused and blinking = 1 (blank display)
	DECFSZ BlinkState,W
	GOTO  m017
	MOVF  State1,W
	XORLW .2
	BTFSS 0x03,Zero_
	GOTO  m017
			;		Digit[0] = 0x0A;											// Non numerical characters are shown as blank by BDC driver chip
	MOVLW .10
	MOVWF Digit
			;		Digit[1] = 0x0A;								
	MOVWF Digit+1
			;		Digit[2] = 0x0A;
	MOVWF Digit+2
			;		Digit[3] = 0x0A;
	MOVWF Digit+3
			;	} else {														// push number to display 
	GOTO  m042
			;		temp=DispValue&0x00FF;										// need to set upper digits to blank if all leading zeroes
m017	CLRF  temp_2+1
	MOVF  DispValue,W
	MOVWF temp_2
			;		Digit[0]=temp%10;											// DIGIT0 = LS Digit 
	MOVWF C2tmp
	MOVF  temp_2+1,W
	MOVWF C2tmp+1
	CLRF  Digit
	MOVLW .16
	MOVWF C1cnt
m018	RLF   C2tmp,1
	RLF   C2tmp+1,1
	RLF   Digit,1
	BTFSC 0x03,Carry
	GOTO  m019
	MOVLW .10
	SUBWF Digit,W
	BTFSS 0x03,Carry
	GOTO  m020
m019	MOVLW .10
	SUBWF Digit,1
m020	DECFSZ C1cnt,1
	GOTO  m018
			;		temp=temp/10;
	MOVF  temp_2,W
	MOVWF C4tmp
	MOVF  temp_2+1,W
	MOVWF C4tmp+1
	CLRF  C5rem
	MOVLW .16
	MOVWF C3cnt
m021	RLF   C4tmp,1
	RLF   C4tmp+1,1
	RLF   C5rem,1
	BTFSC 0x03,Carry
	GOTO  m022
	MOVLW .10
	SUBWF C5rem,W
	BTFSS 0x03,Carry
	GOTO  m023
m022	MOVLW .10
	SUBWF C5rem,1
	BSF   0x03,Carry
m023	RLF   temp_2,1
	RLF   temp_2+1,1
	DECFSZ C3cnt,1
	GOTO  m021
			;		Digit[1]=temp%10;
	MOVF  temp_2,W
	MOVWF C7tmp
	MOVF  temp_2+1,W
	MOVWF C7tmp+1
	CLRF  Digit+1
	MOVLW .16
	MOVWF C6cnt
m024	RLF   C7tmp,1
	RLF   C7tmp+1,1
	RLF   Digit+1,1
	BTFSC 0x03,Carry
	GOTO  m025
	MOVLW .10
	SUBWF Digit+1,W
	BTFSS 0x03,Carry
	GOTO  m026
m025	MOVLW .10
	SUBWF Digit+1,1
m026	DECFSZ C6cnt,1
	GOTO  m024
			;		temp=DispValue;
	MOVF  DispValue,W
	MOVWF temp_2
	MOVF  DispValue+1,W
	MOVWF temp_2+1
			;		temp=temp>>8;												// temp is only 8 bit so now get upper 2 digits
	MOVWF temp_2
	CLRF  temp_2+1
			;		Digit[2]=temp%10;
	MOVWF C9tmp
	MOVF  temp_2+1,W
	MOVWF C9tmp+1
	CLRF  Digit+2
	MOVLW .16
	MOVWF C8cnt
m027	RLF   C9tmp,1
	RLF   C9tmp+1,1
	RLF   Digit+2,1
	BTFSC 0x03,Carry
	GOTO  m028
	MOVLW .10
	SUBWF Digit+2,W
	BTFSS 0x03,Carry
	GOTO  m029
m028	MOVLW .10
	SUBWF Digit+2,1
m029	DECFSZ C8cnt,1
	GOTO  m027
			;		temp=temp/10;		
	MOVF  temp_2,W
	MOVWF C11tmp
	MOVF  temp_2+1,W
	MOVWF C11tmp+1
	CLRF  C12rem
	MOVLW .16
	MOVWF C10cnt
m030	RLF   C11tmp,1
	RLF   C11tmp+1,1
	RLF   C12rem,1
	BTFSC 0x03,Carry
	GOTO  m031
	MOVLW .10
	SUBWF C12rem,W
	BTFSS 0x03,Carry
	GOTO  m032
m031	MOVLW .10
	SUBWF C12rem,1
	BSF   0x03,Carry
m032	RLF   temp_2,1
	RLF   temp_2+1,1
	DECFSZ C10cnt,1
	GOTO  m030
			;		Digit[3]=temp%10;
	MOVF  temp_2,W
	MOVWF C14tmp
	MOVF  temp_2+1,W
	MOVWF C14tmp+1
	CLRF  Digit+3
	MOVLW .16
	MOVWF C13cnt
m033	RLF   C14tmp,1
	RLF   C14tmp+1,1
	RLF   Digit+3,1
	BTFSC 0x03,Carry
	GOTO  m034
	MOVLW .10
	SUBWF Digit+3,W
	BTFSS 0x03,Carry
	GOTO  m035
m034	MOVLW .10
	SUBWF Digit+3,1
m035	DECFSZ C13cnt,1
	GOTO  m033
			;		//temp=DispValue;//like the goggles, it does nothing!!
			;		
			;		// now use logic to set digits blank - no leading zeros
			;		
			;		
			;			
			;		if(State1==STATE_STARTED  ||  State1 == STATE_PAUSED){
	DECF  State1,W
	BTFSC 0x03,Zero_
	GOTO  m036
	MOVF  State1,W
	XORLW .2
	BTFSS 0x03,Zero_
	GOTO  m042
			;			if (Digit[3] == 0){
m036	MOVF  Digit+3,1
	BTFSS 0x03,Zero_
	GOTO  m037
			;			Digit[3] = 0x0A;
	MOVLW .10
	MOVWF Digit+3
			;			}
			;			if (Digit[2] == 0 && (Digit[3] == 0 || Digit[3] == 0x0A)){
m037	MOVF  Digit+2,1
	BTFSS 0x03,Zero_
	GOTO  m039
	MOVF  Digit+3,1
	BTFSC 0x03,Zero_
	GOTO  m038
	MOVF  Digit+3,W
	XORLW .10
	BTFSS 0x03,Zero_
	GOTO  m039
			;			Digit[2] = 0x0A;
m038	MOVLW .10
	MOVWF Digit+2
			;			}
			;			if (Digit[1] == 0 && (Digit[2] == 0 || Digit[2] == 0x0A)&& (Digit[2] == 0 || Digit[2] == 0x0A) ){
m039	MOVF  Digit+1,1
	BTFSS 0x03,Zero_
	GOTO  m042
	MOVF  Digit+2,1
	BTFSC 0x03,Zero_
	GOTO  m040
	MOVF  Digit+2,W
	XORLW .10
	BTFSS 0x03,Zero_
	GOTO  m042
m040	MOVF  Digit+2,1
	BTFSC 0x03,Zero_
	GOTO  m041
	MOVF  Digit+2,W
	XORLW .10
	BTFSS 0x03,Zero_
	GOTO  m042
			;			Digit[1] = 0x0A;
m041	MOVLW .10
	MOVWF Digit+1
			;			}
			;
			;		}		
			;		
			;/*		if (Digit[3] == 0 && Digit[2] == 0 && Digit[1] == 0 && Digit[0] == 0){
			;			Digit[0] = 0x0A;
			;		}*/
			;		// NO - ALWAYS HAVE Digit[0] set to valid number - will blink '0' if pasued at zero
			;		// and will not show 0 when timer starts		
			;	}
			;}
m042	RETURN
			;
			;void debounce(char test, char *state, char *counter) {
debounce
			;	if ((*state) != test) {
	BCF   0x03,IRP
	MOVF  state,W
	MOVWF FSR
	MOVF  INDF,W
	XORWF test,W
	BTFSC 0x03,Zero_
	GOTO  m043
			;   	(*counter)++;
	BCF   0x03,IRP
	MOVF  counter,W
	MOVWF FSR
	INCF  INDF,1
			;		if ((*counter) >= MAX_DEBOUNCE_COUNT) {
	BCF   0x03,IRP
	MOVF  counter,W
	MOVWF FSR
	MOVLW .10
	SUBWF INDF,W
	BTFSS 0x03,Carry
	GOTO  m044
			;			(*counter) = 0;
	BCF   0x03,IRP
	MOVF  counter,W
	MOVWF FSR
	CLRF  INDF
			;			(*state) = test;
	BCF   0x03,IRP
	MOVF  state,W
	MOVWF FSR
	MOVF  test,W
	MOVWF INDF
			;		}
			;	} else {
	GOTO  m044
			;		(*counter) = 0;
m043	BCF   0x03,IRP
	MOVF  counter,W
	MOVWF FSR
	CLRF  INDF
			;	}
			;}
m044	RETURN

	ORG 0x2007
	DATA 3F58H
	END


; *** KEY INFO ***

; 0x00AA   38 word(s)  3 % : InitialiseHardware
; 0x00D0   18 word(s)  1 % : InitialiseGlobals
; 0x00E2   40 word(s)  3 % : DisplayDigit
; 0x010A  191 word(s) 18 % : Dec2BCD
; 0x01C9   33 word(s)  3 % : debounce
; 0x0004   60 word(s)  5 % : IntHandler
; 0x0040  106 word(s) 10 % : main

; RAM usage: 40 bytes (19 local), 184 bytes free
; Maximum call level: 1 (+2 for interrupt)
; Total of 487 code words (47 %)
