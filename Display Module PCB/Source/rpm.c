//#define   PIC16F627A

//#include "16F627A.H"
#include "INT16CXX.H"

#pragma origin = 0x0004																	// CHECK THIS IN DATASHEET.  WHAT 0x0004 ???
#pragma config        = 0b0011.1111.0101.1000   	// MCLR Pin is input
//#pragma config        = 0b0011.1111.0111.1000   	// MCLR Pin is input

#define BCD_A			RB0		// OUTPUT, BIT0 BCD WORD
#define BCD_B      		RB1   	// OUTPUT, BIT1 BCD WORD  
#define BCD_C			RB2   	// OUTPUT, BIT2 BCD WORD		
#define BCD_D	 	 	RB3   	// OUTPUT, BIT3 BCD WORD		
#define MC14543_LD_CTRL	RB5		// Rising Edge Load to BCD DRiver


#define DISP0			RA6
#define DISP1			RB4
#define DISP2			RA3	
#define DISP3			RA2

#define DECIMAL			RA7

#define StartButton		RA0
#define StopButton		RA1


//#define Status0         RB6    // NOT USED
//#define Status1         RB7    // NOT USED

#define MAX_DEBOUNCE_COUNT 	10			// How many times do we need the changed state = 100 ms

// Declare Function Prototypes
void InitialiseHardware(void);
void InitialiseGlobals(void);
char CheckSeconds(void);

void DisplayDigit(char DigitCounter);
void Dec2BCD(void);

void debounce(char test, char *state, char *counter);

char FSRTemp;
char T2ms;								// 2 millisecond counter
char T10ms;								// 10's millisecond counter
char T100ms;							// 100's millisecond counter

char DigitCounter;
uns16 DispValue;		// unsigned 16 bit integer
char Digit[4];
char SecondsFlag;

char StartButtonState;
char StartButtonPrev;
char StartButtonCounter;
char StartButtonTest;

char StopButtonState;
char StopButtonPrev;
char StopButtonCounter;
char StopButtonTest;

#pragma codepage 0
interrupt IntHandler() {
   
   int_save_registers
   FSRTemp = FSR;


	if (TMR2IF == 1) {
		T2ms++;
			DigitCounter++;			// Every 10ms Update the display digits
			if(DigitCounter==4){	// 0 = Units, 1 = tens, 2 = hundreds, 3 = thousands
				DigitCounter=0;
			}
			DisplayDigit(DigitCounter);
			if(T2ms == 5) {
				T10ms++;
				T2ms = 0;
				debounce(StartButton, &StartButtonState, & StartButtonCounter);
				debounce(StopButton, &StopButtonState, & StopButtonCounter);
				if (T10ms == 10) {
					T100ms++;
					T10ms = 0;
					Dec2BCD();
					if(T100ms == 10) {
						T100ms =0;	
						SecondsFlag=1;		
					}
				}
			}
		TMR2IF = 0;
		}

   FSR = FSRTemp;
   int_restore_registers
}



void main(void) {

int Seconds =00;
int Minutes =00;
char Timing=0;
char START =0;
char STOP =0;


char StartInit =0;
char StartInitState=0;
char StartInitCounter=0;
char StopInit =0;

char temp =0;

InitialiseHardware();
InitialiseGlobals();

	while(StartInit==0 && StopInit ==0){

		DispValue=0x8888;

		if(StartButton==StartButtonState){
			StartInit=1;
			START=0;
			StartButtonPrev=StartButtonState;
		}
		if(StopButton==StopButtonState){
			StopInit=1;
			STOP=0;
			StopButtonPrev=StopButtonState;
		}
	}

	while (1){
	// Check state of start / stop buttons.

//	START=StartButtonState;

	if(StartButtonState!=StartButtonPrev) {
		StartButtonPrev=StartButtonState;
		if(START==0)
			START=1;
		else
			START=0;
	} 

	if(StopButtonState!=StopButtonPrev) {
		StopButtonPrev=StopButtonState;
		if(STOP==0)
			STOP=1;
		else
			STOP=0;
	} 

	

		if(START){
			if(Timing==0){
				T2ms = 0;							// Reset Timers
				T10ms = 0;
				T100ms = 0;
				SecondsFlag=0;
				Timing=1;
			}	
/*			else {
					Timing=0;
			}*/	
			if(SecondsFlag){
				Seconds++;
				SecondsFlag=0;
				if(Seconds==60){
					Minutes++;
					Seconds=0;
					if(Minutes==100){
						Minutes=0;
					}
				}	
				DispValue=Minutes;
				DispValue=DispValue<<8;
				DispValue=DispValue+Seconds;
			}
		}
		else {	
			Timing=0;
		}
		if(STOP){
			DispValue=0;
			Minutes=0;
			Seconds=0;
			Timing=0;
		}	
    }

}// end main()


void InitialiseHardware(void) {
  
	TRISA  = 0b00100011; 				// PORTA (0 = OUTPUT)
	PORTA  = 0b11011100;				// Initialise PORTA
	TRISB  = 0b11000000;      			// PORTB (0 = OUTPUT)
	PORTB  = 0b00000000;				// Initialise PORTB
	OPTION = 0b10001000;   				// No weak pull ups, prescaler assigned to WDT
   	INTCON = 0b11000000;				// TMR2 used to provide 1ms ticks.
	CMCON  = 0b00000111;				// Enable RA0:3 as Digital Inputs
	PCON   = 0b00001000;				// Set internal osciallator to 4MHz
	T2CON  = 0b00001101;				// TMR2 on, prescale = 1:4, and postscale = 1:2  (8us ticks with 4MHz oscillator)
	PIE1   = 0b00000010;				// Bit 1 enables TMR2 = PR2 interface
	PIR1   = 0b00000000;				// Read this to see if TMR2 = PR2 flag is set
	PR2 = 250;							// TMR2 match value, for 2ms ticks
MC14543_LD_CTRL=0;						// Disable loads to BCD driver

}

void InitialiseGlobals(void) {

	T2ms = 0;							// Reset Timers
	T10ms = 0;
	T100ms = 0;
	SecondsFlag=0;
	DigitCounter =0;

	StartButtonState =0;
	StartButtonPrev =0;
	StartButtonCounter =0;
	StartButtonTest =0;

	StopButtonState =0;
	StopButtonPrev =0;
	StopButtonCounter =0;
	StopButtonTest =0;


}

void DisplayDigit(char DigitCounter){

	DISP0=0;
	DISP1=0;
	DISP2=0;
	DISP3=0;

	DECIMAL=0;

//	Put current digit on PORTB lower nibble
	PORTB=PORTB&0b11110000;					// CLEAR PORTB LOWER NIBBLE
//	PORTB=PORTB|DigitCounter;				// Write current digit to lower nibble of PORTB

// Turn on only the digit of interest


		switch (DigitCounter) {
  			case 0:
				PORTB=PORTB|Digit[0];
				DISP0=1;								
	   			break;
  			case 1:
				PORTB=PORTB|Digit[1];
				DISP1=1;										
	   			break;
  			case 2:
				PORTB=PORTB|Digit[2];
				DISP2=1;	
				DECIMAL=1;									
	   			break;
  			case 3:
				PORTB=PORTB|Digit[3];
				DISP3=1;										
	   			break;
			default:
				break;
		}
	MC14543_LD_CTRL=1;						// Load digit into BCD driver.
    MC14543_LD_CTRL=0;						// Disable loads to BCD driver
}	

void Dec2BCD(){

uns16 temp =0;


//THIS CONVERTS 16BIT NO to 4 BCD bytes
/*
	temp=DispValue%10;
	Digit[0]=temp;			
	temp=DispValue/10;		
	Digit[1]=temp%10;			
	temp=DispValue/100;	
	Digit[2]=temp%10;			
	temp=DispValue/1000;	
	Digit[3]=temp;
*/

	temp=DispValue&0x00FF;
	Digit[0]=temp%10;
	temp=temp/10;
	Digit[1]=temp%10;
	temp=DispValue;
	temp=temp>>8;
	Digit[2]=temp%10;
	temp=temp/10;
	Digit[3]=temp%10;
	temp=DispValue;

}

void debounce(char test, char *state, char *counter) {
	if ((*state) != test) {
   	(*counter)++;
		if ((*counter) >= MAX_DEBOUNCE_COUNT) {
			(*counter) = 0;
			(*state) = test;
		}
	} else {
		(*counter) = 0;
	}
}


