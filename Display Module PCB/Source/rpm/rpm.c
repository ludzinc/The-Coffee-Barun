//#define   PIC16F88

//#include "16F88.H"
#include "INT16CXX.H"


#pragma origin = 0x0004																	// CHECK THIS IN DATASHEET.  WHAT 0x0004 ???
#pragma config        	= 0b0011.1111.0101.1000   	// MCLR Pin is input
#pragma config reg2     = 0b0011.1111.1111.1100   

#define BCD_A			PORTB.0	//RB0		// OUTPUT, BIT0 BCD WORD
#define BCD_B      		PORTB.1 //RB1   	// OUTPUT, BIT1 BCD WORD  
#define BCD_C			PORTB.2 //RB2   	// OUTPUT, BIT2 BCD WORD		
#define BCD_D	 	 	PORTB.3 //RB3   	// OUTPUT, BIT3 BCD WORD
#define MC14543_BI_CTRL PORTB.4 //RB4		// Set low to disable blanking
#define MC14543_LD_CTRL	PORTB.5 //RB5		// Rising Edge Load to BCD DRiver

#define DISP0			PORTA.6	//RA2
#define DISP1			PORTA.4 //RA3
#define DISP2			PORTA.3 //RA4	
#define DISP3			PORTA.2 //RA6
#define DECIMAL			PORTA.7 //RA7

#define ScaleRange		1850
#define ScaleOffset		500
#define FilterDepth		20

// Declare Function Prototypes
void InitialiseHardware(void);
void InitialiseGlobals(void);
void DisplayDigit(char DigitCounter);
void Dec2BCD(void);
void GetA2D(void);

char FSRTemp;
char DigitCounter;
uns16 DispValue;		// unsigned 16 bit integer
uns16 A2DResult;
uns16 A2DFilter[FilterDepth];
char Digit[4];
char FilterIndex;

float MagicNumber;
float OldMagicNumber;

#pragma codepage 0
interrupt IntHandler() {
   	int_save_registers
   	FSRTemp = FSR;
	if (TMR2IF == 1) {
		DigitCounter++;				// Every 10ms Update the display digits
		if (DigitCounter == 4) {	// 0 = Units, 1 = tens, 2 = hundreds, 3 = thousands
			DigitCounter = 0;
			}
		Dec2BCD();
		DisplayDigit(DigitCounter);
		TMR2IF = 0;
	}
   	FSR = FSRTemp;
   	int_restore_registers
}

#include "Math24f.h"

void main(void) {

	InitialiseHardware();
	InitialiseGlobals();

	while (1) {
		GetA2D();
	}
}// end main()


void InitialiseHardware(void) {
  	TRISA  = 0b00100011; 				// PORTA (0 = OUTPUT)
	PORTA  = 0b11011100;				// Initialise PORTA
	TRISB  = 0b11000000;      			// PORTB (0 = OUTPUT)
	PORTB  = 0b00000000;				// Initialise PORTB
	OPTION = 0b10001000;   				// No weak pull ups, prescaler assigned to WDT											(checked ok)
   	INTCON = 0b11000000;				// TMR2 used to provide 1ms ticks.														(checked ok)
	CMCON  = 0b00000111;				// Enable RA0:3 as Digital Inputs														(checked ok)
	PCON   = 0b00000000;				// 													(checked ok)

	OSCCON = 0b01101100;				// 4MHz internal oscillator

	T2CON  = 0b00010101;				// TMR2 on, prescale = 1:4, and postscale = 1:2  (8us ticks with 4MHz oscillator) 		(check this for 8MHz)
	PIE1   = 0b00000010;				// Bit 1 enables TMR2 = PR2 interface
	PIR1   = 0b00000000;				// Read this to see if TMR2 = PR2 flag is set
	PR2 = 250;							// TMR2 match value, for 2ms ticks
	MC14543_LD_CTRL = 0;				// Disable loads to BCD driver
	MC14543_BI_CTRL = 0;				// 1 Blanks display

	ANSEL 	= 0x01;						// Select AN0
	ADCON0	= 0x81;						// Tosc / 32, A/D on	
	ADCON1 	= 0x80;						// Set MSB of ADRESH to 0, VrefH = Vdd, VrefL = Vss;
}

void InitialiseGlobals(void) {
	char i =0;
	DigitCounter =0;
	DispValue = 0;
	MagicNumber =0;
	FilterIndex=0;
	for (i=0;i<4;i++) {
		Digit[i]=0;
	}
	for(i=0;i<FilterDepth-1;i++){
		A2DFilter[i]=0;	
	}
}

void DisplayDigit(char DigitCounter){

	DISP0=0;
	DISP1=0;
	DISP2=0;
	DISP3=0;

	DECIMAL=0;

//	Put current digit on PORTB lower nibble
	PORTB=PORTB&0b11110000;					// CLEAR PORTB LOWER NIBBLE
// Turn on only the digit of interest
		switch (DigitCounter) {
  			case 0:
				PORTB=PORTB|Digit[0];
				//DECIMAL=1;								
				DISP0=1;
	   			break;
  			case 1:
				PORTB=PORTB|Digit[1];
				//DECIMAL=1;
				DISP1=1;										
	   			break;
  			case 2:
				PORTB=PORTB|Digit[2];	
				//DECIMAL=1;									
	   			DISP2=1;
				break;
  			case 3:
				PORTB=PORTB|Digit[3];
				//DECIMAL=1;
				DISP3=1;										
	   			break;
			default:
				break;
		}
	MC14543_LD_CTRL=1;						// Load digit into BCD driver.
    MC14543_LD_CTRL=0;						// Disable loads to BCD driver
}	

void Dec2BCD() {
	uns16 temp =0;
	//THIS CONVERTS 16BIT NO to 4 BCD bytes
		temp=DispValue%10;
		Digit[0]=temp;			
		temp=DispValue/10;		
		Digit[1]=temp%10;			
		temp=DispValue/100;	
		Digit[2]=temp%10;			
		temp=DispValue/1000;	
		Digit[3]=temp;
}


void GetA2D() {
	uns16 Filter =0;
	char i =0;
	uns16 filtertemp =0;
	uns16 comparison =0;

	A2DResult =0;
	
	ADCON0=ADCON0|0x04;							//Start A2D Conversion
	while((ADCON0&0x04) != 0){};						// Wait for conversion
	A2DResult=ADRESH;
	A2DResult=A2DResult<<8;
	A2DResult=A2DResult|ADRESL;
	A2DResult=A2DResult&0b0000.1111.1111.1111;
	
	//if(A2DResult<3){
	//	A2DResult=0;
	//}
	
	// NOTE ON FILTERING - A2D range is 0 - 1023 (12 Bits)
	// FILTER variable is 16 bit
	// max no for Fitler depth = 64
	
	A2DFilter[FilterIndex]=A2DResult;
	//A2DFilter[FilterIndex]=FilterDepth;
	FilterIndex++;
	if(FilterIndex>FilterDepth-1){
		FilterIndex=0;
	}

	for(i=0;i<FilterDepth-1;i++) {
		filtertemp = A2DFilter[i];
		//filtertemp=25;
		Filter= Filter+filtertemp;
	}	
	Filter=Filter+filtertemp;
	Filter=Filter/FilterDepth;
	
//DispValue = Filter;

MagicNumber = Filter;
MagicNumber = MagicNumber * ScaleRange;
MagicNumber = MagicNumber / 1024;

comparison = MagicNumber-10;

if(OldMagicNumber < comparison){
	//MagicNumber = Filter;
	OldMagicNumber=MagicNumber;
}

comparison =  MagicNumber+10;
if(OldMagicNumber > comparison){
	OldMagicNumber=MagicNumber;
} 

if(MagicNumber > 10){
	DispValue= OldMagicNumber;
} else {
	DispValue = 0;
}
		
		
//DispValue = FilterDepth;

//Ludzinc
//DispValue = (Filter/1024);
//DispValue = DispValue * ScaleMax;
//DispValue = DispValue + ScaleOffset;
}