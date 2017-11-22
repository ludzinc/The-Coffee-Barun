
; CC5X Version 3.2, Copyright (c) B Knudsen Data
; C compiler for the PICmicro family
; ************   5. Sep 2011  12:50  *************

	processor  16F88
	radix  DEC

INDF        EQU   0x00
STATUS      EQU   0x03
FSR         EQU   0x04
PORTA       EQU   0x05
TRISA       EQU   0x85
PORTB       EQU   0x06
TRISB       EQU   0x86
PCLATH      EQU   0x0A
INTCON      EQU   0x0B
Carry       EQU   0
Zero_       EQU   2
RP0         EQU   5
RP1         EQU   6
IRP         EQU   7
OPTION_REG  EQU   0x81
PIR1        EQU   0x0C
T2CON       EQU   0x12
ADRESH      EQU   0x1E
ADCON0      EQU   0x1F
PIE1        EQU   0x8C
PCON        EQU   0x8E
OSCCON      EQU   0x8F
PR2         EQU   0x92
ANSEL       EQU   0x9B
CMCON       EQU   0x9C
ADRESL      EQU   0x9E
ADCON1      EQU   0x9F
TMR2IF      EQU   1
FSRTemp     EQU   0x3A
DigitCounter EQU   0x3B
DispValue   EQU   0x3C
A2DResult   EQU   0x3E
Digit       EQU   0x68
FilterIndex EQU   0x6C
MagicNumber EQU   0x6D
OldMagicNumber EQU   0x71
svrWREG     EQU   0x70
svrSTATUS   EQU   0x20
svrPCLATH   EQU   0x21
FpOverflow  EQU   1
FpUnderFlow EQU   2
FpRounding  EQU   6
arg1f24     EQU   0x30
arg2f24     EQU   0x33
aarg        EQU   0x36
sign        EQU   0x38
counter     EQU   0x39
xtra        EQU   0x36
temp        EQU   0x37
expo        EQU   0x38
sign_3      EQU   0x39
expo_2      EQU   0x36
xtra_2      EQU   0x37
sign_4      EQU   0x38
rval        EQU   0x30
sign_6      EQU   0x36
expo_4      EQU   0x37
xtra_4      EQU   0x38
rval_3      EQU   0x30
i           EQU   0x29
DigitCounter_2 EQU   0x22
temp_2      EQU   0x22
C1cnt       EQU   0x24
C2tmp       EQU   0x25
C3cnt       EQU   0x24
C4tmp       EQU   0x25
C5rem       EQU   0x27
C6cnt       EQU   0x24
C7tmp       EQU   0x25
C8cnt       EQU   0x24
C9tmp       EQU   0x25
C10rem      EQU   0x27
C11cnt      EQU   0x24
C12tmp      EQU   0x25
C13cnt      EQU   0x24
C14tmp      EQU   0x25
C15rem      EQU   0x27
Filter      EQU   0x29
i_2         EQU   0x2B
filtertemp  EQU   0x2C
comparison  EQU   0x2E
C16cnt      EQU   0x30
C17tmp      EQU   0x31
C18rem      EQU   0x33

	GOTO main

  ; FILE RPM.c
			;//#define   PIC16F88
			;
			;//#include "16F88.H"
			;#include "INT16CXX.H"
			;
			;
			;#pragma origin = 0x0004																	// CHECK THIS IN DATASHEET.  WHAT 0x0004 ???
	ORG 0x0004
			;#pragma config        	= 0b0011.1111.0101.1000   	// MCLR Pin is input
			;#pragma config reg2     = 0b0011.1111.1111.1100   
			;
			;#define BCD_A			PORTB.0	//RB0		// OUTPUT, BIT0 BCD WORD
			;#define BCD_B      		PORTB.1 //RB1   	// OUTPUT, BIT1 BCD WORD  
			;#define BCD_C			PORTB.2 //RB2   	// OUTPUT, BIT2 BCD WORD		
			;#define BCD_D	 	 	PORTB.3 //RB3   	// OUTPUT, BIT3 BCD WORD
			;#define MC14543_BI_CTRL PORTB.4 //RB4		// Set low to disable blanking
			;#define MC14543_LD_CTRL	PORTB.5 //RB5		// Rising Edge Load to BCD DRiver
			;
			;#define DISP0			PORTA.6	//RA2
			;#define DISP1			PORTA.4 //RA3
			;#define DISP2			PORTA.3 //RA4	
			;#define DISP3			PORTA.2 //RA6
			;#define DECIMAL			PORTA.7 //RA7
			;
			;#define ScaleRange		1850
			;#define ScaleOffset		500
			;#define FilterDepth		20
			;
			;// Declare Function Prototypes
			;void InitialiseHardware(void);
			;void InitialiseGlobals(void);
			;void DisplayDigit(char DigitCounter);
			;void Dec2BCD(void);
			;void GetA2D(void);
			;
			;char FSRTemp;
			;char DigitCounter;
			;uns16 DispValue;		// unsigned 16 bit integer
			;uns16 A2DResult;
			;uns16 A2DFilter[FilterDepth];
			;char Digit[4];
			;char FilterIndex;
			;
			;float MagicNumber;
			;float OldMagicNumber;
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
	MOVF  PCLATH,W
	MOVWF svrPCLATH
	CLRF  PCLATH
			;   	FSRTemp = FSR;
	MOVF  FSR,W
	MOVWF FSRTemp
			;	if (TMR2IF == 1) {
	BTFSS 0x0C,TMR2IF
	GOTO  m001
			;		DigitCounter++;				// Every 10ms Update the display digits
	INCF  DigitCounter,1
			;		if (DigitCounter == 4) {	// 0 = Units, 1 = tens, 2 = hundreds, 3 = thousands
	MOVF  DigitCounter,W
	XORLW .4
	BTFSC 0x03,Zero_
			;			DigitCounter = 0;
	CLRF  DigitCounter
			;			}
			;		Dec2BCD();
	CALL  Dec2BCD
			;		DisplayDigit(DigitCounter);
	MOVF  DigitCounter,W
	CALL  DisplayDigit
			;		TMR2IF = 0;
	BCF   0x0C,TMR2IF
			;	}
			;   	FSR = FSRTemp;
m001	MOVF  FSRTemp,W
	MOVWF FSR
			;   	int_restore_registers
	MOVF  svrPCLATH,W
	MOVWF PCLATH
	SWAPF svrSTATUS,W
	MOVWF STATUS
	SWAPF svrWREG,1
	SWAPF svrWREG,W
			;}
	RETFIE

  ; FILE Math24f.h
			;// *************************************************
			;// 24 bit basic floating point math operations
			;// Copyright (c) B Knudsen Data, Norway, 2000 - 2009
			;// *************************************************
			;
			;#pragma library 1
			;/* PROTOTYPES for page definition in application header file:
			;float24 operator* _fmul24( float24 arg1f24, float24 arg2f24);
			;float24 operator/ _fdiv24( float24 arg1f24, float24 arg2f24);
			;float24 operator+ _fadd24( float24 arg1f24, float24 arg2f24);
			;float24 operator- _fsub24( float24 arg1f24, float24 arg2f24);
			;float24 operator= _int24ToFloat24( int24 arg1f24);
			;float24 operator= _int32ToFloat24( int32 arg32);
			;int24 operator= _float24ToInt24( float24 arg1f24);
			;bit operator< _f24_LT_f24( float24 arg1f24, float24 arg2f24);
			;bit operator>= _f24_GE_f24( float24 arg1f24, float24 arg2f24);
			;bit operator> _f24_GT_f24( float24 arg1f24, float24 arg2f24);
			;bit operator<= _f24_LE_f24( float24 arg1f24, float24 arg2f24);
			;*/
			;
			;// DEFINABLE SYMBOLS (in the application code):
			;//#define FP_OPTIM_SPEED  // optimize for SPEED: default
			;//#define FP_OPTIM_SIZE   // optimize for SIZE
			;//#define DISABLE_ROUNDING   // disable rounding and save code space
			;
			;#define float24ToIEEE754(a) { a.mid8=rl(a.mid8); a.high8=rr(a.high8);\
			;                              a.mid8=rr(a.mid8); }
			;#define IEEE754ToFloat24(a) { a.mid8=rl(a.mid8); a.high8=rl(a.high8);\
			;                              a.mid8=rr(a.mid8); }
			;
			;
			;/*  24 bit floating point format:
			;
			;  address  ID
			;    X      a.low8  : LSB, bit 0-7 of mantissa
			;    X+1    a.mid8  : bit 8-14 of mantissa, bit 15 is the sign bit
			;    X+2    a.high8 : MSB, bit 0-7 of exponent, with bias 0x7F
			;
			;    bit 15 of mantissa is a hidden bit, always equal to 1
			;    zero (0.0) :  a.high8 = 0 (mantissa & sign ignored)
			;
			;   MSB    LSB
			;    7F 00 00  : 1.0   =  1.0  * 2**(0x7F-0x7F) = 1.0 * 1
			;    7F 80 00  : -1.0  = -1.0  * 2**(0x7F-0x7F) = -1.0 * 1
			;    80 00 00  : 2.0   =  1.0  * 2**(0x80-0x7F) = 1.0 * 2
			;    80 40 00  : 3.0   =  1.5  * 2**(0x80-0x7F) = 1.5 * 2
			;    7E 60 00  : 0.875 =  1.75 * 2**(0x7E-0x7F) = 1.75 * 0.5
			;    7F 60 00  : 1.75  =  1.75 * 2**(0x7E-0x7F) = 1.75 * 1
			;    7F 7F FF  : 1.999969482
			;    00 7C 5A  : 0.0 (mantissa & sign ignored)
			;    01 00 00  : 1.17549435e-38 =  1.0 * 2**(0x01-0x7F)
			;    FE 7F FF  : 3.40277175e+38 =  1.999969482 * 2**(0xFE-0x7F)
			;    FF 00 00  : +INF : positive infinity
			;    FF 80 00  : -INF : negative infinity
			;*/                 
			;
			;#define  FpBIAS  0x7F
			;
			;#ifndef FpFlags_defined
			; #define FpFlags_defined
			;
			; char FpFlags;
			; //bit IOV         @ FpFlags.0; // integer overflow flag: NOT USED
			; bit FpOverflow    @ FpFlags.1; // floating point overflow flag
			; bit FpUnderFlow   @ FpFlags.2; // floating point underflow flag
			; bit FpDiv0        @ FpFlags.3; // floating point divide by zero flag
			; //bit FpNAN       @ FpFlags.4; // not-a-number exception flag: NOT USED
			; bit FpDomainError @ FpFlags.5; // domain error exception flag
			; bit FpRounding    @ FpFlags.6; // floating point rounding flag, 0=truncation
			;                                // 1 = unbiased rounding to nearest LSB
			; //bit FpSaturate  @ FpFlags.7; // floating point saturate flag: NOT USED
			;
			; #pragma floatOverflow FpOverflow
			; #pragma floatUnderflow FpUnderFlow
			;
			; #define InitFpFlags()  FpFlags = 0x40 /* enable rounding as default */
			;#endif
			;
			;#ifdef DISABLE_ROUNDING
			; #pragma floatRounding 0
			;#endif
			;
			;
			;#if __CoreSet__ < 1410
			; #define genAdd(r,a) W=a; btsc(Carry); W=incsz(a); r+=W
			; #define genSub(r,a) W=a; btss(Carry); W=incsz(a); r-=W
			; #define genAddW(r,a) W=a; btsc(Carry); W=incsz(a); W=r+W
			; #define genSubW(r,a) W=a; btss(Carry); W=incsz(a); W=r-W
			;#else
			; #define genAdd(r,a) W=a; r=addWFC(r)
			; #define genSub(r,a) W=a; r=subWFB(r)
			; #define genAddW(r,a) W=a; W=addWFC(r)
			; #define genSubW(r,a) W=a; W=subWFB(r)
			;#endif
			;
			;
			;
			;float24 operator* _fmul24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_fmul24
			;    uns16 aarg;
			;    W = arg1f24.mid8;
	MOVF  arg1f24+1,W
			;    aarg.high8 = W;
	MOVWF aarg+1
			;
			;    // save sign
			;    char sign = arg2f24.mid8 ^ W;  // before first overflow test
	XORWF arg2f24+1,W
	MOVWF sign
			;
			;    W = arg1f24.high8;
	MOVF  arg1f24+2,W
			;    if (!Zero_)
	BTFSS 0x03,Zero_
			;        W = arg2f24.high8;
	MOVF  arg2f24+2,W
			;    if (Zero_)
	BTFSC 0x03,Zero_
			;        goto RES0;
	GOTO  m008
			;
			;    arg1f24.high8 += W /* arg2f24.high8 */;
	ADDWF arg1f24+2,1
			;    W = FpBIAS-1;
	MOVLW .126
			;    if (Carry)  {
	BTFSS 0x03,Carry
	GOTO  m002
			;        arg1f24.high8 -= W;
	SUBWF arg1f24+2,1
			;        if (Carry)
	BTFSS 0x03,Carry
	GOTO  m003
			;            goto OVERFLOW;
	GOTO  m009
			;    }
			;    else  {
			;        arg1f24.high8 -= W;
m002	SUBWF arg1f24+2,1
			;        if (!Carry)
	BTFSS 0x03,Carry
			;            goto UNDERFLOW;
	GOTO  m007
			;    }
			;    aarg.low8 = arg1f24.low8;
m003	MOVF  arg1f24,W
	MOVWF aarg
			;
			;    aarg.15 = 1;
	BSF   aarg+1,7
			;    arg2f24.15 = 1;
	BSF   arg2f24+1,7
			;
			;    arg1f24.low16 = 0;
	CLRF  arg1f24
	CLRF  arg1f24+1
			;
			;    char counter = sizeof(aarg)*8;
	MOVLW .16
	MOVWF counter
			;
			;    do  {
			;        aarg = rr( aarg);
m004	RRF   aarg+1,1
	RRF   aarg,1
			;        if (Carry)  {
	BTFSS 0x03,Carry
	GOTO  m005
			;            arg1f24.low8 += arg2f24.low8;
	MOVF  arg2f24,W
	ADDWF arg1f24,1
			;            genAdd( arg1f24.mid8, arg2f24.mid8);
	MOVF  arg2f24+1,W
	BTFSC 0x03,Carry
	INCFSZ arg2f24+1,W
	ADDWF arg1f24+1,1
			;        }
			;        arg1f24.low16 = rr( arg1f24.low16);
m005	RRF   arg1f24+1,1
	RRF   arg1f24,1
			;    } while (-- counter > 0);
	DECFSZ counter,1
	GOTO  m004
			;
			;    if (!arg1f24.15)  {
	BTFSC arg1f24+1,7
	GOTO  m006
			;        // catch Carry bit that was shifted out previously
			;        arg1f24.low16 = rl( arg1f24.low16);
	RLF   arg1f24,1
	RLF   arg1f24+1,1
			;        if (arg1f24.high8 == 0)
	MOVF  arg1f24+2,1
	BTFSC 0x03,Zero_
			;            goto UNDERFLOW;
	GOTO  m007
			;        arg1f24.high8 -= 1;
	DECF  arg1f24+2,1
			;        W = rl( aarg.high8);
	RLF   aarg+1,W
			;        // restore bit behind LSB in Carry
			;    }
			;
			;   #ifndef DISABLE_ROUNDING
			;    if (FpRounding  &&  Carry)  {
m006	BTFSS 0x74,FpRounding
	GOTO  m011
	BTFSS 0x03,Carry
	GOTO  m011
			;        arg1f24.low8 += 1;
	INCFSZ arg1f24,1
			;        if (!arg1f24.low8)  {
	GOTO  m011
			;            arg1f24.mid8 += 1;
	INCFSZ arg1f24+1,1
			;            if (!arg1f24.mid8)  {
	GOTO  m011
			;                // Carry = 1; //OK
			;                arg1f24.low16 = rr( arg1f24.low16);
	RRF   arg1f24+1,1
	RRF   arg1f24,1
			;                arg1f24.high8 += 1;
	INCFSZ arg1f24+2,1
			;                if (Zero_)
	GOTO  m011
			;                    goto OVERFLOW;
	GOTO  m009
			;            }
			;        }
			;    }
			;   #endif
			;    goto SET_SIGN;
			;
			;  UNDERFLOW:
			;    FpUnderFlow = 1;
m007	BSF   0x74,FpUnderFlow
			;  RES0:
			;    arg1f24.high8 = 0;
m008	CLRF  arg1f24+2
			;    goto MANTISSA;
	GOTO  m010
			;
			;  OVERFLOW:
			;    FpOverflow = 1;
m009	BSF   0x74,FpOverflow
			;    arg1f24.high8 = 0xFF;
	MOVLW .255
	MOVWF arg1f24+2
			;  MANTISSA:
			;    arg1f24.low16 = 0x8000;
m010	CLRF  arg1f24
	MOVLW .128
	MOVWF arg1f24+1
			;
			;  SET_SIGN:
			;    if (!(sign & 0x80))
m011	BTFSS sign,7
			;        arg1f24.15 = 0;
	BCF   arg1f24+1,7
			;    return arg1f24;
	MOVF  arg1f24,W
	RETURN
			;}
			;
			;
			;
			;float24 operator/ _fdiv24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_fdiv24
			;    uns16 aarg;
			;    W = arg1f24.mid8;
			;    aarg.high8 = W;
			;
			;    // save sign
			;    char sign = arg2f24.mid8 ^ W;  // before first overflow test
			;
			;    W = arg2f24.high8;
			;    if (Zero_)
			;        goto Div0;
			;    if (!arg1f24.high8)
			;        goto RES0;
			;
			;    arg1f24.high8 -= arg2f24.high8;
			;    W = FpBIAS;
			;    if (!Carry)  {
			;        arg1f24.high8 += W;
			;        if (!Carry)
			;            goto UNDERFLOW;
			;    }
			;    else  {
			;        arg1f24.high8 += W;
			;        if (Carry)
			;            goto OVERFLOW;
			;    }
			;
			;    aarg.low8 = arg1f24.low8;
			;    aarg.15 = 1;
			;    arg2f24.15 = 1;
			;
			;    // division: shift & add
			;    char counter = 16;
			;    arg1f24.low16 = 0;  // speedup
			;
			;#if defined FP_OPTIM_SPEED || !defined FP_OPTIM_SIZE  // SPEED
			;
			;    goto START_ML;
			;
			;  TEST_ZERO_L:
			;    W = aarg.low8 - arg2f24.low8;
			;    if (!Carry)
			;        goto SHIFT_IN_CARRY;
			;    aarg.low8 = W;
			;    aarg.high8 = 0;
			;    goto SET_AND_SHIFT_IN_CARRY;
			;
			;// MAIN LOOP
			;    do  {
			;      LOOP_ML:
			;        if (!Carry)  {
			;           START_ML:
			;            W = aarg.high8 - arg2f24.mid8;
			;            if (Zero_)
			;                goto TEST_ZERO_L;
			;            if (!Carry)
			;                goto SHIFT_IN_CARRY;
			;        }
			;        aarg.low8 -= arg2f24.low8;
			;        genSub( aarg.high8, arg2f24.mid8);
			;      SET_AND_SHIFT_IN_CARRY:
			;        Carry = 1;
			;      SHIFT_IN_CARRY:
			;        arg1f24.low16 = rl( arg1f24.low16);
			;        // Carry = 0;  // ok, speedup
			;        aarg = rl( aarg);
			;    } while (-- counter > 0);
			;
			;
			;
			;#else  // SIZE
			;
			;    goto START_ML;
			;
			;// MAIN LOOP
			;    do  {
			;      LOOP_ML:
			;        if (Carry)
			;            goto SUBTRACT;
			;      START_ML:
			;        W = aarg.low8 - arg2f24.low8;
			;        genSubW( aarg.high8, arg2f24.mid8);
			;        if (!Carry)
			;            goto SKIP_SUB;
			;       SUBTRACT:
			;        aarg.low8 -= arg2f24.low8;
			;        genSub( aarg.high8, arg2f24.mid8);
			;        Carry = 1;
			;       SKIP_SUB:
			;        arg1f24.low16 = rl( arg1f24.low16);
			;        // Carry = 0;  // ok
			;        aarg = rl( aarg);
			;    } while (-- counter > 0);
			;
			;#endif
			;
			;    if (!arg1f24.15)  {
			;        if (!arg1f24.high8)
			;            goto UNDERFLOW;
			;        arg1f24.high8 --;
			;        counter ++;
			;        goto LOOP_ML;
			;    }
			;
			;   #ifndef DISABLE_ROUNDING
			;    if (FpRounding)  {
			;        if (Carry)
			;            goto ADD_1;
			;        aarg.low8 -= arg2f24.low8;
			;        genSub( aarg.high8, arg2f24.mid8);
			;        if (Carry)  {
			;          ADD_1:
			;            arg1f24.low8 += 1;
			;            if (!arg1f24.low8)  {
			;                arg1f24.mid8 ++;
			;                if (!arg1f24.mid8)  {
			;                    arg1f24.low16 = rr( arg1f24.low16);
			;                    arg1f24.high8 ++;
			;                    if (!arg1f24.high8)
			;                        goto OVERFLOW;
			;                }
			;            }
			;        }
			;    }
			;   #endif
			;    goto SET_SIGN;
			;
			;  Div0:
			;    FpDiv0 = 1;
			;    goto SATURATE;
			;
			;  UNDERFLOW:
			;    FpUnderFlow = 1;
			;  RES0:
			;    arg1f24.high8 = 0;
			;    goto MANTISSA;
			;
			;  OVERFLOW:
			;    FpOverflow = 1;
			;  SATURATE:
			;    arg1f24.high8 = 0xFF;
			;  MANTISSA:
			;    arg1f24.low16 = 0x8000;
			;
			;  SET_SIGN:
			;    if (!(sign & 0x80))
			;        arg1f24.15 = 0;
			;    return arg1f24;
			;}
			;
			;
			;float24 operator+ _fadd24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_fadd24
			;    char xtra, temp;
			;    char expo = arg1f24.high8 - arg2f24.high8;
	MOVF  arg2f24+2,W
	SUBWF arg1f24+2,W
	MOVWF expo
			;    if (!Carry)  {
	BTFSC 0x03,Carry
	GOTO  m012
			;        expo = -expo;
	COMF  expo,1
	INCF  expo,1
			;        temp = arg1f24.high8;
	MOVF  arg1f24+2,W
	MOVWF temp
			;        arg1f24.high8 = arg2f24.high8;
	MOVF  arg2f24+2,W
	MOVWF arg1f24+2
			;        arg2f24.high8 = temp;
	MOVF  temp,W
	MOVWF arg2f24+2
			;        temp = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF temp
			;        arg1f24.mid8 = arg2f24.mid8;
	MOVF  arg2f24+1,W
	MOVWF arg1f24+1
			;        arg2f24.mid8 = temp;
	MOVF  temp,W
	MOVWF arg2f24+1
			;        temp = arg1f24.low8;
	MOVF  arg1f24,W
	MOVWF temp
			;        arg1f24.low8 = arg2f24.low8;
	MOVF  arg2f24,W
	MOVWF arg1f24
			;        arg2f24.low8 = temp;
	MOVF  temp,W
	MOVWF arg2f24
			;    }
			;    if (expo > sizeof(arg1f24)*8-7)
m012	MOVLW .18
	SUBWF expo,W
	BTFSC 0x03,Carry
			;        goto _RETURN_MF;
	GOTO  m030
			;    if (!arg2f24.high8)
	MOVF  arg2f24+2,1
	BTFSC 0x03,Zero_
			;        goto _RETURN_MF;   // result is arg1f24
	GOTO  m030
			;
			;    xtra = 0;
	CLRF  xtra
			;
			;    temp = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF temp
			;    char sign = arg2f24.mid8 ^ arg1f24.mid8;
	XORWF arg2f24+1,W
	MOVWF sign_3
			;    arg1f24.15 = 1;
	BSF   arg1f24+1,7
			;    arg2f24.15 = 1;
	BSF   arg2f24+1,7
			;
			;    while (1)  {
			;        W = 8;
m013	MOVLW .8
			;        expo -= W;
	SUBWF expo,1
			;        if (!Carry)
	BTFSS 0x03,Carry
			;            break;
	GOTO  m014
			;        xtra = arg2f24.low8;
	MOVF  arg2f24,W
	MOVWF xtra
			;        arg2f24.low8 = arg2f24.mid8;
	MOVF  arg2f24+1,W
	MOVWF arg2f24
			;        arg2f24.mid8 = 0;
	CLRF  arg2f24+1
			;    }
	GOTO  m013
			;    expo += W;
m014	ADDWF expo,1
			;    if (expo)  {
	BTFSC 0x03,Zero_
	GOTO  m016
			;        do  {
			;            Carry = 0;
m015	BCF   0x03,Carry
			;            arg2f24.low16 = rr( arg2f24.low16);
	RRF   arg2f24+1,1
	RRF   arg2f24,1
			;            xtra = rr( xtra);
	RRF   xtra,1
			;        } while (--expo > 0);
	DECFSZ expo,1
	GOTO  m015
			;    }
			;
			;
			;    if (sign & 0x80)  {
m016	BTFSS sign_3,7
	GOTO  m022
			;        // SUBTRACT
			;        arg1f24.low8 -= arg2f24.low8;
	MOVF  arg2f24,W
	SUBWF arg1f24,1
			;        genSub( arg1f24.mid8, arg2f24.mid8);
	MOVF  arg2f24+1,W
	BTFSS 0x03,Carry
	INCFSZ arg2f24+1,W
	SUBWF arg1f24+1,1
			;        if (!Carry)  {  // arg2f24 > arg1f24
	BTFSC 0x03,Carry
	GOTO  m017
			;            arg1f24.low16 = -arg1f24.low16;
	COMF  arg1f24+1,1
	COMF  arg1f24,1
	INCF  arg1f24,1
	BTFSC 0x03,Zero_
	INCF  arg1f24+1,1
			;            // xtra == 0 because arg1f24.exp == arg2f24.exp
			;            temp ^= 0x80;  // invert sign
	MOVLW .128
	XORWF temp,1
			;        }
			;        xtra = -xtra;
m017	COMF  xtra,1
	INCF  xtra,1
			;        if (xtra)
	BTFSC 0x03,Zero_
	GOTO  m018
			;            arg1f24.low16 --;
	DECF  arg1f24,1
	INCF  arg1f24,W
	BTFSC 0x03,Zero_
	DECF  arg1f24+1,1
			;        // adjust result left
			;       #define counter expo
			;        counter = 3;
m018	MOVLW .3
	MOVWF expo
			;        while (!arg1f24.mid8)  {
m019	MOVF  arg1f24+1,1
	BTFSS 0x03,Zero_
	GOTO  m020
			;            arg1f24.mid8 = arg1f24.low8;
	MOVF  arg1f24,W
	MOVWF arg1f24+1
			;            arg1f24.low8 = xtra;
	MOVF  xtra,W
	MOVWF arg1f24
			;            xtra = 0;
	CLRF  xtra
			;            arg1f24.high8 -= 8;
	MOVLW .8
	SUBWF arg1f24+2,1
			;            if (!Carry)
	BTFSS 0x03,Carry
			;                goto RES0;
	GOTO  m026
			;            if (--counter == 0)  // max 2 iterations
	DECFSZ expo,1
	GOTO  m019
			;                goto RES0;
	GOTO  m026
			;        }
			;       #undef counter
			;        while (!arg1f24.15)  {
m020	BTFSC arg1f24+1,7
	GOTO  m021
			;            Carry = 0;
	BCF   0x03,Carry
			;            xtra = rl( xtra);
	RLF   xtra,1
			;            arg1f24.low16 = rl( arg1f24.low16);
	RLF   arg1f24,1
	RLF   arg1f24+1,1
			;            arg1f24.high8 --;
	DECFSZ arg1f24+2,1
			;            if (!arg1f24.high8)
	GOTO  m020
			;                goto RES0;   // UNDERFLOW?
	GOTO  m026
			;        }
			;       #ifndef DISABLE_ROUNDING
			;        if (FpRounding  &&  (xtra & 0x80))  {
m021	BTFSS 0x74,FpRounding
	GOTO  m029
	BTFSS xtra,7
	GOTO  m029
			;            xtra = 0; // disable recursion
	CLRF  xtra
			;            goto INCREMENT;
	GOTO  m025
			;        }
			;       #endif
			;    }
			;    else  {
			;        // ADD arg1f24 and arg2f24
			;        arg1f24.low8 += arg2f24.low8;
m022	MOVF  arg2f24,W
	ADDWF arg1f24,1
			;        genAdd( arg1f24.mid8, arg2f24.mid8);
	MOVF  arg2f24+1,W
	BTFSC 0x03,Carry
	INCFSZ arg2f24+1,W
	ADDWF arg1f24+1,1
			;        if (Carry)  {
	BTFSS 0x03,Carry
	GOTO  m024
			;          ADJUST_RIGHT:
			;            arg1f24.low16 = rr( arg1f24.low16);
m023	RRF   arg1f24+1,1
	RRF   arg1f24,1
			;            xtra = rr( xtra);
	RRF   xtra,1
			;            arg1f24.high8 += 1;  // exp
	INCF  arg1f24+2,1
			;            if (!arg1f24.high8)
	BTFSC 0x03,Zero_
			;                goto OVERFLOW;
	GOTO  m027
			;        }
			;       #ifndef DISABLE_ROUNDING
			;        if (FpRounding  &&  (xtra & 0x80))  {
m024	BTFSS 0x74,FpRounding
	GOTO  m029
	BTFSS xtra,7
	GOTO  m029
			;          INCREMENT:
			;            arg1f24.low8 += 1;
m025	INCFSZ arg1f24,1
			;            if (!arg1f24.low8)  {
	GOTO  m029
			;                arg1f24.mid8 += 1;
	INCFSZ arg1f24+1,1
			;                if (!arg1f24.mid8)  {
	GOTO  m029
			;                    Carry = 1; // prepare for shift
	BSF   0x03,Carry
			;                    arg1f24.0 = 0;  // disable recursion
	BCF   arg1f24,0
			;                    goto ADJUST_RIGHT;
	GOTO  m023
			;                }
			;            }
			;        }
			;       #endif
			;    }
			;    goto SET_SIGN;
			;
			;//  UNDERFLOW:
			;//    FpUnderFlow = 1;
			;  RES0:
			;    arg1f24.high8 = 0;
m026	CLRF  arg1f24+2
			;    goto MANTISSA;
	GOTO  m028
			;
			;  OVERFLOW:
			;    FpOverflow = 1;
m027	BSF   0x74,FpOverflow
			;    arg1f24.high8 = 0xFF;
	MOVLW .255
	MOVWF arg1f24+2
			;  MANTISSA:
			;    arg1f24.low16 = 0x8000;
m028	CLRF  arg1f24
	MOVLW .128
	MOVWF arg1f24+1
			;
			;  SET_SIGN:
			;    if (!(temp & 0x80))
m029	BTFSS temp,7
			;        arg1f24.15 = 0;
	BCF   arg1f24+1,7
			;
			;  _RETURN_MF:
			;    return arg1f24;
m030	MOVF  arg1f24,W
	RETURN
			;}
			;
			;
			;// SUBTRACTION
			;
			;float24 operator- _fsub24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_fsub24
			;    arg2f24.mid8 ^= 0x80;
	MOVLW .128
	XORWF arg2f24+1,1
			;    arg1f24 += arg2f24;
	CALL  _fadd24
			;    return arg1f24;
	MOVF  arg1f24,W
	RETURN
			;}
			;
			;
			;float24 operator=( int8 arg) @
			;float24 operator=( uns8 arg) @
			;float24 operator=( int16 arg) @
			;float24 operator=( uns16 arg) @
			;float24 operator= _int24ToFloat24( sharedM int24 arg1f24)
			;{
_int24ToFloat24
			;    sharedM float24 arg2f24;   // unused, but required
			;    char expo = FpBIAS + 16 - 1;
	MOVLW .142
	MOVWF expo_2
			;    char xtra = 0;
	CLRF  xtra_2
			;    char sign = 0;
	CLRF  sign_4
			;    if (arg1f24 < 0)  {
	BTFSS arg1f24+2,7
	GOTO  m032
			;        arg1f24 = -arg1f24;
	COMF  arg1f24+2,1
	COMF  arg1f24+1,1
	COMF  arg1f24,1
	INCFSZ arg1f24,1
	GOTO  m031
	INCF  arg1f24+1,1
	BTFSC 0x03,Zero_
	INCF  arg1f24+2,1
			;        sign |= 0x80;
m031	BSF   sign_4,7
			;    }
			;    if (arg1f24.high8)  {
m032	MOVF  arg1f24+2,1
	BTFSC 0x03,Zero_
	GOTO  m033
			;        expo += 8;
	MOVLW .8
	ADDWF expo_2,1
			;        xtra = arg1f24.low8;
	MOVF  arg1f24,W
	MOVWF xtra_2
			;        arg1f24.low8 = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF arg1f24
			;        arg1f24.mid8 = arg1f24.high8;
	MOVF  arg1f24+2,W
	MOVWF arg1f24+1
			;    }
			;    else if (!arg1f24.mid8)  {
	GOTO  m035
m033	MOVF  arg1f24+1,1
	BTFSS 0x03,Zero_
	GOTO  m035
			;        expo -= 8;
	MOVLW .8
	SUBWF expo_2,1
			;        W = arg1f24.low8;
	MOVF  arg1f24,W
			;        if (!W)
	BTFSC 0x03,Zero_
			;            goto _RETURN_MF;
	GOTO  m037
			;        arg1f24.mid8 = W;
	MOVWF arg1f24+1
			;        arg1f24.low8 = 0;
	CLRF  arg1f24
			;    }
			;
			;    // arg1f24.mid8 != 0
			;    goto TEST_ARG1_B15;
	GOTO  m035
			;    do  {
			;        xtra = rl( xtra);
m034	RLF   xtra_2,1
			;        arg1f24.low16 = rl( arg1f24.low16);
	RLF   arg1f24,1
	RLF   arg1f24+1,1
			;        expo --;
	DECF  expo_2,1
			;      TEST_ARG1_B15:
			;    } while (!arg1f24.15);
m035	BTFSS arg1f24+1,7
	GOTO  m034
			;
			;   #ifndef DISABLE_ROUNDING
			;    if (FpRounding && (xtra & 0x80))  {
	BTFSS 0x74,FpRounding
	GOTO  m036
	BTFSS xtra_2,7
	GOTO  m036
			;        arg1f24.low8 += 1;
	INCFSZ arg1f24,1
			;        if (!arg1f24.low8)  {
	GOTO  m036
			;            arg1f24.mid8 += 1;
	INCFSZ arg1f24+1,1
			;            if (!arg1f24.mid8)  {
	GOTO  m036
			;                Carry = 1;
	BSF   0x03,Carry
			;                arg1f24.low16 = rr( arg1f24.low16);
	RRF   arg1f24+1,1
	RRF   arg1f24,1
			;                expo ++;
	INCF  expo_2,1
			;            }
			;        }
			;    }
			;   #endif
			;
			;    arg1f24.high8 = expo;
m036	MOVF  expo_2,W
	MOVWF arg1f24+2
			;    if (!(sign & 0x80))
	BTFSS sign_4,7
			;        arg1f24.15 = 0;
	BCF   arg1f24+1,7
			;
			;  _RETURN_MF:
			;    float24 rval @ arg1f24;
			;    rval.low24 = arg1f24.low24;
			;    return rval;
m037	MOVF  rval,W
	RETURN
			;}
			;
			;
			;float24 operator=( uns24 arg) @
			;float24 operator= _int32ToFloat24( int32 arg32)
			;{
_int32ToFloat24
			;    char expo = FpBIAS + 16 - 1;
			;    char xtra @ arg32.high8;
			;    char sign = 0;
			;    if (arg32 < 0)  {
			;        arg32 = -arg32;
			;        sign |= 0x80;
			;    }
			;    if (arg32.high8)  {
			;        expo += 8;
			;        arg32.low8 = arg32.midL8;
			;        arg32.midL8 = arg32.midH8;
			;        arg32.midH8 = arg32.high8;
			;        arg32.high8 = 0;
			;    }
			;    if (arg32.midH8)  {
			;        expo += 8;
			;        xtra = arg32.low8;
			;        arg32.low8 = arg32.midL8;
			;        arg32.midL8 = arg32.midH8;
			;    }
			;    else if (!arg32.midL8)  {
			;        expo -= 8;
			;        W = arg32.low8;
			;        if (!W)
			;            goto _RETURN_MF;
			;        arg32.midL8 = W;
			;        arg32.low8 = 0;
			;    }
			;
			;    // arg32.midL8 != 0
			;    goto TEST_ARG_B15;
			;    do  {
			;        xtra = rl( xtra);
			;        arg32.low16 = rl( arg32.low16);
			;        expo --;
			;      TEST_ARG_B15:
			;    } while (!arg32.15);
			;
			;   #ifndef DISABLE_ROUNDING
			;    if (FpRounding && (xtra & 0x80))  {
			;        arg32.low8 += 1;
			;        if (!arg32.low8)  {
			;            arg32.midL8 += 1;
			;            if (!arg32.midL8)  {
			;                Carry = 1;
			;                arg32.low16 = rr( arg32.low16);
			;                expo ++;
			;            }
			;        }
			;    }
			;   #endif
			;
			;    arg32.midH8 = expo;
			;    if (!(sign & 0x80))
			;        arg32.15 = 0;
			;
			;  _RETURN_MF:
			;    float24 rval @ arg32;
			;    rval.low24 = arg32.low24;
			;    return rval;
			;}
			;
			;
			;uns8 operator=( sharedM float24 arg1f24) @
			;int8 operator=( sharedM float24 arg1f24) @
			;uns16 operator=( sharedM float24 arg1f24) @
			;int16 operator=( sharedM float24 arg1f24) @
			;int24 operator= _float24ToInt24( sharedM float24 arg1f24)
			;{
_float24ToInt24
			;    sharedM float24 arg2f24;   // unused, but required
			;    char sign = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF sign_6
			;    char expo = arg1f24.high8 - (FpBIAS-1);
	MOVLW .126
	SUBWF arg1f24+2,W
	MOVWF expo_4
			;    if (!Carry)
	BTFSS 0x03,Carry
			;        goto RES0;
	GOTO  m043
			;    arg1f24.15 = 1;
	BSF   arg1f24+1,7
			;
			;    arg1f24.high8 = 0;
	CLRF  arg1f24+2
			;   #ifndef DISABLE_ROUNDING
			;    char xtra = 0;
	CLRF  xtra_4
			;   #endif
			;
			;    // (a): expo = 0..8 : shift 1 byte to the right
			;    // (b): expo = 9..16: shift 0 byte
			;    // (c): expo = 17..24: shift 1 byte to the left
			;   #if __CoreSet__ / 100 == 12
			;    expo -= 17;
			;    expo = 0xFF - expo;  // COMF (Carry unchanged)
			;    if (Carry)  {  // (c)
			;   #else
			;    expo = 16 - expo;
	SUBLW .16
	MOVWF expo_4
			;    if (!Carry)  {  // (c)
	BTFSC 0x03,Carry
	GOTO  m038
			;   #endif
			;        expo += 8;
	MOVLW .8
	ADDWF expo_4,1
			;        if (!Carry)
	BTFSS 0x03,Carry
			;            goto OVERFLOW;
	GOTO  m042
			;        arg1f24.high8 = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF arg1f24+2
			;        arg1f24.mid8 = arg1f24.low8;
	MOVF  arg1f24,W
	MOVWF arg1f24+1
			;        arg1f24.low8 = 0;
	CLRF  arg1f24
			;    }
			;    else  {  // (a) (b)
	GOTO  m039
			;        // expo = 0 .. 16
			;        W = expo - 8;
m038	MOVLW .8
	SUBWF expo_4,W
			;        if (Carry)  {  // (a)
	BTFSS 0x03,Carry
	GOTO  m039
			;            expo = W;
	MOVWF expo_4
			;           #ifndef DISABLE_ROUNDING
			;            xtra = arg1f24.low8;
	MOVF  arg1f24,W
	MOVWF xtra_4
			;           #endif
			;            arg1f24.low8 = arg1f24.mid8;
	MOVF  arg1f24+1,W
	MOVWF arg1f24
			;            arg1f24.mid8 = 0;
	CLRF  arg1f24+1
			;        }
			;    }
			;    if (expo)  {
m039	MOVF  expo_4,1
	BTFSC 0x03,Zero_
	GOTO  m041
			;        do  {
			;            Carry = 0;
m040	BCF   0x03,Carry
			;            arg1f24.high8 = rr( arg1f24.high8);
	RRF   arg1f24+2,1
			;            arg1f24.low16 = rr( arg1f24.low16);
	RRF   arg1f24+1,1
	RRF   arg1f24,1
			;           #ifndef DISABLE_ROUNDING
			;            xtra = rr( xtra);
	RRF   xtra_4,1
			;           #endif
			;        } while (--expo);
	DECFSZ expo_4,1
	GOTO  m040
			;    }
			;    if (arg1f24.23)  {
m041	BTFSS arg1f24+2,7
	GOTO  m045
			;       OVERFLOW:
			;        FpOverflow = 1;
m042	BSF   0x74,FpOverflow
			;        W = 0xFF;
	MOVLW .255
			;        goto ASSIGNW;
	GOTO  m044
			;       RES0:
			;        W = 0;
m043	CLRW 
			;       ASSIGNW:
			;        arg1f24.low8 = W;
m044	MOVWF arg1f24
			;        arg1f24.mid8 = W;
	MOVWF arg1f24+1
			;        arg1f24.high8 = W;
	MOVWF arg1f24+2
			;        arg1f24.23 = 0;
	BCF   arg1f24+2,7
			;    }
			;    else  {
	GOTO  m047
			;       #ifndef DISABLE_ROUNDING
			;        if (FpRounding && (xtra & 0x80))  {
m045	BTFSS 0x74,FpRounding
	GOTO  m046
	BTFSS xtra_4,7
	GOTO  m046
			;            arg1f24.low8 += 1;
	INCF  arg1f24,1
			;            if (!arg1f24.low8)
	BTFSC 0x03,Zero_
			;                arg1f24.mid8 += 1;
	INCF  arg1f24+1,1
			;        }
			;       #endif
			;        if (sign & 0x80)
m046	BTFSS sign_6,7
	GOTO  m047
			;            arg1f24.low24 = -arg1f24.low24;
	COMF  arg1f24+2,1
	COMF  arg1f24+1,1
	COMF  arg1f24,1
	INCFSZ arg1f24,1
	GOTO  m047
	INCF  arg1f24+1,1
	BTFSC 0x03,Zero_
	INCF  arg1f24+2,1
			;    }
			;    int24 rval @ arg1f24;
			;    rval = arg1f24.low24;
			;    return rval;
m047	MOVF  rval_3,W
	RETURN
			;}
			;
			;
			;bit operator< _f24_LT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_f24_LT_f24
			;    Carry = 0;
	BCF   0x03,Carry
			;    if (!(arg1f24.high8 | arg2f24.high8))
	MOVF  arg2f24+2,W
	IORWF arg1f24+2,W
	BTFSC 0x03,Zero_
			;        return Carry;
	RETURN
			;    if (!arg1f24.15)  {
	BTFSC arg1f24+1,7
	GOTO  m048
			;        if (arg2f24.15)
	BTFSC arg2f24+1,7
			;            return Carry;
	RETURN
			;        W = arg1f24.low8 - arg2f24.low8;
	MOVF  arg2f24,W
	SUBWF arg1f24,W
			;        genSubW( arg1f24.mid8, arg2f24.mid8);
	MOVF  arg2f24+1,W
	BTFSS 0x03,Carry
	INCFSZ arg2f24+1,W
	SUBWF arg1f24+1,W
			;        genSubW( arg1f24.high8, arg2f24.high8);
	MOVF  arg2f24+2,W
	BTFSS 0x03,Carry
	INCFSZ arg2f24+2,W
	SUBWF arg1f24+2,W
			;        goto _RETURN_MF;
	GOTO  m049
			;    }
			;    if (!arg2f24.15)
m048	BTFSS arg2f24+1,7
			;        goto _RETURN_MF;
	GOTO  m049
			;    W = arg2f24.low8 - arg1f24.low8;
	MOVF  arg1f24,W
	SUBWF arg2f24,W
			;    genSubW( arg2f24.mid8, arg1f24.mid8);
	MOVF  arg1f24+1,W
	BTFSS 0x03,Carry
	INCFSZ arg1f24+1,W
	SUBWF arg2f24+1,W
			;    genSubW( arg2f24.high8, arg1f24.high8);
	MOVF  arg1f24+2,W
	BTFSS 0x03,Carry
	INCFSZ arg1f24+2,W
	SUBWF arg2f24+2,W
			;  _RETURN_MF:
			;    if (Carry)
m049	BTFSS 0x03,Carry
	GOTO  m050
			;        return 0;
	BCF   0x03,Carry
	RETURN
			;    return 1;
m050	BSF   0x03,Carry
	RETURN
			;}
			;
			;
			;bit operator>= _f24_GE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_f24_GE_f24
			;    Carry = 1;
			;    if (!(arg1f24.high8 | arg2f24.high8))
			;        return Carry;
			;    if (!arg1f24.15)  {
			;        if (arg2f24.15)
			;            return Carry;
			;        W = arg1f24.low8 - arg2f24.low8;
			;        genSubW( arg1f24.mid8, arg2f24.mid8);
			;        genSubW( arg1f24.high8, arg2f24.high8);
			;        return Carry;
			;    }
			;    Carry = 0;
			;    if (!arg2f24.15)
			;        return Carry;
			;    W = arg2f24.low8 - arg1f24.low8;
			;    genSubW( arg2f24.mid8, arg1f24.mid8);
			;    genSubW( arg2f24.high8, arg1f24.high8);
			;    return Carry;
			;}
			;
			;
			;
			;bit operator> _f24_GT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_f24_GT_f24
			;    Carry = 0;
	BCF   0x03,Carry
			;    if (!(arg1f24.high8 | arg2f24.high8))
	MOVF  arg2f24+2,W
	IORWF arg1f24+2,W
	BTFSC 0x03,Zero_
			;        return Carry;
	RETURN
			;    if (!arg1f24.15)  {
	BTFSC arg1f24+1,7
	GOTO  m051
			;        if (arg2f24.15)
	BTFSC arg2f24+1,7
			;            goto _RETURN_MF;
	GOTO  m052
			;        W = arg2f24.low8 - arg1f24.low8;
	MOVF  arg1f24,W
	SUBWF arg2f24,W
			;        genSubW( arg2f24.mid8, arg1f24.mid8);
	MOVF  arg1f24+1,W
	BTFSS 0x03,Carry
	INCFSZ arg1f24+1,W
	SUBWF arg2f24+1,W
			;        genSubW( arg2f24.high8, arg1f24.high8);
	MOVF  arg1f24+2,W
	BTFSS 0x03,Carry
	INCFSZ arg1f24+2,W
	SUBWF arg2f24+2,W
			;        goto _RETURN_MF;
	GOTO  m052
			;    }
			;    if (!arg2f24.15)
m051	BTFSS arg2f24+1,7
			;        return Carry;
	RETURN
			;    W = arg1f24.low8 - arg2f24.low8;
	MOVF  arg2f24,W
	SUBWF arg1f24,W
			;    genSubW( arg1f24.mid8, arg2f24.mid8);
	MOVF  arg2f24+1,W
	BTFSS 0x03,Carry
	INCFSZ arg2f24+1,W
	SUBWF arg1f24+1,W
			;    genSubW( arg1f24.high8, arg2f24.high8);
	MOVF  arg2f24+2,W
	BTFSS 0x03,Carry
	INCFSZ arg2f24+2,W
	SUBWF arg1f24+2,W
			;  _RETURN_MF:
			;    if (Carry)
m052	BTFSS 0x03,Carry
	GOTO  m053
			;        return 0;
	BCF   0x03,Carry
	RETURN
			;    return 1;
m053	BSF   0x03,Carry
	RETURN
			;}
			;
			;
			;
			;bit operator<= _f24_LE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
			;{
_f24_LE_f24
			;    Carry = 1;
			;    if (!(arg1f24.high8 | arg2f24.high8))
			;        return Carry;
			;    if (!arg1f24.15)  {
			;        Carry = 0;
			;        if (arg2f24.15)
			;            return Carry;
			;        W = arg2f24.low8 - arg1f24.low8;
			;        genSubW( arg2f24.mid8, arg1f24.mid8);
			;        genSubW( arg2f24.high8, arg1f24.high8);
			;        return Carry;
			;    }
			;    if (!arg2f24.15)
			;        return Carry;
			;    W = arg1f24.low8 - arg2f24.low8;
			;    genSubW( arg1f24.mid8, arg2f24.mid8);
			;    genSubW( arg1f24.high8, arg2f24.high8);
			;    return Carry;

  ; FILE RPM.c
			;
			;#include "Math24f.h"
			;
			;void main(void) {
main
			;
			;	InitialiseHardware();
	BSF   0x03,RP0
	BCF   0x03,RP1
	CALL  InitialiseHardware
			;	InitialiseGlobals();
	BCF   0x03,RP0
	CALL  InitialiseGlobals
			;
			;	while (1) {
			;		GetA2D();
m054	CALL  GetA2D
			;	}
	GOTO  m054
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
			;	OPTION = 0b10001000;   				// No weak pull ups, prescaler assigned to WDT											(checked ok)
	MOVLW .136
	BSF   0x03,RP0
	MOVWF OPTION_REG
			;   	INTCON = 0b11000000;				// TMR2 used to provide 1ms ticks.														(checked ok)
	MOVLW .192
	MOVWF INTCON
			;	CMCON  = 0b00000111;				// Enable RA0:3 as Digital Inputs														(checked ok)
	MOVLW .7
	MOVWF CMCON
			;	PCON   = 0b00000000;				// 													(checked ok)
	CLRF  PCON
			;
			;	OSCCON = 0b01101100;				// 4MHz internal oscillator
	MOVLW .108
	MOVWF OSCCON
			;
			;	T2CON  = 0b00010101;				// TMR2 on, prescale = 1:4, and postscale = 1:2  (8us ticks with 4MHz oscillator) 		(check this for 8MHz)
	MOVLW .21
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
			;	MC14543_LD_CTRL = 0;				// Disable loads to BCD driver
	BCF   0x03,RP0
	BCF   PORTB,5
			;	MC14543_BI_CTRL = 0;				// 1 Blanks display
	BCF   PORTB,4
			;
			;	ANSEL 	= 0x01;						// Select AN0
	MOVLW .1
	BSF   0x03,RP0
	MOVWF ANSEL
			;	ADCON0	= 0x81;						// Tosc / 32, A/D on	
	MOVLW .129
	BCF   0x03,RP0
	MOVWF ADCON0
			;	ADCON1 	= 0x80;						// Set MSB of ADRESH to 0, VrefH = Vdd, VrefL = Vss;
	MOVLW .128
	BSF   0x03,RP0
	MOVWF ADCON1
			;}
	RETURN
			;
			;void InitialiseGlobals(void) {
InitialiseGlobals
			;	char i =0;
	CLRF  i
			;	DigitCounter =0;
	CLRF  DigitCounter
			;	DispValue = 0;
	CLRF  DispValue
	CLRF  DispValue+1
			;	MagicNumber =0;
	CLRF  MagicNumber
	CLRF  MagicNumber+1
	CLRF  MagicNumber+2
			;	FilterIndex=0;
	CLRF  FilterIndex
			;	for (i=0;i<4;i++) {
	CLRF  i
m055	MOVLW .4
	SUBWF i,W
	BTFSC 0x03,Carry
	GOTO  m056
			;		Digit[i]=0;
	MOVLW .104
	ADDWF i,W
	MOVWF FSR
	BCF   0x03,IRP
	CLRF  INDF
			;	}
	INCF  i,1
	GOTO  m055
			;	for(i=0;i<FilterDepth-1;i++){
m056	CLRF  i
m057	MOVLW .19
	SUBWF i,W
	BTFSC 0x03,Carry
	GOTO  m058
			;		A2DFilter[i]=0;	
	BCF   0x03,Carry
	RLF   i,W
	ADDLW .64
	MOVWF FSR
	BCF   0x03,IRP
	CLRF  INDF
	INCF  FSR,1
	CLRF  INDF
			;	}
	INCF  i,1
	GOTO  m057
			;}
m058	RETURN
			;
			;void DisplayDigit(char DigitCounter){
DisplayDigit
	MOVWF DigitCounter_2
			;
			;	DISP0=0;
	BCF   PORTA,6
			;	DISP1=0;
	BCF   PORTA,4
			;	DISP2=0;
	BCF   PORTA,3
			;	DISP3=0;
	BCF   PORTA,2
			;
			;	DECIMAL=0;
	BCF   PORTA,7
			;
			;//	Put current digit on PORTB lower nibble
			;	PORTB=PORTB&0b11110000;					// CLEAR PORTB LOWER NIBBLE
	MOVLW .240
	ANDWF PORTB,1
			;// Turn on only the digit of interest
			;		switch (DigitCounter) {
	MOVF  DigitCounter_2,W
	BTFSC 0x03,Zero_
	GOTO  m059
	XORLW .1
	BTFSC 0x03,Zero_
	GOTO  m060
	XORLW .3
	BTFSC 0x03,Zero_
	GOTO  m061
	XORLW .1
	BTFSC 0x03,Zero_
	GOTO  m062
	GOTO  m063
			;  			case 0:
			;				PORTB=PORTB|Digit[0];
m059	MOVF  Digit,W
	IORWF PORTB,1
			;				//DECIMAL=1;								
			;				DISP0=1;
	BSF   PORTA,6
			;	   			break;
	GOTO  m063
			;  			case 1:
			;				PORTB=PORTB|Digit[1];
m060	MOVF  Digit+1,W
	IORWF PORTB,1
			;				//DECIMAL=1;
			;				DISP1=1;										
	BSF   PORTA,4
			;	   			break;
	GOTO  m063
			;  			case 2:
			;				PORTB=PORTB|Digit[2];	
m061	MOVF  Digit+2,W
	IORWF PORTB,1
			;				//DECIMAL=1;									
			;	   			DISP2=1;
	BSF   PORTA,3
			;				break;
	GOTO  m063
			;  			case 3:
			;				PORTB=PORTB|Digit[3];
m062	MOVF  Digit+3,W
	IORWF PORTB,1
			;				//DECIMAL=1;
			;				DISP3=1;										
	BSF   PORTA,2
			;	   			break;
			;			default:
			;				break;
			;		}
			;	MC14543_LD_CTRL=1;						// Load digit into BCD driver.
m063	BSF   PORTB,5
			;    MC14543_LD_CTRL=0;						// Disable loads to BCD driver
	BCF   PORTB,5
			;}	
	RETURN
			;
			;void Dec2BCD() {
Dec2BCD
			;	uns16 temp =0;
	CLRF  temp_2
	CLRF  temp_2+1
			;	//THIS CONVERTS 16BIT NO to 4 BCD bytes
			;		temp=DispValue%10;
	MOVF  DispValue,W
	MOVWF C2tmp
	MOVF  DispValue+1,W
	MOVWF C2tmp+1
	CLRF  temp_2
	CLRF  temp_2+1
	MOVLW .16
	MOVWF C1cnt
m064	RLF   C2tmp,1
	RLF   C2tmp+1,1
	RLF   temp_2,1
	BTFSC 0x03,Carry
	GOTO  m065
	MOVLW .10
	SUBWF temp_2,W
	BTFSS 0x03,Carry
	GOTO  m066
m065	MOVLW .10
	SUBWF temp_2,1
m066	DECFSZ C1cnt,1
	GOTO  m064
			;		Digit[0]=temp;			
	MOVF  temp_2,W
	MOVWF Digit
			;		temp=DispValue/10;		
	MOVF  DispValue,W
	MOVWF C4tmp
	MOVF  DispValue+1,W
	MOVWF C4tmp+1
	CLRF  C5rem
	MOVLW .16
	MOVWF C3cnt
m067	RLF   C4tmp,1
	RLF   C4tmp+1,1
	RLF   C5rem,1
	BTFSC 0x03,Carry
	GOTO  m068
	MOVLW .10
	SUBWF C5rem,W
	BTFSS 0x03,Carry
	GOTO  m069
m068	MOVLW .10
	SUBWF C5rem,1
	BSF   0x03,Carry
m069	RLF   temp_2,1
	RLF   temp_2+1,1
	DECFSZ C3cnt,1
	GOTO  m067
			;		Digit[1]=temp%10;			
	MOVF  temp_2,W
	MOVWF C7tmp
	MOVF  temp_2+1,W
	MOVWF C7tmp+1
	CLRF  Digit+1
	MOVLW .16
	MOVWF C6cnt
m070	RLF   C7tmp,1
	RLF   C7tmp+1,1
	RLF   Digit+1,1
	BTFSC 0x03,Carry
	GOTO  m071
	MOVLW .10
	SUBWF Digit+1,W
	BTFSS 0x03,Carry
	GOTO  m072
m071	MOVLW .10
	SUBWF Digit+1,1
m072	DECFSZ C6cnt,1
	GOTO  m070
			;		temp=DispValue/100;	
	MOVF  DispValue,W
	MOVWF C9tmp
	MOVF  DispValue+1,W
	MOVWF C9tmp+1
	CLRF  C10rem
	MOVLW .16
	MOVWF C8cnt
m073	RLF   C9tmp,1
	RLF   C9tmp+1,1
	RLF   C10rem,1
	BTFSC 0x03,Carry
	GOTO  m074
	MOVLW .100
	SUBWF C10rem,W
	BTFSS 0x03,Carry
	GOTO  m075
m074	MOVLW .100
	SUBWF C10rem,1
	BSF   0x03,Carry
m075	RLF   temp_2,1
	RLF   temp_2+1,1
	DECFSZ C8cnt,1
	GOTO  m073
			;		Digit[2]=temp%10;			
	MOVF  temp_2,W
	MOVWF C12tmp
	MOVF  temp_2+1,W
	MOVWF C12tmp+1
	CLRF  Digit+2
	MOVLW .16
	MOVWF C11cnt
m076	RLF   C12tmp,1
	RLF   C12tmp+1,1
	RLF   Digit+2,1
	BTFSC 0x03,Carry
	GOTO  m077
	MOVLW .10
	SUBWF Digit+2,W
	BTFSS 0x03,Carry
	GOTO  m078
m077	MOVLW .10
	SUBWF Digit+2,1
m078	DECFSZ C11cnt,1
	GOTO  m076
			;		temp=DispValue/1000;	
	MOVF  DispValue,W
	MOVWF C14tmp
	MOVF  DispValue+1,W
	MOVWF C14tmp+1
	CLRF  C15rem
	CLRF  C15rem+1
	MOVLW .16
	MOVWF C13cnt
m079	RLF   C14tmp,1
	RLF   C14tmp+1,1
	RLF   C15rem,1
	RLF   C15rem+1,1
	MOVLW .3
	SUBWF C15rem+1,W
	BTFSS 0x03,Carry
	GOTO  m081
	BTFSS 0x03,Zero_
	GOTO  m080
	MOVLW .232
	SUBWF C15rem,W
	BTFSS 0x03,Carry
	GOTO  m081
m080	MOVLW .3
	SUBWF C15rem+1,1
	MOVLW .232
	SUBWF C15rem,1
	BTFSS 0x03,Carry
	DECF  C15rem+1,1
	BSF   0x03,Carry
m081	RLF   temp_2,1
	RLF   temp_2+1,1
	DECFSZ C13cnt,1
	GOTO  m079
			;		Digit[3]=temp;
	MOVF  temp_2,W
	MOVWF Digit+3
			;}
	RETURN
			;
			;
			;void GetA2D() {
GetA2D
			;	uns16 Filter =0;
	CLRF  Filter
	CLRF  Filter+1
			;	char i =0;
	CLRF  i_2
			;	uns16 filtertemp =0;
	CLRF  filtertemp
	CLRF  filtertemp+1
			;	uns16 comparison =0;
	CLRF  comparison
	CLRF  comparison+1
			;
			;	A2DResult =0;
	CLRF  A2DResult
	CLRF  A2DResult+1
			;	
			;	ADCON0=ADCON0|0x04;							//Start A2D Conversion
	BSF   ADCON0,2
			;	while((ADCON0&0x04) != 0){};						// Wait for conversion
m082	BTFSC ADCON0,2
	GOTO  m082
			;	A2DResult=ADRESH;
	MOVF  ADRESH,W
	MOVWF A2DResult
	CLRF  A2DResult+1
			;	A2DResult=A2DResult<<8;
	MOVWF A2DResult+1
	CLRF  A2DResult
			;	A2DResult=A2DResult|ADRESL;
	BSF   0x03,RP0
	MOVF  ADRESL,W
	BCF   0x03,RP0
	IORWF A2DResult,1
			;	A2DResult=A2DResult&0b0000.1111.1111.1111;
	MOVLW .15
	ANDWF A2DResult+1,1
			;	
			;	//if(A2DResult<3){
			;	//	A2DResult=0;
			;	//}
			;	
			;	// NOTE ON FILTERING - A2D range is 0 - 1023 (12 Bits)
			;	// FILTER variable is 16 bit
			;	// max no for Fitler depth = 64
			;	
			;	A2DFilter[FilterIndex]=A2DResult;
	BCF   0x03,Carry
	RLF   FilterIndex,W
	ADDLW .64
	MOVWF FSR
	BCF   0x03,IRP
	MOVF  A2DResult,W
	MOVWF INDF
	INCF  FSR,1
	MOVF  A2DResult+1,W
	MOVWF INDF
			;	//A2DFilter[FilterIndex]=FilterDepth;
			;	FilterIndex++;
	INCF  FilterIndex,1
			;	if(FilterIndex>FilterDepth-1){
	MOVLW .20
	SUBWF FilterIndex,W
	BTFSC 0x03,Carry
			;		FilterIndex=0;
	CLRF  FilterIndex
			;	}
			;
			;	for(i=0;i<FilterDepth-1;i++) {
	CLRF  i_2
m083	MOVLW .19
	SUBWF i_2,W
	BTFSC 0x03,Carry
	GOTO  m084
			;		filtertemp = A2DFilter[i];
	BCF   0x03,Carry
	RLF   i_2,W
	ADDLW .64
	MOVWF FSR
	BCF   0x03,IRP
	MOVF  INDF,W
	MOVWF filtertemp
	INCF  FSR,1
	MOVF  INDF,W
	MOVWF filtertemp+1
			;		//filtertemp=25;
			;		Filter= Filter+filtertemp;
	ADDWF Filter+1,1
	MOVF  filtertemp,W
	ADDWF Filter,1
	BTFSC 0x03,Carry
	INCF  Filter+1,1
			;	}	
	INCF  i_2,1
	GOTO  m083
			;	Filter=Filter+filtertemp;
m084	MOVF  filtertemp+1,W
	ADDWF Filter+1,1
	MOVF  filtertemp,W
	ADDWF Filter,1
	BTFSC 0x03,Carry
	INCF  Filter+1,1
			;	Filter=Filter/FilterDepth;
	MOVF  Filter,W
	MOVWF C17tmp
	MOVF  Filter+1,W
	MOVWF C17tmp+1
	CLRF  C18rem
	MOVLW .16
	MOVWF C16cnt
m085	RLF   C17tmp,1
	RLF   C17tmp+1,1
	RLF   C18rem,1
	BTFSC 0x03,Carry
	GOTO  m086
	MOVLW .20
	SUBWF C18rem,W
	BTFSS 0x03,Carry
	GOTO  m087
m086	MOVLW .20
	SUBWF C18rem,1
	BSF   0x03,Carry
m087	RLF   Filter,1
	RLF   Filter+1,1
	DECFSZ C16cnt,1
	GOTO  m085
			;	
			;//DispValue = Filter;
			;
			;MagicNumber = Filter;
	MOVF  Filter,W
	MOVWF arg1f24
	MOVF  Filter+1,W
	MOVWF arg1f24+1
	CLRF  arg1f24+2
	CALL  _int24ToFloat24
	MOVWF MagicNumber
	MOVF  rval+1,W
	MOVWF MagicNumber+1
	MOVF  rval+2,W
	MOVWF MagicNumber+2
			;MagicNumber = MagicNumber * ScaleRange;
	MOVF  MagicNumber,W
	MOVWF arg1f24
	MOVF  MagicNumber+1,W
	MOVWF arg1f24+1
	MOVF  MagicNumber+2,W
	MOVWF arg1f24+2
	MOVLW .64
	MOVWF arg2f24
	MOVLW .103
	MOVWF arg2f24+1
	MOVLW .137
	MOVWF arg2f24+2
	CALL  _fmul24
	MOVWF MagicNumber
	MOVF  arg1f24+1,W
	MOVWF MagicNumber+1
	MOVF  arg1f24+2,W
	MOVWF MagicNumber+2
			;MagicNumber = MagicNumber / 1024;
	MOVLW .10
	SUBWF MagicNumber+2,1
	BTFSC 0x03,Carry
	GOTO  m088
	BSF   0x74,FpUnderFlow
	CLRF  MagicNumber+2
			;
			;comparison = MagicNumber-10;
m088	MOVF  MagicNumber,W
	MOVWF arg1f24
	MOVF  MagicNumber+1,W
	MOVWF arg1f24+1
	MOVF  MagicNumber+2,W
	MOVWF arg1f24+2
	CLRF  arg2f24
	MOVLW .32
	MOVWF arg2f24+1
	MOVLW .130
	MOVWF arg2f24+2
	CALL  _fsub24
	CALL  _float24ToInt24
	MOVWF comparison
	MOVF  rval_3+1,W
	MOVWF comparison+1
			;
			;if(OldMagicNumber < comparison){
	MOVF  comparison,W
	MOVWF arg1f24
	MOVF  comparison+1,W
	MOVWF arg1f24+1
	CLRF  arg1f24+2
	CALL  _int24ToFloat24
	MOVF  OldMagicNumber,W
	MOVWF arg2f24
	MOVF  OldMagicNumber+1,W
	MOVWF arg2f24+1
	MOVF  OldMagicNumber+2,W
	MOVWF arg2f24+2
	CALL  _f24_GT_f24
	BTFSS 0x03,Carry
	GOTO  m089
			;	//MagicNumber = Filter;
			;	OldMagicNumber=MagicNumber;
	MOVF  MagicNumber,W
	MOVWF OldMagicNumber
	MOVF  MagicNumber+1,W
	MOVWF OldMagicNumber+1
	MOVF  MagicNumber+2,W
	MOVWF OldMagicNumber+2
			;}
			;
			;comparison =  MagicNumber+10;
m089	MOVF  MagicNumber,W
	MOVWF arg1f24
	MOVF  MagicNumber+1,W
	MOVWF arg1f24+1
	MOVF  MagicNumber+2,W
	MOVWF arg1f24+2
	CLRF  arg2f24
	MOVLW .32
	MOVWF arg2f24+1
	MOVLW .130
	MOVWF arg2f24+2
	CALL  _fadd24
	CALL  _float24ToInt24
	MOVWF comparison
	MOVF  rval_3+1,W
	MOVWF comparison+1
			;if(OldMagicNumber > comparison){
	MOVF  comparison,W
	MOVWF arg1f24
	MOVF  comparison+1,W
	MOVWF arg1f24+1
	CLRF  arg1f24+2
	CALL  _int24ToFloat24
	MOVF  OldMagicNumber,W
	MOVWF arg2f24
	MOVF  OldMagicNumber+1,W
	MOVWF arg2f24+1
	MOVF  OldMagicNumber+2,W
	MOVWF arg2f24+2
	CALL  _f24_LT_f24
	BTFSS 0x03,Carry
	GOTO  m090
			;	OldMagicNumber=MagicNumber;
	MOVF  MagicNumber,W
	MOVWF OldMagicNumber
	MOVF  MagicNumber+1,W
	MOVWF OldMagicNumber+1
	MOVF  MagicNumber+2,W
	MOVWF OldMagicNumber+2
			;} 
			;
			;if(MagicNumber > 10){
m090	BTFSC MagicNumber+1,7
	GOTO  m092
	MOVLW .130
	SUBWF MagicNumber+2,W
	BTFSS 0x03,Carry
	GOTO  m092
	BTFSS 0x03,Zero_
	GOTO  m091
	MOVLW .32
	SUBWF MagicNumber+1,W
	BTFSS 0x03,Carry
	GOTO  m092
	BTFSS 0x03,Zero_
	GOTO  m091
	MOVF  MagicNumber,1
	BTFSC 0x03,Zero_
	GOTO  m092
			;	DispValue= OldMagicNumber;
m091	MOVF  OldMagicNumber,W
	MOVWF arg1f24
	MOVF  OldMagicNumber+1,W
	MOVWF arg1f24+1
	MOVF  OldMagicNumber+2,W
	MOVWF arg1f24+2
	CALL  _float24ToInt24
	MOVWF DispValue
	MOVF  rval_3+1,W
	MOVWF DispValue+1
			;} else {
	GOTO  m093
			;	DispValue = 0;
m092	CLRF  DispValue
	CLRF  DispValue+1
m093	RETURN

	ORG 0x2007
	DATA 3F58H
	DATA 3FFCH
	END


; *** KEY INFO ***

; 0x01E7 P0   44 word(s)  2 % : InitialiseHardware
; 0x0213 P0   36 word(s)  1 % : InitialiseGlobals
; 0x0237 P0   39 word(s)  1 % : DisplayDigit
; 0x025E P0  147 word(s)  7 % : Dec2BCD
; 0x02F1 P0  229 word(s) 11 % : GetA2D
; 0x0004 P0   30 word(s)  1 % : IntHandler
; 0x0022 P0   77 word(s)  3 % : _fmul24
; 0x006F P0  152 word(s)  7 % : _fadd24
; 0x0107 P0    5 word(s)  0 % : _fsub24
; 0x010C P0   62 word(s)  3 % : _int24ToFloat24
; 0x014A P0   74 word(s)  3 % : _float24ToInt24
; 0x0194 P0   38 word(s)  1 % : _f24_LT_f24
; 0x01BA P0   38 word(s)  1 % : _f24_GT_f24
; 0x01E0 P0    7 word(s)  0 % : main

; RAM usage: 85 bytes (27 local), 283 bytes free
; Maximum call level: 3 (+2 for interrupt)
;  Codepage 0 has  979 word(s) :  47 %
;  Codepage 1 has    0 word(s) :   0 %
; Total of 979 code words (23 %)
