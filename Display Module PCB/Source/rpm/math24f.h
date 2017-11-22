// *************************************************
// 24 bit basic floating point math operations
// Copyright (c) B Knudsen Data, Norway, 2000 - 2009
// *************************************************

#pragma library 1
/* PROTOTYPES for page definition in application header file:
float24 operator* _fmul24( float24 arg1f24, float24 arg2f24);
float24 operator/ _fdiv24( float24 arg1f24, float24 arg2f24);
float24 operator+ _fadd24( float24 arg1f24, float24 arg2f24);
float24 operator- _fsub24( float24 arg1f24, float24 arg2f24);
float24 operator= _int24ToFloat24( int24 arg1f24);
float24 operator= _int32ToFloat24( int32 arg32);
int24 operator= _float24ToInt24( float24 arg1f24);
bit operator< _f24_LT_f24( float24 arg1f24, float24 arg2f24);
bit operator>= _f24_GE_f24( float24 arg1f24, float24 arg2f24);
bit operator> _f24_GT_f24( float24 arg1f24, float24 arg2f24);
bit operator<= _f24_LE_f24( float24 arg1f24, float24 arg2f24);
*/

// DEFINABLE SYMBOLS (in the application code):
//#define FP_OPTIM_SPEED  // optimize for SPEED: default
//#define FP_OPTIM_SIZE   // optimize for SIZE
//#define DISABLE_ROUNDING   // disable rounding and save code space

#define float24ToIEEE754(a) { a.mid8=rl(a.mid8); a.high8=rr(a.high8);\
                              a.mid8=rr(a.mid8); }
#define IEEE754ToFloat24(a) { a.mid8=rl(a.mid8); a.high8=rl(a.high8);\
                              a.mid8=rr(a.mid8); }


/*  24 bit floating point format:

  address  ID
    X      a.low8  : LSB, bit 0-7 of mantissa
    X+1    a.mid8  : bit 8-14 of mantissa, bit 15 is the sign bit
    X+2    a.high8 : MSB, bit 0-7 of exponent, with bias 0x7F

    bit 15 of mantissa is a hidden bit, always equal to 1
    zero (0.0) :  a.high8 = 0 (mantissa & sign ignored)

   MSB    LSB
    7F 00 00  : 1.0   =  1.0  * 2**(0x7F-0x7F) = 1.0 * 1
    7F 80 00  : -1.0  = -1.0  * 2**(0x7F-0x7F) = -1.0 * 1
    80 00 00  : 2.0   =  1.0  * 2**(0x80-0x7F) = 1.0 * 2
    80 40 00  : 3.0   =  1.5  * 2**(0x80-0x7F) = 1.5 * 2
    7E 60 00  : 0.875 =  1.75 * 2**(0x7E-0x7F) = 1.75 * 0.5
    7F 60 00  : 1.75  =  1.75 * 2**(0x7E-0x7F) = 1.75 * 1
    7F 7F FF  : 1.999969482
    00 7C 5A  : 0.0 (mantissa & sign ignored)
    01 00 00  : 1.17549435e-38 =  1.0 * 2**(0x01-0x7F)
    FE 7F FF  : 3.40277175e+38 =  1.999969482 * 2**(0xFE-0x7F)
    FF 00 00  : +INF : positive infinity
    FF 80 00  : -INF : negative infinity
*/                 

#define  FpBIAS  0x7F

#ifndef FpFlags_defined
 #define FpFlags_defined

 char FpFlags;
 //bit IOV         @ FpFlags.0; // integer overflow flag: NOT USED
 bit FpOverflow    @ FpFlags.1; // floating point overflow flag
 bit FpUnderFlow   @ FpFlags.2; // floating point underflow flag
 bit FpDiv0        @ FpFlags.3; // floating point divide by zero flag
 //bit FpNAN       @ FpFlags.4; // not-a-number exception flag: NOT USED
 bit FpDomainError @ FpFlags.5; // domain error exception flag
 bit FpRounding    @ FpFlags.6; // floating point rounding flag, 0=truncation
                                // 1 = unbiased rounding to nearest LSB
 //bit FpSaturate  @ FpFlags.7; // floating point saturate flag: NOT USED

 #pragma floatOverflow FpOverflow
 #pragma floatUnderflow FpUnderFlow

 #define InitFpFlags()  FpFlags = 0x40 /* enable rounding as default */
#endif

#ifdef DISABLE_ROUNDING
 #pragma floatRounding 0
#endif


#if __CoreSet__ < 1410
 #define genAdd(r,a) W=a; btsc(Carry); W=incsz(a); r+=W
 #define genSub(r,a) W=a; btss(Carry); W=incsz(a); r-=W
 #define genAddW(r,a) W=a; btsc(Carry); W=incsz(a); W=r+W
 #define genSubW(r,a) W=a; btss(Carry); W=incsz(a); W=r-W
#else
 #define genAdd(r,a) W=a; r=addWFC(r)
 #define genSub(r,a) W=a; r=subWFB(r)
 #define genAddW(r,a) W=a; W=addWFC(r)
 #define genSubW(r,a) W=a; W=subWFB(r)
#endif



float24 operator* _fmul24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    uns16 aarg;
    W = arg1f24.mid8;
    aarg.high8 = W;

    // save sign
    char sign = arg2f24.mid8 ^ W;  // before first overflow test

    W = arg1f24.high8;
    if (!Zero_)
        W = arg2f24.high8;
    if (Zero_)
        goto RES0;

    arg1f24.high8 += W /* arg2f24.high8 */;
    W = FpBIAS-1;
    if (Carry)  {
        arg1f24.high8 -= W;
        if (Carry)
            goto OVERFLOW;
    }
    else  {
        arg1f24.high8 -= W;
        if (!Carry)
            goto UNDERFLOW;
    }
    aarg.low8 = arg1f24.low8;

    aarg.15 = 1;
    arg2f24.15 = 1;

    arg1f24.low16 = 0;

    char counter = sizeof(aarg)*8;

    do  {
        aarg = rr( aarg);
        if (Carry)  {
            arg1f24.low8 += arg2f24.low8;
            genAdd( arg1f24.mid8, arg2f24.mid8);
        }
        arg1f24.low16 = rr( arg1f24.low16);
    } while (-- counter > 0);

    if (!arg1f24.15)  {
        // catch Carry bit that was shifted out previously
        arg1f24.low16 = rl( arg1f24.low16);
        if (arg1f24.high8 == 0)
            goto UNDERFLOW;
        arg1f24.high8 -= 1;
        W = rl( aarg.high8);
        // restore bit behind LSB in Carry
    }

   #ifndef DISABLE_ROUNDING
    if (FpRounding  &&  Carry)  {
        arg1f24.low8 += 1;
        if (!arg1f24.low8)  {
            arg1f24.mid8 += 1;
            if (!arg1f24.mid8)  {
                // Carry = 1; //OK
                arg1f24.low16 = rr( arg1f24.low16);
                arg1f24.high8 += 1;
                if (Zero_)
                    goto OVERFLOW;
            }
        }
    }
   #endif
    goto SET_SIGN;

  UNDERFLOW:
    FpUnderFlow = 1;
  RES0:
    arg1f24.high8 = 0;
    goto MANTISSA;

  OVERFLOW:
    FpOverflow = 1;
    arg1f24.high8 = 0xFF;
  MANTISSA:
    arg1f24.low16 = 0x8000;

  SET_SIGN:
    if (!(sign & 0x80))
        arg1f24.15 = 0;
    return arg1f24;
}



float24 operator/ _fdiv24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    uns16 aarg;
    W = arg1f24.mid8;
    aarg.high8 = W;

    // save sign
    char sign = arg2f24.mid8 ^ W;  // before first overflow test

    W = arg2f24.high8;
    if (Zero_)
        goto Div0;
    if (!arg1f24.high8)
        goto RES0;

    arg1f24.high8 -= arg2f24.high8;
    W = FpBIAS;
    if (!Carry)  {
        arg1f24.high8 += W;
        if (!Carry)
            goto UNDERFLOW;
    }
    else  {
        arg1f24.high8 += W;
        if (Carry)
            goto OVERFLOW;
    }

    aarg.low8 = arg1f24.low8;
    aarg.15 = 1;
    arg2f24.15 = 1;

    // division: shift & add
    char counter = 16;
    arg1f24.low16 = 0;  // speedup

#if defined FP_OPTIM_SPEED || !defined FP_OPTIM_SIZE  // SPEED

    goto START_ML;

  TEST_ZERO_L:
    W = aarg.low8 - arg2f24.low8;
    if (!Carry)
        goto SHIFT_IN_CARRY;
    aarg.low8 = W;
    aarg.high8 = 0;
    goto SET_AND_SHIFT_IN_CARRY;

// MAIN LOOP
    do  {
      LOOP_ML:
        if (!Carry)  {
           START_ML:
            W = aarg.high8 - arg2f24.mid8;
            if (Zero_)
                goto TEST_ZERO_L;
            if (!Carry)
                goto SHIFT_IN_CARRY;
        }
        aarg.low8 -= arg2f24.low8;
        genSub( aarg.high8, arg2f24.mid8);
      SET_AND_SHIFT_IN_CARRY:
        Carry = 1;
      SHIFT_IN_CARRY:
        arg1f24.low16 = rl( arg1f24.low16);
        // Carry = 0;  // ok, speedup
        aarg = rl( aarg);
    } while (-- counter > 0);



#else  // SIZE

    goto START_ML;

// MAIN LOOP
    do  {
      LOOP_ML:
        if (Carry)
            goto SUBTRACT;
      START_ML:
        W = aarg.low8 - arg2f24.low8;
        genSubW( aarg.high8, arg2f24.mid8);
        if (!Carry)
            goto SKIP_SUB;
       SUBTRACT:
        aarg.low8 -= arg2f24.low8;
        genSub( aarg.high8, arg2f24.mid8);
        Carry = 1;
       SKIP_SUB:
        arg1f24.low16 = rl( arg1f24.low16);
        // Carry = 0;  // ok
        aarg = rl( aarg);
    } while (-- counter > 0);

#endif

    if (!arg1f24.15)  {
        if (!arg1f24.high8)
            goto UNDERFLOW;
        arg1f24.high8 --;
        counter ++;
        goto LOOP_ML;
    }

   #ifndef DISABLE_ROUNDING
    if (FpRounding)  {
        if (Carry)
            goto ADD_1;
        aarg.low8 -= arg2f24.low8;
        genSub( aarg.high8, arg2f24.mid8);
        if (Carry)  {
          ADD_1:
            arg1f24.low8 += 1;
            if (!arg1f24.low8)  {
                arg1f24.mid8 ++;
                if (!arg1f24.mid8)  {
                    arg1f24.low16 = rr( arg1f24.low16);
                    arg1f24.high8 ++;
                    if (!arg1f24.high8)
                        goto OVERFLOW;
                }
            }
        }
    }
   #endif
    goto SET_SIGN;

  Div0:
    FpDiv0 = 1;
    goto SATURATE;

  UNDERFLOW:
    FpUnderFlow = 1;
  RES0:
    arg1f24.high8 = 0;
    goto MANTISSA;

  OVERFLOW:
    FpOverflow = 1;
  SATURATE:
    arg1f24.high8 = 0xFF;
  MANTISSA:
    arg1f24.low16 = 0x8000;

  SET_SIGN:
    if (!(sign & 0x80))
        arg1f24.15 = 0;
    return arg1f24;
}


float24 operator+ _fadd24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    char xtra, temp;
    char expo = arg1f24.high8 - arg2f24.high8;
    if (!Carry)  {
        expo = -expo;
        temp = arg1f24.high8;
        arg1f24.high8 = arg2f24.high8;
        arg2f24.high8 = temp;
        temp = arg1f24.mid8;
        arg1f24.mid8 = arg2f24.mid8;
        arg2f24.mid8 = temp;
        temp = arg1f24.low8;
        arg1f24.low8 = arg2f24.low8;
        arg2f24.low8 = temp;
    }
    if (expo > sizeof(arg1f24)*8-7)
        goto _RETURN_MF;
    if (!arg2f24.high8)
        goto _RETURN_MF;   // result is arg1f24

    xtra = 0;

    temp = arg1f24.mid8;
    char sign = arg2f24.mid8 ^ arg1f24.mid8;
    arg1f24.15 = 1;
    arg2f24.15 = 1;

    while (1)  {
        W = 8;
        expo -= W;
        if (!Carry)
            break;
        xtra = arg2f24.low8;
        arg2f24.low8 = arg2f24.mid8;
        arg2f24.mid8 = 0;
    }
    expo += W;
    if (expo)  {
        do  {
            Carry = 0;
            arg2f24.low16 = rr( arg2f24.low16);
            xtra = rr( xtra);
        } while (--expo > 0);
    }


    if (sign & 0x80)  {
        // SUBTRACT
        arg1f24.low8 -= arg2f24.low8;
        genSub( arg1f24.mid8, arg2f24.mid8);
        if (!Carry)  {  // arg2f24 > arg1f24
            arg1f24.low16 = -arg1f24.low16;
            // xtra == 0 because arg1f24.exp == arg2f24.exp
            temp ^= 0x80;  // invert sign
        }
        xtra = -xtra;
        if (xtra)
            arg1f24.low16 --;
        // adjust result left
       #define counter expo
        counter = 3;
        while (!arg1f24.mid8)  {
            arg1f24.mid8 = arg1f24.low8;
            arg1f24.low8 = xtra;
            xtra = 0;
            arg1f24.high8 -= 8;
            if (!Carry)
                goto RES0;
            if (--counter == 0)  // max 2 iterations
                goto RES0;
        }
       #undef counter
        while (!arg1f24.15)  {
            Carry = 0;
            xtra = rl( xtra);
            arg1f24.low16 = rl( arg1f24.low16);
            arg1f24.high8 --;
            if (!arg1f24.high8)
                goto RES0;   // UNDERFLOW?
        }
       #ifndef DISABLE_ROUNDING
        if (FpRounding  &&  (xtra & 0x80))  {
            xtra = 0; // disable recursion
            goto INCREMENT;
        }
       #endif
    }
    else  {
        // ADD arg1f24 and arg2f24
        arg1f24.low8 += arg2f24.low8;
        genAdd( arg1f24.mid8, arg2f24.mid8);
        if (Carry)  {
          ADJUST_RIGHT:
            arg1f24.low16 = rr( arg1f24.low16);
            xtra = rr( xtra);
            arg1f24.high8 += 1;  // exp
            if (!arg1f24.high8)
                goto OVERFLOW;
        }
       #ifndef DISABLE_ROUNDING
        if (FpRounding  &&  (xtra & 0x80))  {
          INCREMENT:
            arg1f24.low8 += 1;
            if (!arg1f24.low8)  {
                arg1f24.mid8 += 1;
                if (!arg1f24.mid8)  {
                    Carry = 1; // prepare for shift
                    arg1f24.0 = 0;  // disable recursion
                    goto ADJUST_RIGHT;
                }
            }
        }
       #endif
    }
    goto SET_SIGN;

//  UNDERFLOW:
//    FpUnderFlow = 1;
  RES0:
    arg1f24.high8 = 0;
    goto MANTISSA;

  OVERFLOW:
    FpOverflow = 1;
    arg1f24.high8 = 0xFF;
  MANTISSA:
    arg1f24.low16 = 0x8000;

  SET_SIGN:
    if (!(temp & 0x80))
        arg1f24.15 = 0;

  _RETURN_MF:
    return arg1f24;
}


// SUBTRACTION

float24 operator- _fsub24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    arg2f24.mid8 ^= 0x80;
    arg1f24 += arg2f24;
    return arg1f24;
}


float24 operator=( int8 arg) @
float24 operator=( uns8 arg) @
float24 operator=( int16 arg) @
float24 operator=( uns16 arg) @
float24 operator= _int24ToFloat24( sharedM int24 arg1f24)
{
    sharedM float24 arg2f24;   // unused, but required
    char expo = FpBIAS + 16 - 1;
    char xtra = 0;
    char sign = 0;
    if (arg1f24 < 0)  {
        arg1f24 = -arg1f24;
        sign |= 0x80;
    }
    if (arg1f24.high8)  {
        expo += 8;
        xtra = arg1f24.low8;
        arg1f24.low8 = arg1f24.mid8;
        arg1f24.mid8 = arg1f24.high8;
    }
    else if (!arg1f24.mid8)  {
        expo -= 8;
        W = arg1f24.low8;
        if (!W)
            goto _RETURN_MF;
        arg1f24.mid8 = W;
        arg1f24.low8 = 0;
    }

    // arg1f24.mid8 != 0
    goto TEST_ARG1_B15;
    do  {
        xtra = rl( xtra);
        arg1f24.low16 = rl( arg1f24.low16);
        expo --;
      TEST_ARG1_B15:
    } while (!arg1f24.15);

   #ifndef DISABLE_ROUNDING
    if (FpRounding && (xtra & 0x80))  {
        arg1f24.low8 += 1;
        if (!arg1f24.low8)  {
            arg1f24.mid8 += 1;
            if (!arg1f24.mid8)  {
                Carry = 1;
                arg1f24.low16 = rr( arg1f24.low16);
                expo ++;
            }
        }
    }
   #endif

    arg1f24.high8 = expo;
    if (!(sign & 0x80))
        arg1f24.15 = 0;

  _RETURN_MF:
    float24 rval @ arg1f24;
    rval.low24 = arg1f24.low24;
    return rval;
}


float24 operator=( uns24 arg) @
float24 operator= _int32ToFloat24( int32 arg32)
{
    char expo = FpBIAS + 16 - 1;
    char xtra @ arg32.high8;
    char sign = 0;
    if (arg32 < 0)  {
        arg32 = -arg32;
        sign |= 0x80;
    }
    if (arg32.high8)  {
        expo += 8;
        arg32.low8 = arg32.midL8;
        arg32.midL8 = arg32.midH8;
        arg32.midH8 = arg32.high8;
        arg32.high8 = 0;
    }
    if (arg32.midH8)  {
        expo += 8;
        xtra = arg32.low8;
        arg32.low8 = arg32.midL8;
        arg32.midL8 = arg32.midH8;
    }
    else if (!arg32.midL8)  {
        expo -= 8;
        W = arg32.low8;
        if (!W)
            goto _RETURN_MF;
        arg32.midL8 = W;
        arg32.low8 = 0;
    }

    // arg32.midL8 != 0
    goto TEST_ARG_B15;
    do  {
        xtra = rl( xtra);
        arg32.low16 = rl( arg32.low16);
        expo --;
      TEST_ARG_B15:
    } while (!arg32.15);

   #ifndef DISABLE_ROUNDING
    if (FpRounding && (xtra & 0x80))  {
        arg32.low8 += 1;
        if (!arg32.low8)  {
            arg32.midL8 += 1;
            if (!arg32.midL8)  {
                Carry = 1;
                arg32.low16 = rr( arg32.low16);
                expo ++;
            }
        }
    }
   #endif

    arg32.midH8 = expo;
    if (!(sign & 0x80))
        arg32.15 = 0;

  _RETURN_MF:
    float24 rval @ arg32;
    rval.low24 = arg32.low24;
    return rval;
}


uns8 operator=( sharedM float24 arg1f24) @
int8 operator=( sharedM float24 arg1f24) @
uns16 operator=( sharedM float24 arg1f24) @
int16 operator=( sharedM float24 arg1f24) @
int24 operator= _float24ToInt24( sharedM float24 arg1f24)
{
    sharedM float24 arg2f24;   // unused, but required
    char sign = arg1f24.mid8;
    char expo = arg1f24.high8 - (FpBIAS-1);
    if (!Carry)
        goto RES0;
    arg1f24.15 = 1;

    arg1f24.high8 = 0;
   #ifndef DISABLE_ROUNDING
    char xtra = 0;
   #endif

    // (a): expo = 0..8 : shift 1 byte to the right
    // (b): expo = 9..16: shift 0 byte
    // (c): expo = 17..24: shift 1 byte to the left
   #if __CoreSet__ / 100 == 12
    expo -= 17;
    expo = 0xFF - expo;  // COMF (Carry unchanged)
    if (Carry)  {  // (c)
   #else
    expo = 16 - expo;
    if (!Carry)  {  // (c)
   #endif
        expo += 8;
        if (!Carry)
            goto OVERFLOW;
        arg1f24.high8 = arg1f24.mid8;
        arg1f24.mid8 = arg1f24.low8;
        arg1f24.low8 = 0;
    }
    else  {  // (a) (b)
        // expo = 0 .. 16
        W = expo - 8;
        if (Carry)  {  // (a)
            expo = W;
           #ifndef DISABLE_ROUNDING
            xtra = arg1f24.low8;
           #endif
            arg1f24.low8 = arg1f24.mid8;
            arg1f24.mid8 = 0;
        }
    }
    if (expo)  {
        do  {
            Carry = 0;
            arg1f24.high8 = rr( arg1f24.high8);
            arg1f24.low16 = rr( arg1f24.low16);
           #ifndef DISABLE_ROUNDING
            xtra = rr( xtra);
           #endif
        } while (--expo);
    }
    if (arg1f24.23)  {
       OVERFLOW:
        FpOverflow = 1;
        W = 0xFF;
        goto ASSIGNW;
       RES0:
        W = 0;
       ASSIGNW:
        arg1f24.low8 = W;
        arg1f24.mid8 = W;
        arg1f24.high8 = W;
        arg1f24.23 = 0;
    }
    else  {
       #ifndef DISABLE_ROUNDING
        if (FpRounding && (xtra & 0x80))  {
            arg1f24.low8 += 1;
            if (!arg1f24.low8)
                arg1f24.mid8 += 1;
        }
       #endif
        if (sign & 0x80)
            arg1f24.low24 = -arg1f24.low24;
    }
    int24 rval @ arg1f24;
    rval = arg1f24.low24;
    return rval;
}


bit operator< _f24_LT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    Carry = 0;
    if (!(arg1f24.high8 | arg2f24.high8))
        return Carry;
    if (!arg1f24.15)  {
        if (arg2f24.15)
            return Carry;
        W = arg1f24.low8 - arg2f24.low8;
        genSubW( arg1f24.mid8, arg2f24.mid8);
        genSubW( arg1f24.high8, arg2f24.high8);
        goto _RETURN_MF;
    }
    if (!arg2f24.15)
        goto _RETURN_MF;
    W = arg2f24.low8 - arg1f24.low8;
    genSubW( arg2f24.mid8, arg1f24.mid8);
    genSubW( arg2f24.high8, arg1f24.high8);
  _RETURN_MF:
    if (Carry)
        return 0;
    return 1;
}


bit operator>= _f24_GE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    Carry = 1;
    if (!(arg1f24.high8 | arg2f24.high8))
        return Carry;
    if (!arg1f24.15)  {
        if (arg2f24.15)
            return Carry;
        W = arg1f24.low8 - arg2f24.low8;
        genSubW( arg1f24.mid8, arg2f24.mid8);
        genSubW( arg1f24.high8, arg2f24.high8);
        return Carry;
    }
    Carry = 0;
    if (!arg2f24.15)
        return Carry;
    W = arg2f24.low8 - arg1f24.low8;
    genSubW( arg2f24.mid8, arg1f24.mid8);
    genSubW( arg2f24.high8, arg1f24.high8);
    return Carry;
}



bit operator> _f24_GT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    Carry = 0;
    if (!(arg1f24.high8 | arg2f24.high8))
        return Carry;
    if (!arg1f24.15)  {
        if (arg2f24.15)
            goto _RETURN_MF;
        W = arg2f24.low8 - arg1f24.low8;
        genSubW( arg2f24.mid8, arg1f24.mid8);
        genSubW( arg2f24.high8, arg1f24.high8);
        goto _RETURN_MF;
    }
    if (!arg2f24.15)
        return Carry;
    W = arg1f24.low8 - arg2f24.low8;
    genSubW( arg1f24.mid8, arg2f24.mid8);
    genSubW( arg1f24.high8, arg2f24.high8);
  _RETURN_MF:
    if (Carry)
        return 0;
    return 1;
}



bit operator<= _f24_LE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
{
    Carry = 1;
    if (!(arg1f24.high8 | arg2f24.high8))
        return Carry;
    if (!arg1f24.15)  {
        Carry = 0;
        if (arg2f24.15)
            return Carry;
        W = arg2f24.low8 - arg1f24.low8;
        genSubW( arg2f24.mid8, arg1f24.mid8);
        genSubW( arg2f24.high8, arg1f24.high8);
        return Carry;
    }
    if (!arg2f24.15)
        return Carry;
    W = arg1f24.low8 - arg2f24.low8;
    genSubW( arg1f24.mid8, arg2f24.mid8);
    genSubW( arg1f24.high8, arg2f24.high8);
    return Carry;
}


#undef genAdd
#undef genSub
#undef genAddW
#undef genSubW

#pragma library 0
