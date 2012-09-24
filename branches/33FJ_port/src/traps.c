/*****************************************************************************
*                     Copyright (c) 2011, Marcello Bonfe'                     
*                   ENDIF - ENgineering Department In Ferrara,
*                           University of Ferrara.
*                            All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions 
*  are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the University of Ferrara nor the
*      names of its contributors may be used to endorse or promote products
*      derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
*  ARE DISCLAIMED. IN NO EVENT SHALL MARCELLO BONFE' BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
*  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
* 
*****************************************************************************
 *                                                                    *
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       traps.c                                         *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the traps routine.
 *
 **********************************************************************/
 
#include "sys_hw.h"
  
/*
Primary Exception Vector handlers:
These routines are used if INTCON2bits.ALTIVT = 0.
All trap service routines in this file simply ensure that device
continuously executes code within the trap service routine. Users
may modify the basic framework provided here to suit to the needs
of their application.
*/
void __attribute__((interrupt,auto_psv)) _OscillatorFail(void)
{
        INTCON1bits.OSCFAIL = 0;        //Clear the trap flag
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

void __attribute__((interrupt,auto_psv)) _AddressError(void)
{
        INTCON1bits.ADDRERR = 0;        //Clear the trap flag
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}
void __attribute__((interrupt,auto_psv)) _StackError(void)
{
        INTCON1bits.STKERR = 0;         //Clear the trap flag
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

void __attribute__((interrupt,auto_psv)) _MathError(void)
{
        INTCON1bits.MATHERR = 0;        //Clear the trap flag
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

/*
Alternate Exception Vector handlers:
These routines are used if INTCON2bits.ALTIVT = 1.
All trap service routines in this file simply ensure that device
continuously executes code within the trap service routine. Users
may modify the basic framework provided here to suit to the needs
of their application.
*/

void __attribute__((interrupt,auto_psv)) _AltOscillatorFail(void)
{
        INTCON1bits.OSCFAIL = 0;
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

void __attribute__((interrupt,auto_psv)) _AltAddressError(void)
{
        INTCON1bits.ADDRERR = 0;
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

void __attribute__((interrupt,auto_psv)) _AltStackError(void)
{
        INTCON1bits.STKERR = 0;
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

void __attribute__((interrupt,auto_psv)) _AltMathError(void)
{
        INTCON1bits.MATHERR = 0;
// SWITCH ON BOTH LEDS
        //LED1_TRIS = 0;
        //LED2_TRIS = 0;
        //LED1 = 1;
        //LED2 = 1;
        while (1);
}

