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
******************************************************************************
 *                                                                    *
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       my_fractmath.h                                  *
 *    Date:           18/1/2011                                       *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains prototypes for C implementation of
 *  several useful math operations, some wrapped from
 *  Microchip Fixed-Point Math library
 *
 **********************************************************************/
#ifndef MY_FRACTMATH_H
#define MY_FRACTMATH_H

#include "generic_defs.h" // for data types
#include <libq.h>

/* 2.30: 2 integral bits and 30 fractional bits
 * The definitions below yield 2 integer bits
 * 30 fractional bits */
#define FRACBITS 30 /* Must be even for FxSqrt (Turkowski) */
#define ITERS (15 + (FRACBITS >> 1))
//typedef int32_t Fract;

/* USEFUL TRIG CONSTANTS IN BRADS UNITS */
#define PI_BRADS       0x7FFFFFFF
#define mPI_BRADS      0x80000000
#define HALF_PI_BRADS  0x40000000
#define mHALF_PI_BRADS 0xC0000000

/* MAIN TRIG CONSTANTS IN Q16 */
#define PI_Q16           0x0003243F // PI*2^16 ~ 205887

/* MAIN TRIG CONSTANTS IN Q13, 16bit */
#define PI_Q13           0x6488 // PI*2^13 ~ 25736

/* Prototypes for iSqrt() and FxAbs(), in FxSqrtAbs.c */
uint32_t iSqrt(uint32_t a);
uint32_t FxAbs(int32_t var);
uint32_t MyAbs(int32_t var);

/* Prototype for _Q16atan2(),_Q16atan2ByPI() (Q16wrappers.c) */
_Q16 _Q16atan2(_Q16 x, _Q16 y); // output Q16
_Q16 _Q16atan2ByPI(_Q16 x, _Q16 y); // output BRADS

/* RIGHT shift for signed integers that rounds toward ZERO */
#define RSH(val,bits) ((val < 0) ? -((-val)>>bits) : val >> bits)


#endif
