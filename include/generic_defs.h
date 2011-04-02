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
 *    Filename:       generic_defs.h                                  *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains generic and usefult defines.
 *
 **********************************************************************/
#ifndef GENERIC_DEFS_H
#define GENERIC_DEFS_H
 
#define INPUT  1
#define OUTPUT 0
 
#define ANALOG  0
#define DIGITAL 1

#define TRUE  1
#define FALSE 0

// Useful ASCII chars 
#define CR 0x0D
#define LF 0x0A 
#define HT 0x09 // TAB horizontal
#define VL 0x7C // |
#define SC 0x3B // ;
#define PLUS 0x2B
#define MINUS 0x2D
#define ZERO  0x30
#define SPACE 0x20
#define NULLC 0x00

// Useful macro for 16 bit manipulation
#define _16BIT_0  0x0001
#define _16BIT_1  0x0002
#define _16BIT_2  0x0004
#define _16BIT_3  0x0008
#define _16BIT_4  0x0010
#define _16BIT_5  0x0020
#define _16BIT_6  0x0040
#define _16BIT_7  0x0080
#define _16BIT_8  0x0100
#define _16BIT_9  0x0200
#define _16BIT_10 0x0400
#define _16BIT_11 0x0800
#define _16BIT_12 0x1000
#define _16BIT_13 0x2000
#define _16BIT_14 0x4000
#define _16BIT_15 0x8000

// Useful macro for 8 bit manipulation
#define _8BIT_0 0x01
#define _8BIT_1 0x02
#define _8BIT_2 0x04
#define _8BIT_3 0x08
#define _8BIT_4 0x10
#define _8BIT_5 0x20
#define _8BIT_6 0x40
#define _8BIT_7 0x80

// ISO C99 stdint.h compatible data types
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;

// LONG TYPE FOR BINARY MODE MANAGEMENT
typedef union {
  int32_t      l;
  uint32_t     ul;
  int16_t     i[2];
  uint16_t    ui[2];
  int8_t     c[4];
  uint8_t     uc[4];
} LNG;

// WORD TYPE FOR BINARY MODE MANAGEMENT
typedef union
{
  uint16_t  ui;
  int16_t   i;
  int8_t    c[2];
  uint8_t   uc[2];
} WRD;

#endif
