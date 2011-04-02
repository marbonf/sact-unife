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
 *    Filename:       FxSqrtAbs.c                                     *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains Fixed-Point implementation of Sqrt and Abs
 *
 ********************************************************************/
 

#include "my_fractmath.h"


/*********************************************
 * FxAbs
 *********************************************/
uint32_t FxAbs(int32_t var)
  {
    int32_t tmp = 0;
    tmp = (var^(var>>31)) - (var>>31);
    return (uint32_t)tmp;
  }
  
/*********************************************
 * MyAbs: basic logic
 *********************************************/
uint32_t MyAbs(int32_t var)
  {
    int32_t tmp = 0;
    if(var>0)
        tmp=var;
    if(var<0)
        tmp=-var;
    
    return (uint32_t)tmp;
  }

/*********************************************
 * iSqrt, works on "pure" integers
 * result max 32767 -> MAX 32 bit input:
 * 0x40000000
 *********************************************/
uint32_t iSqrt(uint32_t x)
{ 
    uint32_t op, res, one;

    op = x;
    res = 0;

    /* "one" starts at the highest power of four <= than the argument. */
    one = 1UL << 30;  /* second-to-top bit set, use 1u << 14 for uint16 type;
                         use 1uL<<30 for uint32 */
    while (one > op) one = one >> 2;

    while (one != 0) {
        if (op >= res + one) {
            op = op - res - one;
            res = res + (one << 1);  // <-- faster than 2 * one
        }
        res = res >> 1;
        one = one >> 2;
    }
    
    /* Do arithmetic rounding to nearest integer */
    if (op > res)
    {
        res++;
    }
    
    return res;
}
