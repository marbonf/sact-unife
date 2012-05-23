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
 *    Filename:       PID.c                                           *
 *    Date:           29/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for reusable PID controller.
 *
 **********************************************************************/

#ifndef PID_H
#define PID_H

#include "generic_defs.h" // for data types

typedef struct {
    unsigned d_term_enable : 1; //must be disabled for first cycle
    unsigned saturated     : 1; //saturation
    unsigned UNUSED        : 6;
} tPIDflags;

typedef struct {
    int32_t     qdSum;    // 1.31 format
    int16_t     qKp;
    int16_t     qKi;
    int16_t     qKd;
    uint8_t     qN;        // 2^N scaling factor
    int32_t     qdOutMax;
    int32_t     qdOutMin;
    int32_t     qdInRef; 
    int32_t     qdInMeas;
    int32_t     qdErrPrev;
    int16_t     qOut;
    } tPIDParm;

// INIT MODE: -1 -> Out = OutMin, +1 -> Out = OutMax, 0 -> Out = 0
void InitPID( tPIDParm *pParm, tPIDflags *pFlags, int8_t mode);
void CalcPID( tPIDParm *pParm, tPIDflags *pFlags);
void CalcPI( tPIDParm *pParm, tPIDflags *pFlags);
void CalcP( tPIDParm *pParm, tPIDflags *pFlags);

#endif
