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
 *  This file contains the reusable code for PID controllers.
 *  - CalcPID computes full PID algorithm
 *  - CalcPI  computes only P+I terms
 *  - CalcP   computes only P term (and saturation)
 *
 **********************************************************************/

#include "PID.h"
#include "my_fractmath.h" // for RSH

/************************************************
* Init PID: resets integral, output and flags
************************************************/
void InitPID(tPIDParm *pParm, tPIDflags *pFlags, int8_t mode)
{
    switch(mode)
    {
        case 1: //pParm->qOut = (pParm->qdOutMax>>pParm->qN);
                pParm->qOut = RSH(pParm->qdOutMax,pParm->qN);
                pParm->qdSum = pParm->qdOutMax;
                break;
        case -1://pParm->qOut = (pParm->qdOutMin>>pParm->qN);
                pParm->qOut = RSH(pParm->qdOutMin,pParm->qN);
                pParm->qdSum = pParm->qdOutMin;
                break;
        case 0:    
        default:pParm->qOut = 0;
                pParm->qdSum = 0;
                break; 
    }
    
    pParm->qdErrPrev=0;
        
    pFlags->saturated = 0;
    pFlags->d_term_enable = 0;
    
}


/************************************************
* Calculate PID
************************************************/
void CalcPID(tPIDParm *pParm, tPIDflags *pFlags)
{
    int32_t p_term;
    int32_t d_term;
    int32_t temp;
    int32_t error;
    
    //START ASSUMING NOT IN LIMITATION
    pFlags->saturated = 0;
    
    error = pParm->qdInRef - pParm->qdInMeas;
    
    p_term = error * (int32_t)pParm->qKp;
    
    if(pFlags->d_term_enable)
        d_term = (error - pParm->qdErrPrev) * (int32_t)pParm->qKd;
    else
        d_term = 0;
        
    temp = p_term + d_term;
    
    // If in limit due to just P+D term then clear
    // off I term so that when we come out of limit
    // the I term can smoothly take over to eliminate 
    // steady state error.
    if(temp > pParm->qdOutMax)
    {
        pFlags->saturated = 1;
        temp = pParm->qdOutMax;
        pParm->qdSum = 0;
    }
    else
    {
        if(temp < pParm->qdOutMin)
        {
            pFlags->saturated = 1;
            temp = pParm->qdOutMin;
            pParm->qdSum = 0;
        }
    }
    
    // If NOT in limit, update I term
    if(!pFlags->saturated)
    {
        pParm->qdSum += error * (int32_t)pParm->qKi;
        temp += pParm->qdSum;
        
        if(temp > pParm->qdOutMax)
        {
            pFlags->saturated = 1;
            temp = pParm->qdOutMax;
            // CLAMP I term
            pParm->qdSum = temp - p_term - d_term;
        }
        else
        {
            if(temp < pParm->qdOutMin)
            {
                pFlags->saturated = 1;
                temp = pParm->qdOutMin;
                // CLAMP I term
                pParm->qdSum = temp - p_term - d_term;
            }
            //else
            //{
                // Output = unclamped P+I+D (still in temp)
            //}
        }        
    }
    
    // SCALE FINAL RESULT
    //pParm->qOut = (temp >> pParm->qN);
    pParm->qOut = RSH(temp, pParm->qN);
    
    // PREPARE NEXT CYCLE
    pParm->qdErrPrev = error;
    //enable D term, which must be disabled
    //at least for 1st cycle, since we don't
    //have a valid prev. error sample
    pFlags->d_term_enable = 1;     
}


/************************************************
* Calculate PI
************************************************/
void CalcPI(tPIDParm *pParm, tPIDflags *pFlags)
{
    int32_t p_term;
    int32_t temp;
    int32_t error;
    
    //START ASSUMING NOT IN LIMITATION
    pFlags->saturated = 0;
    
    error = pParm->qdInRef - pParm->qdInMeas;
    
    p_term = error * (int32_t)pParm->qKp;
            
    temp = p_term;
    
    // If in limit due to just P term then clear
    // off I term so that when we come out of limit
    // the I term can smoothly take over to eliminate 
    // steady state error.
    if(temp > pParm->qdOutMax)
    {
        pFlags->saturated = 1;
        temp = pParm->qdOutMax;
        pParm->qdSum = 0;
    }
    else
    {
        if(temp < pParm->qdOutMin)
        {
            pFlags->saturated = 1;
            temp = pParm->qdOutMin;
            pParm->qdSum = 0;
        }
    }
    
    // If NOT in limit, update I term
    if(!pFlags->saturated)
    {
        pParm->qdSum += error * (int32_t)pParm->qKi;
        temp += pParm->qdSum;
        
        if(temp > pParm->qdOutMax)
        {
            pFlags->saturated = 1;
            temp = pParm->qdOutMax;
            // CLAMP I term
            pParm->qdSum = temp - p_term;
        }
        else
        {
            if(temp < pParm->qdOutMin)
            {
                pFlags->saturated = 1;
                temp = pParm->qdOutMin;
                // CLAMP I term
                pParm->qdSum = temp - p_term;
            }
            //else
            //{
                // Output = unclamped P+I+D (still in temp)
            //}
        }        
    }
    
    // SCALE FINAL RESULT
    //pParm->qOut = (temp >> pParm->qN);
    pParm->qOut = RSH(temp, pParm->qN);
}

/************************************************
* Calculate P
************************************************/
void CalcP(tPIDParm *pParm, tPIDflags *pFlags)
{
    int32_t temp;
    
    //START ASSUMING NOT IN LIMITATION
    pFlags->saturated = 0;
    
    temp = (pParm->qdInRef - pParm->qdInMeas) * (int32_t)pParm->qKp;
    
    // If in limit, saturate output
    if(temp > pParm->qdOutMax)
    {
        pFlags->saturated = 1;
        temp = pParm->qdOutMax;
    }
    else
    {
        if(temp < pParm->qdOutMin)
        {
            pFlags->saturated = 1;
            temp = pParm->qdOutMin;
        }
    }
    
    // SCALE FINAL RESULT
    //pParm->qOut = (temp >> pParm->qN);
    pParm->qOut = RSH(temp, pParm->qN);
}

