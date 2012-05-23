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
 *    Filename:       Q16wrappers.c                                   *
 *    Date:           21/1/2011                                       *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains wrappers for Microchip Fixed-Point Math library
 *  particularly _Q16Atan2.
 *
 **********************************************************************/

#include "my_fractmath.h"

// WRAPPER TO USE _Q16atanYByX as a four-quadrant (atan2) version, output in Q16
_Q16 _Q16atan2(_Q16 x, _Q16 y)
{
    _Q16 temp;
    
    if(x == 0)
    {
        if(y > 0)
            temp = PI_Q16 >> 1; // PI/2
        else if(y < 0)
                 temp = - (PI_Q16 >> 1); // -PI/2
            else temp = 0;
    }    
    else
    {
         temp = _Q16atanYByX(x,y); //SCALED in Q16 (15.16)
         if(x < 0)
         {
              if(y >= 0)
                  temp += PI_Q16;
             else
                 temp -= PI_Q16;
         }
    }
    
    return temp;
}// END _Q16atan2..

// WRAPPER TO USE _Q16atanYByXByPI as a four-quadrant (atan2) version, output in BRADS
_Q16 _Q16atan2ByPI(_Q16 x, _Q16 y)
{
    _Q16 temp;
    
    if(x == 0)
    {
        if(y > 0)
            temp = HALF_PI_BRADS; // PI/2 in BRADS
        else if(y < 0)
                 temp = mHALF_PI_BRADS; // -PI/2 in BRADS
            else temp = 0;
    }    
    else
    {
         temp = _Q16atanYByXByPI(x,y)<<15; //OUTPUT SCALED
                                            //from Q16 (15.16) to BRADS
         if(x < 0)
         {
              if(y >= 0)
                  temp += PI_BRADS;
             else
                 temp += mPI_BRADS;
         }
    }
    
    return temp;
}// END _Q16atan2..

