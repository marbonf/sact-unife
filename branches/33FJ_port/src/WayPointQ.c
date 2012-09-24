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
 *    Filename:       WayPointQ.c                                     *
 *    Date:           01/08/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the way point queue management.
 *
 **********************************************************************/

#include "generic_defs.h"
#include "WayPointQ.h"

// PATH WAYPOINTS
volatile int16_t x_way[MAX_WAY];
volatile int16_t y_way[MAX_WAY];
volatile int16_t r_way[MAX_WAY];
volatile uint8_t way_head, way_tail;

void WayPointQ_Reset(void)
{
    way_head = 0;
    way_tail = 0;
}

uint8_t WayPointQ_InUse(void)
{
    return way_head - way_tail;
}

int WayPointQ_IsFull(void)
{
    return ((way_head - way_tail) == MAX_WAY);
}

int WayPointQ_IsEmpty(void)
{
    return ((way_head - way_tail) == 0);
}

int WayPointQ_Put(int16_t x, int16_t y, int16_t r)
{
    if( (way_head - way_tail) != MAX_WAY)
    {
        x_way[way_head%MAX_WAY] = x;
        y_way[way_head%MAX_WAY] = y;
        r_way[way_head%MAX_WAY] = r;
        way_head = (way_head+1);
    }
    else return 0;
    
    return 1;
}

int WayPointQ_Get(int16_t *x, int16_t *y, int16_t *r)
{
   if( (way_head - way_tail) != 0)
   {
      *x = x_way[way_tail%MAX_WAY];
      *y = y_way[way_tail%MAX_WAY];
      *r = r_way[way_tail%MAX_WAY];
      way_tail = (way_tail+1);
   }
   else return 0;
   
   return 1;            

}


