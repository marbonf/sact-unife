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
 *    Filename:       Trajectories.c                                  *
 *    Date:           02/01/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the trajectory generation functions.
 *
 **********************************************************************/

#include "Trajectories.h"

#include "my_fractmath.h" // for all math stuffs

/************************************************
* Init TRAJ: resets, output and flags
************************************************/
void InitTRAJ(tTRAJParm *pParm, tTRAJflags *pFlags)
{

}

/************************************************
* Calculate JOG mode trajectory:
* endless feed w/piecewise velocity
************************************************/
void JogTRAJ(tTRAJParm *pParm, tTRAJflags *pFlags)
{
    if(pFlags->enable)
    {
        if(pParm->qdVelocity.i[1] < pParm->qVelCOM)
        {
            // commanded velocity
              pParm->qdVelocity.l += ((int32_t)pParm->qACC << pParm->qACCshift); // accelerate
              if(pParm->qdVelocity.i[1] > pParm->qVelCOM) // don't exceed commanded velocity
                pParm->qdVelocity.i[1] = pParm->qVelCOM;
              if(pParm->qdVelocity.i[1] > pParm->qVLIM) // Don't exceed velocity limit parameter
                pParm->qdVelocity.i[1] = pParm->qVLIM;
        }
        else // Is velact1 >= velcom1
              if(pParm->qdVelocity.i[1] > pParm->qVelCOM) // If current velocity is more than
              { // commanded velocity
                  pParm->qdVelocity.l -= ((int32_t)pParm->qACC << pParm->qACCshift); // decelerate
                  if(pParm->qdVelocity.i[1] < pParm->qVelCOM) // don't exceed commanded velocity
                    pParm->qdVelocity.i[1] = pParm->qVelCOM;
                  if( -pParm->qdVelocity.i[1] > pParm->qVLIM) // Don't exceed velocity limit parameter
                    pParm->qdVelocity.i[1] = -pParm->qVLIM;
              } 

        //pParm->qdPosition += (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
        pParm->qdPosition += (int32_t)RSH(pParm->qdVelocity.i[1], pParm->qVELshift);
    }
    else
    {
        pParm->qdVelocity.l = 0;
    }
    
    pFlags->active = pFlags->enable;
}

/************************************************
* Calculate POSITION mode trajectory:
* Simple Trapezoidal velocity "move-to-position"
************************************************/
void PosTRAJ(tTRAJParm *pParm, tTRAJflags *pFlags)
{
  if(pFlags->enable)							//if enable
  {
    if(pFlags->exec && !pFlags->done)			//it's executing and the motion is not complete
    {
      // get rising edge of exec flag
      if(!pFlags->busy)							//if free
      { 
        // compute half distance
        //pParm->qdHalfDIST = (pParm->qdPosCOM - pParm->qdPosition)>>1;
        pParm->qdHalfDIST = RSH((pParm->qdPosCOM - pParm->qdPosition),1);
        
        if(pParm->qdHalfDIST < 0)				
        {
           pParm->qdHalfDIST = - pParm->qdHalfDIST;
           pFlags->neg_move = 1;
        }
        else
           pFlags->neg_move = 0;				

        pParm->qdVelocity.l = 0;				//reset velocity
        pParm->qFlatCOUNT = 0;					//reset flatcount
        pFlags->half_move = 0;					//first half of motion
        pFlags->busy = 1;
      }//END IF(!..busy)
      else //motion in execution (busy flag active)	//trapezoidal motion implementation
      {
        if(!pFlags->half_move)						//first half of motion
        {
          if(pParm->qdVelocity.i[1] < pParm->qVLIM)	//if speed is smaller than max speed
          {
            // increase commanded velocity
            pParm->qdVelocity.l += ((int32_t)pParm->qACC << pParm->qACCshift); // accelerate
            if(pParm->qdVelocity.i[1] > pParm->qVLIM) // Don't exceed velocity limit parameter
               pParm->qdVelocity.i[1] = pParm->qVLIM;
          }
          else 										//velocity limit has been reached, increment flatcount
            pParm->qFlatCOUNT++;

          // decrement half distance
          //pParm->qdHalfDIST -= (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
          pParm->qdHalfDIST -= (int32_t)RSH(pParm->qdVelocity.i[1],pParm->qVELshift);

          if(pFlags->neg_move)
             //pParm->qdPosition -= (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
            pParm->qdPosition -= (int32_t)RSH(pParm->qdVelocity.i[1], pParm->qVELshift);
          else
             //pParm->qdPosition += (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
            pParm->qdPosition += (int32_t)RSH(pParm->qdVelocity.i[1], pParm->qVELshift);

          //if half distance is negative, first half of the move is completed
          if(pParm->qdHalfDIST <= 0)
             pFlags->half_move = 1;
            
        }//end IF(!pFlags->half_move)
        else //half_move flag is active(seconda metà del movimento)
        {
          if(pParm->qFlatCOUNT)
             pParm->qFlatCOUNT--; //decrement flatcount until deceleration phase should start
          else
          {
             if(pParm->qdVelocity.l) //if velocity is not zero
             {
                pParm->qdVelocity.l -= ((int32_t)pParm->qACC << pParm->qACCshift); // decelerate
                if(pParm->qdVelocity.i[1] < 0) // Don't exceed velocity limit parameter
                    pParm->qdVelocity.i[1] = 0;
             }
             else // velocity IS zero, motion completed
             {
               pFlags->done = 1;				//movimento completo
               pParm->qdPosition = pParm->qdPosCOM;
             }
          }//end ELSE (for deceleration phase)
          
          if(pFlags->neg_move)
             //pParm->qdPosition -= (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
            pParm->qdPosition -= (int32_t)RSH(pParm->qdVelocity.i[1], pParm->qVELshift);
          else
             //pParm->qdPosition += (int32_t)(pParm->qdVelocity.i[1] >> pParm->qVELshift);
            pParm->qdPosition += (int32_t)RSH(pParm->qdVelocity.i[1], pParm->qVELshift);

        }//END else (half_move flag is active)
      }//END motion is execution (busy flag active)
    }//END IF(...exec)
    else
    {
        pFlags->busy = 0;
        pFlags->done = 0;
    }
    
  }//END IF(.. enable)

  pFlags->active = pFlags->enable;  
}


