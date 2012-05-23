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
 *    Filename:       Trajectories.h                                  *
 *    Date:           02/01/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for the trajectory generation functions.
 *
 **********************************************************************/

#ifndef TRAJ_H
#define TRAJ_H

#include "generic_defs.h" // FOR LNG datatype

/**********************************************************
 * BASIC Trajectory planners, derived from Microchip AN696
 *********************************************************/
typedef struct {
    unsigned enable    : 1; //enable <- INPUT
    unsigned active    : 1; //motion active -> OUTPUT
    unsigned exec      : 1; //exec motion -> INPUT
    unsigned busy      : 1; //motion being executed
                            //(internal flag)
    unsigned done      : 1; //motion completed -> OUTPUT
    unsigned half_move : 1; //half travel distance completed
                            //FOR PosTRAJ trapezoidal move
                            //(internal flag)
    unsigned neg_move  : 1; //move is negative
    unsigned UNUSED    : 1;
} tTRAJflags;

typedef struct {
    int32_t         qdPosition;    // 1.31 format
    LNG             qdVelocity;
    int16_t         qVLIM;
    int16_t         qACC;
    uint8_t         qVELshift;
    uint8_t         qACCshift;
    int16_t         qVelCOM;
    int32_t         qdPosCOM;
    int16_t         qFlatCOUNT;
    int32_t         qdHalfDIST;
    } tTRAJParm;

void InitTRAJ( tTRAJParm *pParm, tTRAJflags *pFlags);
void JogTRAJ( tTRAJParm *pParm, tTRAJflags *pFlags);
void PosTRAJ( tTRAJParm *pParm, tTRAJflags *pFlags);

#endif

