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
 *    Filename:       Trajectories.h          	                      *
 *    Date:           2/1/2011                                        *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *	Code Description
 *  
 *  Header file for the trajectory generation functions.
 *
 **********************************************************************/

#ifndef TRAJ_H
#define TRAJ_H

#include "generic_defs.h" // FOR LNG datatype

#include "my_fractmath.h" // for all math stuffs

/**********************************************************
 * BASIC Trajectory planners, derived from Microchip AN696
 *********************************************************/
typedef struct {
	unsigned enable 			: 1; //enable <- INPUT
	unsigned active				: 1; //motion active -> OUTPUT
	unsigned UNUSED				: 6;
} tTRAJflags;

typedef struct {
    long 			qdPosition;	// 1.31 format
    LNG				qdVelocity;
    int				qVLIM;
    int				qACC;
    unsigned char   qVELshift;
    unsigned char   qACCshift;
    int				qVelCOM;
  	long			qdPosFIN;
    } tTRAJParm;

void InitTRAJ( tTRAJParm *pParm, tTRAJflags *pFlags);
void JogTRAJ( tTRAJParm *pParm, tTRAJflags *pFlags);

/**********************************************************
 * ADVANCED Trajectory planners, applying nonlinear
 * filtering theory (Zanasi-et-al.):
 * SECOND-ORDER NONLINEAR FILTER
 * Implementation notes:
 * - fixed-point version, requires Umax and Fc (= 1/Tc)
 *   to be a power of 2, to use only shifting (no div.)
*******************************************************/
typedef struct {
    long 			qdXint;	// 1.31 format
    long			qdXdot_int;
    long			qdRprev; // if MODE = 1 -> previous command
							 // if MODE = 2 -> command derivative
  	long			qdRcommand; // if MODE = 1 -> current command
							 	// if MODE = 2 -> tracking error, calculated outside
  	unsigned char   MODE;
    } tNLFStatus;

typedef struct {
    long 			qdX;	// 1.31 format
    long			qdXdot;
    long			qdXddot;
    } tNLFOut;


void InitNLFilter2Fx(tNLFOut *NLFOut,tNLFStatus *NLFStatus);

void NLFilter2Fx(tNLFOut *NLFOut,tNLFStatus *NLFStatus, 			// DATA STRUCTURES
				 unsigned long Xdot_max, unsigned char Umax_SHIFT,	// DYNAMIC LIMITS
				 unsigned char Fc_SHIFT);							// SAMPLING FREQUENCY
 
#endif

