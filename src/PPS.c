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
 *    Filename:       PPS.c                                       *
 *    Date:           20/09/2012                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.30                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization for Peripheral Pin Selections.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "extern_globals.h"
#include "generic_defs.h"
#include "PPS.h"

/************************************************************************
Function:
PPS_Init(void)
serve per rimappare i pin del modulo QEI come input
serve per rimappare i pin del modulo UART sia come input che come output
**********************************************************************/

void PPS_Init(void)
{
	// sblocco il registro
	__builtin_write_OSCCONL(OSCCON & 0xbf);

	// Configure Input Functions *********************
    //FUNCTION REGISTER =   REMAPPABLE PIN (see PPS.h)
	RPINR14bits.QEA1R   =   RP1_IN;	    //rimappo nel pin 5 (RP1)
	RPINR14bits.QEB1R   =   RP2_IN;	    //rimappo nel pin 6 (RP2)
	RPINR16bits.QEA2R   =   RP4_IN;	    //rimappo nel pin 11 (RP4)
	RPINR16bits.QEB2R   =   RP7_IN;	    //rimappo nel pin 16 (RP7)
	RPINR18bits.U1RXR   =   RP11_IN;    //rimappo nel pin 22 (RP11)
    RPINR4bits.T4CKR    =   RP3_IN;     //rimappo nel pin 7 (RP3)
	
	//RPINR15bits.INDXIR 	= 	xxxx; //se mi servisse rimappare l'index
	
	// Configure Output Functions *********************
    // REMAPPABLE PIN   =   FUNCTION VALUE (see PPS.h)
	// RPOR7bits.RP15R	=	RP_OUT_UPDN1;	//se mi servisse rimappare l'UPDN

	RPOR2bits.RP5R		= 	RP_OUT_NULL;
	RPOR5bits.RP10R		=	RP_OUT_U1TX; //rimappo nel pin 21 (RP10)

	// blocco il registro
	__builtin_write_OSCCONL(OSCCON | 0x40);

}
