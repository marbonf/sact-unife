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
 *    Filename:       QEI.c                                           *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization (no ISR) for QEI.
 *
 **********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "QEI.h"

  
void QEI_Init(void)
{
    // QEICON - QEI Control Register
        // Bit15 - CNTERR (1=Position Error Count Has Occurred)
        // Bit14 - Unused
        // Bit13 - QEISIDL (0=QEI Continues in idle, 1=discontinue)
        // Bit12 - 1=Index Pin is High, 0=Low (Read Only)
        // Bit11 - 1=Pos Counter Dir is +,0=-
        //    Bits10-8 - QEIM, Interface Mode Select - see manual
        // Bit7 - SWPAB (1=A&B swapped)
        // Bit6 - PCDOUT (1=DIR Output Enable,0=I/O Pin)
        // Bit5 - 1=Enable Timer gate accumulation
        // Bits4-3 - Timer input prescale
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit2 - 1=Index Pulse resets counter,0=doesn't
        // Bit1 - Timer Clock Select 1=QEA (rising),0 = Tcy
        // Bit0 - Counter Dir - 1=QEB state determines, 0=Bit11 determines
    QEICONbits.QEIM = 0;     // Disable QEI Module, SEE LATER!!
    QEICONbits.CNTERR = 0;   // Clear any count errors
    QEICONbits.QEISIDL = 0;  // Continue operation during sleep
#ifdef REV1_BOARD
    QEICONbits.SWPAB = direction_flags.encoder2_chB_lead;//1;    // QEA and QEB swapped
#endif
#ifdef REV2_BOARD
    QEICONbits.SWPAB = direction_flags.encoder1_chB_lead;//1;    // QEA and QEB swapped
#endif
    QEICONbits.PCDOUT = 0;   // Normal I/O pin operation
    QEICONbits.POSRES = 0;   // Index pulse DO NOT resets position counter
            
    // DFLTCON - Digital Filter Control Register
        // Bit15-8 - Not Used
        // Bit7 - QEOUT (1=Filters enabled on QEA/QEB,0=disabled)
        // Bit6-4 - QECK (QEA/QEB Filter Clock Divide Bits)
        // 111=1:256
        // 110=1:128
        // :::
        // 000=1:1
        // Bit3 -1=Filter enabled on index, 0=disabled
        // Bits2-0 - INDEX Filter Clock Divide ratio as per Bits6-4
    DFLTCONbits.CEID = 1;     // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1;    // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 1;     // 1:2 clock divide for digital filter for QEn

    POSCNT = 0;               // Reset position counter
    MAXCNT = 0xFFFF; //Set maximum count to full scale

    IFS2bits.QEIIF=0; //Clear off any interrupt due to config

    // RE-ENABLE QEI
    QEICONbits.QEIM = 7; // 7->X4 | 5->X2 mode with position counter reset by match with MAXCNT

}  

