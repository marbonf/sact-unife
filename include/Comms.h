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
 *    Filename:       Comms.h                                         *
 *    Date:           20/08/2011                                      *
 *    File Version:   0.8                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for communication channels inits
 *
 **********************************************************************/
 
#ifndef COMMS_H
#define COMMS_H

#include "sys_hw.h"

#define BAUDRATE1       115200  //Desired Baud Rate UART1
#define BAUDRATE2       230400  //Desired Baud Rate UART2
//#define BRGVAL1         ((FCY/BAUDRATE1)/16)-1   //Formula for U1BRG register with BRGH = 0
#define BRGVAL1         ((FCY/BAUDRATE1)/4)-1    //Formula for U1BRG register with BRGH = 1
#define BRGVAL2         ((FCY/BAUDRATE2)/16)-1   //Formula for U2BRG register

// BUFFERING VARIABLES
extern unsigned char u1tmpbuf[];
extern uint8_t u1buftail;
extern uint8_t u1bufhead;

extern unsigned char u2tmpbuf[];
extern uint8_t u2buftail;
extern uint8_t u2bufhead;

// Function prototypes for global scope
void UART1_Init(void);
void UART2_Init(void);

//Functions and Variables with Global Scope:
void __attribute__((interrupt,auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt,auto_psv)) _U1RXInterrupt(void);
void __attribute__((interrupt,auto_psv)) _U2TXInterrupt(void);
void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void);

/**************************************************************
 * Functions for UART TX, can be redirected to UART1/UART2
 * NOTES:
 * - prototypes are NOT conforming to stdio.h style
 *   because of hw/compiler dependance (RXREG is 16 bits)
 * - 2nd param is a pointer to SFR structure (defined in
 *   target device header file)
 *   MUST CHECK if qualified as volatile or near
 * - IMPLEMENTED ONLY FOR 8n1 bits mode!
 *************************************************************/
void putsUART(unsigned char *buffer, volatile UART *ureg);
void putcUART(unsigned char byte, volatile UART *ureg);
void putiUART(int16_t k,volatile UART *ureg);
void putuiUART(uint16_t k,volatile UART *ureg);

void SendNUART(unsigned char *buffer, volatile UART *ureg, uint8_t N);

#endif
