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
 *    Filename:       ADC_DMA.h                                       *
 *    Date:           20/04/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for Peripheral Pin Selection module.
 *
 **********************************************************************/
 

#ifndef PPS_H
#define PPS_H

// REMAPPABLE PINS (definitions useful FOR INPUTS!)
#define RP0_IN   0b00000
#define RP1_IN   0b00001
#define RP2_IN   0b00010
#define RP3_IN   0b00011
#define RP4_IN   0b00100
#define RP5_IN   0b00101
#define RP6_IN   0b00110
#define RP7_IN   0b00111
#define RP8_IN   0b01000
#define RP9_IN   0b01001
#define RP10_IN  0b01010
#define RP11_IN  0b01011
#define RP12_IN  0b01100
#define RP13_IN  0b01101
#define RP14_IN  0b01110
#define RP15_IN  0b01111
#define RP16_IN  0b10000
#define RP17_IN  0b10001
#define RP18_IN  0b10010
#define RP19_IN  0b10011
#define RP20_IN  0b10100
#define RP21_IN  0b10101
#define RP22_IN  0b10110
#define RP23_IN  0b10111
#define RP24_IN  0b11000
#define RP25_IN  0b11001
#define RP_VSS   0b11111

//REMAPPABLE OUTPUT FUNCTIONS (definitions useful FOR OUTPUTS!)

#define RP_OUT_NULL     0b00000 //RPn tied to default port pin
#define RP_OUT_C1OUT    0b00001 //RPn tied to Comparator1 Output
#define RP_OUT_C2OUT    0b00010 //RPn tied to Comparator2 Output
#define RP_OUT_U1TX     0b00011 //RPn tied to UART1 Transmit
#define RP_OUT_U1RTS    0b00100 //RPn tied to UART1 Ready To Send
#define RP_OUT_U2TX     0b00101 //RPn tied to UART2 Transmit
#define RP_OUT_U2RTS    0b00110 //RPn tied to UART2 Ready To Send
#define RP_OUT_SDO1     0b00111 //RPn tied to SPI1 Data Output
#define RP_OUT_SCK1     0b01000 //RPn tied to SPI1 Clock Output
#define RP_OUT_SS1      0b01001 //RPn tied to SPI1 Slave Select Output
#define RP_OUT_SDO2     0b01010 //RPn tied to SPI2 Data Output
#define RP_OUT_SCK2     0b01011 //RPn tied to SPI2 Clock Output
#define RP_OUT_SS2      0b01100 //RPn tied to SPI2 Slave Select Output
#define RP_OUT_C1TX     0b10000 //RPn tied to ECAN1 Transmit
#define RP_OUT_OC1      0b10010 //RPn tied to Output Compare 1
#define RP_OUT_OC2      0b10011 //RPn tied to Output Compare 2
#define RP_OUT_OC3      0b10100 //RPn tied to Output Compare 3
#define RP_OUT_OC4      0b10101 //RPn tied to Output Compare 4
#define RP_OUT_UPDN1    0b11010 //RPn tied to QEI1 direction (UPDN) status
#define RP_OUT_UPDN2    0b11011 //RPn tied to QEI2 direction (UPDN) status

//Functions and Variables with Global Scope:
void PPS_Init(void);

#endif

