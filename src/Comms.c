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
 *    Filename:       Comms.c                                         *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains init and aux functions
 *  for ALL communication channels:
 *  UART1/2, I2C, CAN (not used..)
 *
 **********************************************************************/
 
#include "sys_hw.h"
#include "generic_defs.h"
#include "Comms.h"

// BUFFERING VARIABLES
unsigned char u1tmpbuf[256];
uint8_t u1buftail = 0;
uint8_t u1bufhead = 0;

unsigned char u2tmpbuf[256];
uint8_t u2buftail = 0;
uint8_t u2bufhead = 0;

void UART1_Init(void)
{

    U1MODE = 0x8000; // UART1 Enabled (bit 15)
                     // all other standard configs (8bit,noparity, etc)
    U1STA  = 0x0000; // resets all errors etc.
    
    U1BRG  = BRGVAL1; // See .h
    
    //reset flags
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
    
    //Set the interrupt priority
    //7 = maximum
    //4 = default
    //0 = disable int.
    IPC2bits.U1RXIP = 6;
    
    IEC0bits.U1RXIE = 1; // RX interrupt ENABLED
    IEC0bits.U1TXIE = 0; // TX interrupt DISABLED
    
    U1STAbits.UTXISEL1 = 0;   //Optional: setup UART1 transmitter to interrupt
    U1STAbits.UTXISEL0 = 0;   //when a character is transferred to the
                              //Transmit Shift register and as result,
                              //the transmit buffer becomes empty.
    U1STAbits.UTXEN = 1;    //Enable UART1 transmitter

} 

void UART2_Init(void)
{

    U2MODE = 0x8000; // UART1 Enabled (bit 15)
                     // all other standard configs (8bit,noparity, etc)
    U2STA  = 0x0000; // resets all errors etc.
    
    U2BRG  = BRGVAL2; // See .h
    
    //reset flags
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    
    IEC1bits.U2RXIE = 1; // RX interrupt ENABLED
    IEC1bits.U2TXIE = 0; // TX interrupt DISABLED
    
    U2STAbits.UTXISEL1 = 0;   //Optional: setup UART1 transmitter to interrupt
    U2STAbits.UTXISEL0 = 0;   //when a character is transferred to the
                              //Transmit Shift register and as result,
                              //the transmit buffer becomes empty.
    U2STAbits.UTXEN = 1;    //Enable UART1 transmitter

}

/****************************************
 * UART1/2 TX interrupt (Unused)
 ***************************************/
 void __attribute__((interrupt,auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;    //Clear the UART1 transmitter interrupt flag
}

void __attribute__((interrupt,auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0;    //Clear the UART2 transmitter interrupt flag
}

/****************************************
 * UART1 RX interrupt
 ***************************************/
void __attribute__((interrupt,auto_psv)) _U1RXInterrupt(void)
{
    
while(U1STAbits.URXDA) // DATA AVAILABLE
{
    // Clear overflow error
    // NOTE: this flushes the buffer!!!
    if(U1STAbits.OERR)
    {
        U1STAbits.OERR = 0;
        break;
    } 

    u1tmpbuf[u1bufhead++] = U1RXREG;

}// END WHILE URXDA (Data Available)

 IFS0bits.U1RXIF = 0; // RESET Interrupt FLAG HERE, buffer should be empty!
}// END U1 RX Interrupt


/****************************************
 * UART2 RX interrupt
 ***************************************/
void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
    
while(U2STAbits.URXDA) // DATA AVAILABLE
{
    // Clear overflow error
    // NOTE: this flushes the buffer!!!
    if(U2STAbits.OERR)
    {
        U2STAbits.OERR = 0;
        break;
    } 

    u2tmpbuf[u2bufhead++] = U2RXREG;

}// END WHILE URXDA (Data Available)

 IFS1bits.U2RXIF = 0; // RESET Interrupt FLAG HERE, buffer should be empty!
}// END U2 RX Interrupt


/**************************************************************
 * Functions for UART TX, can be redirected to UART1/UART2
 *************************************************************/

// putsUART(): sends a NULL terminated buffer on the required UART
void putsUART(unsigned char *buffer, volatile UART *ureg)
{
    unsigned char * temp_ptr = buffer;
    
    /* transmit till NULL character is encountered */
    
    while(*temp_ptr != NULLC)
        {
            while(ureg->uxsta & _16BIT_9);  /* wait if the buffer is full */
            ureg->uxtxreg = *temp_ptr++;   /* transfer data byte to TX reg */
        }
}

// putcUART(): sends a single character on the required UART
void putcUART(unsigned char symbol, volatile UART *ureg)
{
    while(ureg->uxsta & _16BIT_9);  /* wait if the buffer is full */
    ureg->uxtxreg = symbol;   /* transfer data byte to TX reg */
}

// putiUART(): sends an integer value as a string on the required UART
void putiUART(int16_t k,volatile UART *ureg)
{
unsigned char c;

if(k<0){
    k=-1*k;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = MINUS;
}
else{
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = PLUS;
}
    c = k/10000;
    if (c > 0)
        k = k - c*10000;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    c = k/1000;
    if (c > 0)
        k = k - c*1000;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    c = k/100;
    if (c > 0)
        k = k - c*100;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    c = k/10;
    if (c > 0)
        k = k - c*10;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (char)(k + ZERO);
}

// putiUART(): sends an unsigned integer value as a string on the required UART
void putuiUART(uint16_t k,volatile UART *ureg)
{
unsigned char c;

    c = k/10000;
    if (c > 0)
        k = k - c*10000;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);    
    c = k/1000;
    if (c > 0)
        k = k - c*1000;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    c = k/100;
    if (c > 0)
        k = k - c*100;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    c = k/10;
    if (c > 0)
        k = k - c*10;
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (c + ZERO);
    while (ureg->uxsta & _16BIT_9);
    ureg->uxtxreg = (char)(k + ZERO);
}


// SendNUART(): sends N characters from a buffer (NOT necessarily NULL terminated) to the required UART
void SendNUART(unsigned char *buffer, volatile UART *ureg, uint8_t N)
{
    uint8_t idx = 0;
    while(idx < N)
    {
        putcUART(buffer[idx],ureg);
        idx++;
    }
}
