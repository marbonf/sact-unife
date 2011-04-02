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
 *    Filename:       Timers.c                                        *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization and ISR for any timer.
 *
 **********************************************************************/
 
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "Timers.h"
#include "Controls.h"

uint8_t speed_loop_count = 0;

/******************************************************
* TIMER INIT ROUTINES
******************************************************/

// TIMER1 and TIMER4 used for encoder counting
void Timer1_Init(void)
{
    // T1CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T1CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - Not Used
        // Bit 2 - 1= Synch external clock input
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T1CON = 0x0006;  //Timer1 settato per il conteggio sincrono
    PR1 = 0xFFFF;    //Period Register
    TMR1=0;
    T1CONbits.TON=1; //Start Timer1
}

void Timer4_Init(void)
{
    // T4CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T4CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - T45 - 1=Timer4/5 as 32 bit timer
        // Bit 2 - Not used
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T4CON = 0x0002;  //Timer1 settato per il conteggio sincrono
    PR4 = 0xFFFF;    //Period Register
    TMR4=0;
    T4CONbits.TON=1; //Start Timer1
}

//TIMER5 used to schedule high-freq control loop
void Timer5_Init(void)
{
    // T5CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T1CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - Not used
        // Bit 2 - Not used
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T5CON = 0x0010;    // Timer set w/1:8 prescaler
    
    // CHECK THAT FCY/N IS CONSISTENT WITH 1:N PRESCALER!!!
      PR5 = (FCY/8) / SPEED_LOOP_FREQ;
      
      // RESET INTERRUPT FLAG 
      IFS1bits.T5IF = 0;   
      
      // INTERRUPT ENABLE 
      IEC1bits.T5IE = 1;
      
      //Set the interrupt priority
    //7 = maximum
    //4 = default
    //0 = disable int.
      IPC5bits.T5IP = 6;  
      
      //ENABLE TIMER
      T5CONbits.TON=1;     
}

/************************************************************
* _T5Interrupt() is the Timer5 interrupt service routine (ISR).
* The routine must have global scope in order to be an ISR.
* The ISR name is chosen from the device linker script.
************************************************************/
void __attribute__((interrupt,auto_psv)) _T5Interrupt(void)
{
#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin2_OUT = 1;
#endif

IFS1bits.T5IF = 0;    // Clear interrupt flag

    if(control_flags.pos_loop_active)
    {
        SpeedLoops();
    }
    else if(control_flags.current_loop_active)
    {
        RefCurrentFilter();
    }

    // counter for loop frequency submultiple
    speed_loop_count++;
    
    if(speed_loop_count > (SPEED_LOOP_FREQ/POS_LOOP_FREQ - 1))
    {
        speed_loop_count = 0;
        UpdateOdometryFx();
        if(control_flags.pos_loop_active)
            PositionLoops();
        else if(control_flags.cart_loop_active)
            CartesianLoops();
    }

#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin2_OUT = 0;
#endif

}
