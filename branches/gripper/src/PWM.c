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
 *    Filename:       PWM.c                                            *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization and ISR for PWM generator.
 *
 *  In particular, the WriteConfig() function inverts the polarity of
 *  The PWM output signals.  Normally, this would be done because
 *  the gate driver circuits require active low polarity.  However, in
 *  this application, the polarity is changed so that the active portion
 *  of the duty cycle will occur at the end of the PWM timer count
 *  period.  The PWM is configured to trigger an A/D conversion at
 *  the end of the PWM cycle.  This allows the A/D to always be 
 *  triggered at the end of the on time, regardless of the PWM duty
 *  cycle.  The user must write a complemented duty cycle value to 
 *  the PDC registers for this technique to work.  (Note:  This trick
 *  will only work when the PWM is operating in the independent mode
 *  where no dead time insertion is used.)
 *
 *                                          ADC triggered here
 *                                                |
 *  PWM1    _____                  _______________         
 *               |________________|               |_______
 *
 *  PWM2    _____          _______________________         
 *               |________|                       |_______ 
 *
 **********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "PWM.h"
  

// TO CHANGE PWM PIN POLARITY (inverted for better motor current sensing)
uint16_t ReadConfig(int16_t);
void WriteConfig(int16_t,int16_t);


void PWM_Init(void)
{
    uint16_t config_value,temp;
    
    // PTCON - Timebase Control
        // Bit 15 1=Timebase is ON, 0=OFF
        // Bit 14 - Not Implemented
        // Bit 13 1=Timebase Halts in CPU idle,0 = runs
        // Bits12-8 - Not Implemented
        // Bits7-4 Timebase Interrupt Postscaler Select
        // 1111 = 1:16 Postscale
        // ::::
        // 0001 = 1:2 Postscale
        // 0000 = 1:1 Postscle
        // Bits3-2 Timebase I/p Clk Prescale Select
        // 11 = 1:64 Prescale
        // 10 = 1:16 Prescale
        // 01 = 1:4 Prescale
        // 00 = 1:1 Prescale
        // Bits1-0 Timebase Mode Select
        // 11 = Continuous Up/Down with Double Updates
        // 10 = Continuous Up/Down
        // 01 = Single Event
        // 00 = Free-running        
    PTCON = 0x0000;     //Timebase OFF (turned on later), runs in idle, no post or prescaler, free-running
    PTMRbits.PTDIR = 0;                         
    PTMRbits.PTMR = 0; 
    
    // PTPER - Period Register
        // Bit 15 - Not Implemented
        // Bits14-0 - Value used for period comparison and therefore
        //              reset or change in direct of PWM ramp
        // PWM period is given by (PTER+1) * Tcy * Prescaler if in Free-run or single event
        //                    and by (PTER+1) * Tcy * Prescaler / 2 otherwise        
    PTPER = FULL_DUTY/2;
    
    // SEVTCMP - Special Event Trigger Control
        // Bit15 1=Trigger Occurs When Counting Down, 0 = Up
        // Bits14-0 = Special Event Compare Value
    SEVTCMPbits.SEVTDIR = 0;                        
    SEVTCMP = FULL_DUTY/2 - SAMPLE_ADVANCE;    // Trigger occurs just before switch turns off
    
    // PWMCON1 - PWM Control Register #1
        // Bits15-12 Not Implemented
        // Bits11-8 1=PWM Pin Pair Independent, 0=Complementary
        // Bit11 = PWM4 --- Bit8 = PWM1
        // Bits7-4 1=PWM High Side Pin is Enabled for PWM, 0 = I/O
        // Bit7 = PWM4 --- Bit4 = PWM1
        // Bits3-0 1=PWM Low Side Pin is Enabled for PWM, 0 = I/O
        // Bit3 = PWM4 --- Bit4 = PWM1
    PWMCON1 = 0x0F03; // PWM1/2 low-side firing signals USED (independent), all other general I/Os
    
    // PWMCON2 - PWM Control Register #2
        // Bits15-12 Not Implemented
        // Bits11-8 Special Event Trigger Postscale
        // 1111 = 1:16 Postscale
        // ::::
        // 0001 = 1:2 Postscale
        // 0000 = 1:1 Postscle
        // Bits7-2 Not Implemented
        // Bit1 - 1=Output Overrides Synch to PWM Timebase
        //          0=Output Overrides Occur on next Tcy boundary
        // Bit0 - 1=Updates from Duty Cycle and Period Registers Disabled
        //          0=Updates from Duty Cycle and Period Registers Enabled
    PWMCON2 = 0x0000;
    
    // DTCON1 & 2 - Deadtime control registers
        // See manual for details of bits
    DTCON1 = 0x0000; // Deadtime disabled
    DTCON2 = 0x0000; 
    
    // FLTACON - FaultA Input control register
        // Bits15-8 1=PWMs Driven ACTIVE on Fault, 0 = INACTIVE
        // Bit15 = #4 High Side
        // Bit14 = #4 Low Side
        // Bit13 = #3 High Side
        // :::::
        // Bit8 = #1 Low Side
        // Bit7 - 1=Cycle-cycle limit, 0=latched fault
        // Bits6-4 Not Implemented
        // Bits3-0 1=Pin Pair Controlled by FLTA, 0 = not controlled
        // Bit3 = #4 Pair
        // ::::
        // Bit0 = #1 Pair
    FLTACON = 0xFF00; //All pins driven ACTIVE on Fault, BUT FLTA control DISABLED
    
    // OVDCON - Override control register
        // Bits15-8 1= PWMs pin controlled by PWM generator, 0 = controlled by corresponding POUTxx bit
        // Bit15 = #4 High Side
        // Bit14 = #4 Low Side
        // Bit13 = #3 High Side
        // :::::
        // Bit8 = #1 Low Side
        // Bits7-0: POUTxx bits (1= pin driven ACTIVE, 0= pin driven INACTIVE
    OVDCON = 0x3FFF; // All but PWM4H/L (J10 terminal I/Os) override DISABLED
    

    // PDC1-4 - PWM#1-4 Duty Cycle Register
        // Bits15-0 PWM Duty Cycle Value    
    PDC1=FULL_DUTY;    // In this instance 0 duty cycle is full value
    PDC2=FULL_DUTY;    // in duty cycle registers due to inversion
    
    // CHANGE PWM PIN POLARITY    
    config_value=ReadConfig(4);
    temp=config_value;
    // Mask off the bits we are interested in and shift them down
    temp=(temp & 0x0700)>>8;
    // If the PWM polarity and reset definition bits are set 
    // correctly already then temp should be set to 4
    if (temp != 4)
    {
        // Form correct config_value
        config_value &= 0xF0FF;
        config_value |= 0x0400;
        // Write the new config value into F80004
        WriteConfig(4,config_value);
    }

    PTCONbits.PTEN = 1; //NOW that PWM polarity is inverted
                        //we can enable the PWM generator
                        
    IFS2bits.PWMIF = 0;
    IFS2bits.FLTAIF =0;    
    
    // ENABLE INTERRUPT!
    IEC2bits.PWMIE = 1;
}


// READ/WRITE CONFIG BITS
uint16_t ReadConfig(int16_t address)
{
// OLD CODE GENERATING WARNING (on unsafe access of WREG0)
//    // Set Table Page to point to config page
//    asm(" mov.w    #0x00F8,w2");
//    asm(" mov.w w2,TBLPAG");
//    // Now read the values
//    asm(" tblrdl [w0],w0");
//    // Note that strictly speaking the return WREG0
//    // is uneccesary as the correct result is already in W0.
//    // It does keep the compiler from generating a warning.
//    return WREG0;

// NEW CODE USING builtin functions
    TBLPAG = 0x00F8;
   return __builtin_tblrdl(address);
}

//NOTE: setup of NVMCON differs from dsPIC30F (0x4008)
//      and dsPIC33F (0x4000)
void WriteConfig(int16_t address, int16_t value) 
{ 
// TODO: modify also write using builtin (how????)    

    // Set Table Page to point to config page
    asm(" mov.w #0x00F8,w2");
    asm(" mov.w w2,TBLPAG");
    // Set NVMCON with correct value for program of
    // config location
    asm(" mov.w #0x4008,w2");
    asm(" mov.w w2,NVMCON");
    // Address will be in w0 and value in w1
    // load these into the programming registers  
    asm(" tblwtl W1,[W0]");
    asm(" bset   NVMCON,#14"); 
    asm(" mov    #0x55,W2 ");
    asm(" mov    W2,NVMKEY ");
    asm(" mov    #0xaa,W2 ");
    asm(" mov    W2,NVMKEY ");
    asm(" bset   NVMCON,#15"); 

    while(NVMCONbits.WR); 
}
  
/*****************************************************
 * ISRs for PWM interrupt: manage ticks count for a
 * quite simple real-time process scheduling
 ****************************************************/
void __attribute__((interrupt,no_auto_psv)) _PWMInterrupt(void)
{
    slow_event_count++;
    medium_event_count++;
    IFS2bits.PWMIF = 0;
}
