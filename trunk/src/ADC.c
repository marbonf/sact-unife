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
 *    Filename:       ADC.c          	                              *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains the initialization and ISR for ADC.
 *
 **********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "generic_defs.h"
#include "ADC.h"
#include "PWM.h"
#include "Controls.h"

// LOCALS
int16_t mcurrent1samp[MCURR_MAV_ORDER],mcurrent2samp[MCURR_MAV_ORDER];
int16_t mcurrent_temp;
uint8_t mcurr_idxtemp;
uint8_t mcurrsampIdx = 0;
uint8_t ADindex = 0;


// ADC_Init() is used to configure A/D to convert 
// AN0-1 (Current sense) with PWM synchronization
// other channels are scanned
//channel per interrupt. The A/D is set up for a sampling rate of 1MSPS
//Timer3 is used to provide sampling time delay.
//The input pin being acquired and converted is AN7.
void ADC_Init(void)
{
        //ADCON1 Register
        //Set up A/D for Automatic Sampling
        //Use internal counter (SAMC) to provide sampling time
        //Set up A/D conversrion results to be read in integer
        //number format.
        //Set up simultaneous sampling for multiple S/H amplifiers
        //All other bits....
        ADCON1 = 0;
        ADCON1bits.FORM = 0;
        ADCON1bits.SSRC = 3; //sincronizzato con il PWM
        ADCON1bits.ASAM = 1;
        ADCON1bits.SIMSAM = 1;
        ADCON1bits.SAMP = 0; //se ASAM=1 non dovrebbe influire

        //ADCON2 Register
        //Set up A/D for interrupting every sample
        //Set up to sample on 4 S/H amplifiers - CH0,1,2,3
        //No input scans
        //All other bits to default
        ADCON2 = 0;
        ADCON2bits.SMPI = 0;
        ADCON2bits.CHPS = 2; //CH0,1,2,3
        ADCON2bits.ALTS = 0; //Always use MUX A input multiplexer settings
		//ADCON2bits.VCFG = 3; //0 - AVdd/AVss, 3 - (Ideally) use external references

        //ADCON3 Register
        //At 14.7 MIPS, Tcy = 67.8 ns = Instruction Cycle Time
        //The A/D converter will take 12*Tad periods to convert each sample
        //So to convert 4 channels within PWM rate (50us @ 20KHz)
        //Using equation in the Family Reference Manual we have
        //ADCS = 2*Tad/Tcy - 1 = 2 us per channel (4) -> 8 us
        ADCON3 = 0;
        ADCON3bits.SAMC = 4; //at least one Tad left for sampling
        ADCON3bits.ADCS = 4; //Tad = (ADCS+1)*Tcy/2

        //ADCHS Register
        //Set up A/D Channel Select Register to convert
        //AN8 on Mux A input of CH0
        //AN0 on CH1 (Current motor 2!!!!), AN1 on CH2 (Current motor 1!!!)
        //AN2 on CH3 (unused) S/H amplifiers
        ADCHS = 0x0008; 
        
        //ADCSSL Register
        //Channel Scanning is disabled. All bits left to their default state
        ADCSSL = 0x0000;

        //ADPCFG Register
        //Set up channels as analog inputs and configure rest as digital
        //Recall that we configured all A/D pins as digital when code execution
        //entered main() out of reset
        ADPCFG = 0xFFFF;
        	CURRSENSE1_PCFG = ANALOG;
        	CURRSENSE2_PCFG = ANALOG;
        	AN8_PCFG = ANALOG;
        	AN9_PCFG = ANALOG;
        	AN10_PCFG = ANALOG;
        	AN11_PCFG = ANALOG;
        	AN12_PCFG = ANALOG;
        	AN13_PCFG = ANALOG;
        	AN14_PCFG = ANALOG;
        	AN15_PCFG = ANALOG;
        
        
        // FOR ARRAY STORAGE OF SAMPLING:
		ADindex = 0;
		mcurrsampIdx = 0;

		

        //Clear the A/D interrupt flag bit
        IFS0bits.ADIF = 0;

        //Set the A/D interrupt enable bit
        IEC0bits.ADIE = 1;
        
        //Set the interrupt priority
        //7 = maximum
    	//4 = default
    	//0 = disable int.
        IPC2bits.ADIP = 7;

        //Turn on the A/D converter
        //This is typically done after configuring other registers
        ADCON1bits.ADON = 1;

}

/************************************************************
* _ADCInterrupt() is the A/D interrupt service routine (ISR).
* The routine must have global scope in order to be an ISR.
* The ISR name is chosen from the device linker script.
************************************************************/
void __attribute__((interrupt,auto_psv)) _ADCInterrupt(void)
{
#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin1_OUT = 1;
#endif

#ifdef SIMULATE
	if(control_flags.current_loop_active)
	{
		mcurrent1 = ((FULL_DUTY - PDC1) >> 2) + mcurrent1_offset; 
		mcurrent2 = ((FULL_DUTY - PDC2) >> 2) + mcurrent2_offset;
	}
	else
	{
		mcurrent1_filt = mcurrent1_offset;
		mcurrent2_filt = mcurrent2_offset;
	}
#else
	// motor currents
	// N.1 (AN1)
	mcurrent1 = ADCBUF2;
	
	// N.2 (AN0)
	mcurrent2 = ADCBUF1;
#endif

	// standard Analog inputs
	ADResult[ADindex] = ADCBUF0;
	
	//update index
	ADindex++;
	if(ADindex > 7) ADindex = 0;
	
	// scan channels 8-15
	if(ADCHSbits.CH0SA == 15)
		ADCHSbits.CH0SA = 8;
	else
		ADCHSbits.CH0SA++;

	// moving average filtering
	mcurrent1samp[mcurrsampIdx] = mcurrent1;
	mcurrent2samp[mcurrsampIdx] = mcurrent2;
	//update index
	mcurrsampIdx++;
	if(mcurrsampIdx > (MCURR_MAV_ORDER - 1)) mcurrsampIdx = 0;

	//execute CURRENT CONTROL LOOP (if active)
	if(control_flags.current_loop_active && (mcurrsampIdx == 0))
	{
		mcurrent_temp = 0;
		mcurr_idxtemp = 0;
		while(mcurr_idxtemp < MCURR_MAV_ORDER)
		{
			mcurrent_temp += mcurrent1samp[mcurr_idxtemp];
			mcurr_idxtemp++;
		}
		mcurrent1_filt = (mcurrent_temp >> MCURR_MAV_SHIFT);

		mcurrent_temp = 0;
		mcurr_idxtemp = 0;
		while(mcurr_idxtemp < MCURR_MAV_ORDER)
		{
			mcurrent_temp += mcurrent2samp[mcurr_idxtemp];
			mcurr_idxtemp++;
		}
		mcurrent2_filt = (mcurrent_temp >> MCURR_MAV_SHIFT);

		CurrentLoops();
	}

#ifdef DEVELOP_MODE 
#ifdef LOG_ADCINT
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
	dataLOGdecim++;
	if(dataLOGdecim == LOGDECIM)
	{ 
		//dataLOG1[dataLOGIdx] = mcurrent1_filt-mcurrent1_offset;
		dataLOG2[dataLOGIdx] = rcurrent1;
		
		dataLOG3[dataLOGIdx] = PIDCurrent1.qOut;
		
		dataLOG4[dataLOGIdx] = (int16_t)PIDCurrent1.qdSum;
		if(DIR1)
			dataLOG1[dataLOGIdx] = -(mcurrent1_filt-mcurrent1_offset);
		else
			dataLOG1[dataLOGIdx] = mcurrent1_filt-mcurrent1_offset;
		
		//if(DIR2)
		//	dataLOG3[dataLOGIdx] = mcurrent2_filt;
		//else
		//	dataLOG3[dataLOGIdx] = -mcurrent2_filt;	
			
		//dataLOG4[dataLOGIdx] = rcurrent1;
		//dataLOG4[dataLOGIdx] = rcurrent2;
		
		dataLOGIdx++;
		if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
		
		dataLOGdecim = 0;
	}// IF DECIMATION	
#endif //LOG_ADCINT

// FOR TEST PROBE
J10Pin1_OUT = 0;
#endif

    //Clear the A/D Interrupt flag bit
    IFS0bits.ADIF = 0;
}

