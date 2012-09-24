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
 *                                                                *
 *    Author: Marcello Bonfe'                                     *
 *                                                                *
 *    Filename:       sys_hw.h                                    *
 *    Date:           28/12/2012                                  *
 *    File Version:   0.1                                         *
 *    Compiler:       MPLAB C30 v3.30                             *
 *                                                                *
 ******************************************************************
 *    Code Description
 *  
 *  This file contains system and hardware specific definitions.
 *  .....
 *
 ********************************************************************/
#ifndef SYS_HW_H
#define SYS_HW_H
 
#include <p33fxxxx.h>

/****************************************************************
 * DEFINITIONS TO ADAPT BOARD SPECIFIC OPTIONS OR SIMULATIONS
 ***************************************************************/
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//#define SIMULATE_FULL
#define SIMULATE_BASIC


/****************************************************************
 * IF COMPILED FOR DEVELOP MODE
 * ADDS DATALOGS AND TEST PROBE OUTPUTS FOR TASK MONITORING
 ***************************************************************/
#define DEVELOP_MODE
//UNCOMMENT THE FOLLOWING IF LONG (data-type) DATALOG IS DESIRED
#define LOG_LONG
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//#define LOG_POSLOOP
#define LOG_CARTLOOP
//#define LOG_SPEEDLOOP
//#define LOG_ADCINT

/*******************************************************************
 * System Clock Timing -
 * For oscillator configuration FRC (7.37MHz x PLL config),
 * Device Throughput in MIPS = Fcy = 7370000*40 / 2 / 2 / 2 = ~36.85 MIPS
 * Instruction Cycle time = Tcy = 1/(Fcy) = ~27 nanoseconds
 ******************************************************************/
#define FRC_FREQ    7370000     //On-board Crystal fcy
#define PLL_M       40         //On-chip PLL setting
#define PLL_N2      2
#define PLL_N1      2
//Instruction Cycle Fcy
#define FCY         FRC_FREQ*PLL_M / PLL_N2 / PLL_N1 / 2

/*******************************************************************
 * H-Bridge Control pins (PWM/DIR/BRAKE) 
 * and feedback (Thermal Flag, Current Sense)
 * for LMD18200
 ******************************************************************/
 
// PWMx pins are "Module controlled", no need to set TRISx manually!
//#define PWM1_TRIS TRISEbits.TRISE0
//#define PWM2_TRIS TRISEbits.TRISE2
 
// Other pins are instead standard dig.outputs..
#define DIR1_TRIS TRISBbits.TRISB14
#define DIR2_TRIS TRISBbits.TRISB12
 
// NO BRAKE USE!!!
//#define BRAKE1_TRIS TRISEbits.TRISB14
//#define BRAKE2_TRIS TRISEbits.TRISB12
 
//#define THERMFLG1_TRIS TRISBbits.TRISB3
//#define THERMFLG2_TRIS TRISBbits.TRISB2
 
#define CURRSENSE1_TRIS TRISAbits.TRISA0
#define CURRSENSE2_TRIS TRISAbits.TRISA1
 
// Current sense analog pin PCFG (Port ConFiGuration)
// 1 - Digital, 0 - Analog
#define CURRSENSE1_PCFG ADPCFGbits.PCFG0
#define CURRSENSE2_PCFG ADPCFGbits.PCFG1
 
// PWMx pins are controlled by duty-cycle generators, no need to use LATx
//#define PWM1 LATEbits.LATE0 
//#define PWM2 LATEbits.LATE2 

// Other pins are instead standard digital I/Os
#define DIR1 LATBbits.LATB14 
#define DIR2 LATBbits.LATB12 

//#define BRAKE1 LATEbits.LATE4 
//#define BRAKE2 LATEbits.LATE5 

//#define THERMFLG1 PORTBbits.RB3 
//#define THERMFLG2 PORTBbits.RB2 


/******************************************************************
* Encoders: QEI inputs and T1/T4 CLK
******************************************************************/

// QEI mode -> "Module controlled", no need to set TRISx manually
#define QEA_TRIS TRISBbits.TRISB4
#define QEB_TRIS TRISBbits.TRISB5

#define T1CK_TRIS TRISCbits.TRISC14
#define T4CK_TRIS TRISCbits.TRISC13

/******************************************************************
* LEDS / Switches
******************************************************************/
//#define LED1_TRIS TRISDbits.TRISD3
//#define LED2_TRIS TRISDbits.TRISD4

//#define SW_TEST1_TRIS TRISDbits.TRISD1
//#define SW_TEST2_TRIS TRISDbits.TRISD2

//#define LED1 LATDbits.LATD3
//#define LED2 LATDbits.LATD4

//#define SW_TEST1 PORTDbits.RD1
//#define SW_TEST2 PORTDbits.RD2


/*******************************************************************
 * Communication channels (UART1-2, CAN, SPI)
 ******************************************************************/

// UART1: Tx -> RF3, Rx <- RF2
// UART2: Tx -> RF5, Rx <- RF4
// CAN:   Rx <- RF0, Tx -> RF1
// I2C:   SDA - RG3, SCL - RG2 (J11 terminal, for SRF08 sonars)
// are all "Module controlled", no need to use TRISx/LATx

/*******************************************************************
 * ANALOG INPUTS
 ******************************************************************/
 
// Analog pin PCFG (Port ConFiGuration)
// 1 - Digital, 0 - Analog
//#define AN8_PCFG ADPCFGbits.PCFG8

 
/*******************************************************************
 * DIGITAL I/Os
 ******************************************************************/
 


#endif
