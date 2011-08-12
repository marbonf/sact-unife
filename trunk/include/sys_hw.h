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
 *    Date:           28/12/2010                                  *
 *    File Version:   0.1                                         *
 *    Compiler:       MPLAB C30 v3.23                             *
 *                                                                *
 ******************************************************************
 *    Code Description
 *  
 *  This file contains system and hardware specific definitions.
 *  Compatibility:
 *  - SACT board design rev.1
 *  - SACT board design rev.2
 *  - all boards with dsPIC30F6015
 *
 ********************************************************************/
#ifndef SYS_HW_H
#define SYS_HW_H
 
#include <p30fxxxx.h>

/****************************************************************
 * DEFINITIONS TO ADAPT BOARD SPECIFIC OPTIONS OR SIMULATIONS
 ***************************************************************/
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//#define SIMULATE
//#define REV2_BOARD
#define REV1_BOARD

/****************************************************************
 * IF COMPILED FOR DEVELOP MODE
 * ADDS DATALOGS AND TEST PROBE OUTPUTS FOR TASK MONITORING
 ***************************************************************/
//#define DEVELOP_MODE
//UNCOMMENT THE FOLLOWING IF LONG (data-type) DATALOG IS DESIRED
#define LOG_LONG
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//#define LOG_POSLOOP
#define LOG_CARTLOOP
//#define LOG_SPEEDLOOP
//#define LOG_ADCINT

/*******************************************************************
 * System Clock Timing -
 * For oscillator configuration XT x PLL8 mode,
 * Device Throughput in MIPS = Fcy = 7372800*8/4 = ~14.74 MIPS
 * Instruction Cycle time = Tcy = 1/(Fcy) = ~68 nanoseconds
 ******************************************************************/
#define XTFREQ          7372800UL            //On-board Crystal fcy
#define PLLMODE         16                   //On-chip PLL setting
#define FCY             XTFREQ*PLLMODE/4     //Instruction Cycle Fcy

/*******************************************************************
 * H-Bridge Control pins (PWM/DIR/BRAKE) 
 * and feedback (Thermal Flag, Current Sense)
 * for LMD18200
 ******************************************************************/
 
// PWMx pins are "Module controlled", no need to set TRISx manually!
#define PWM1_TRIS TRISEbits.TRISE0
#define PWM2_TRIS TRISEbits.TRISE2
 
// Other pins are instead standard dig.outputs..
#define DIR1_TRIS TRISEbits.TRISE1
#define DIR2_TRIS TRISEbits.TRISE3
 
#define BRAKE1_TRIS TRISEbits.TRISE4
#define BRAKE2_TRIS TRISEbits.TRISE5
 
#define THERMFLG1_TRIS TRISBbits.TRISB3
#define THERMFLG2_TRIS TRISBbits.TRISB2
 
#define CURRSENSE1_TRIS TRISBbits.TRISB1
#define CURRSENSE2_TRIS TRISBbits.TRISB0
 
// Current sense analog pin PCFG (Port ConFiGuration)
// 1 - Digital, 0 - Analog
#define CURRSENSE1_PCFG ADPCFGbits.PCFG1
#define CURRSENSE2_PCFG ADPCFGbits.PCFG0
 
// PWMx pins are controlled by duty-cycle generators, no need to use LATx
#define PWM1 LATEbits.LATE0 
#define PWM2 LATEbits.LATE2 

// Other pins are instead standard digital I/Os
#define DIR1 LATEbits.LATE1 
#define DIR2 LATEbits.LATE3 

#define BRAKE1 LATEbits.LATE4 
#define BRAKE2 LATEbits.LATE5 

#define THERMFLG1 PORTBbits.RB3 
#define THERMFLG2 PORTBbits.RB2 


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
#define LED1_TRIS TRISDbits.TRISD3
#define LED2_TRIS TRISDbits.TRISD4

#define SW_TEST1_TRIS TRISDbits.TRISD1
#define SW_TEST2_TRIS TRISDbits.TRISD2

#define LED1 LATDbits.LATD3
#define LED2 LATDbits.LATD4

#define SW_TEST1 PORTDbits.RD1
#define SW_TEST2 PORTDbits.RD2


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
 
// J13 terminal
#define AN8_TRIS TRISBbits.TRISB8
#define AN9_TRIS TRISBbits.TRISB9 
#define AN10_TRIS TRISBbits.TRISB10
#define AN11_TRIS TRISBbits.TRISB11

// J14 terminal
#define AN12_TRIS TRISBbits.TRISB12
#define AN13_TRIS TRISBbits.TRISB13
#define AN14_TRIS TRISBbits.TRISB14
#define AN15_TRIS TRISBbits.TRISB15

// Analog pin PCFG (Port ConFiGuration)
// 1 - Digital, 0 - Analog
#define AN8_PCFG ADPCFGbits.PCFG8
#define AN9_PCFG ADPCFGbits.PCFG9
#define AN10_PCFG ADPCFGbits.PCFG10
#define AN11_PCFG ADPCFGbits.PCFG11
#define AN12_PCFG ADPCFGbits.PCFG12
#define AN13_PCFG ADPCFGbits.PCFG13
#define AN14_PCFG ADPCFGbits.PCFG14
#define AN15_PCFG ADPCFGbits.PCFG15
 
 
/*******************************************************************
 * DIGITAL I/Os
 ******************************************************************/
 
// J10 terminal 
#define J10Pin1_TRIS TRISEbits.TRISE6 
#define J10Pin2_TRIS TRISEbits.TRISE7
#define J10Pin3_TRIS TRISGbits.TRISG6
#define J10Pin4_TRIS TRISGbits.TRISG7
 
// J11 terminal
#define J11Pin1_TRIS TRISGbits.TRISG8 
#define J11Pin2_TRIS TRISFbits.TRISF6
#define J11Pin3_TRIS TRISGbits.TRISG3
#define J11Pin4_TRIS TRISGbits.TRISG2
#define J11Pin5_TRIS TRISDbits.TRISD8
#define J11Pin6_TRIS TRISDbits.TRISD9

// J12 terminal
#define J12Pin1_TRIS TRISDbits.TRISD10
#define J12Pin2_TRIS TRISDbits.TRISD11
#define J12Pin3_TRIS TRISDbits.TRISD0
#define J12Pin4_TRIS TRISDbits.TRISD5
#define J12Pin5_TRIS TRISDbits.TRISD6
#define J12Pin6_TRIS TRISDbits.TRISD7

/////////////////////////////////////
// IF USED AS INPUTS, BETTER TO USE
// PORTx regs.
// J10 terminal 
#define J10Pin1_IN PORTEbits.RE6 
#define J10Pin2_IN PORTEbits.RE7
#define J10Pin3_IN PORTGbits.RG6
#define J10Pin4_IN PORTGbits.RG7
 
// J11 terminal
#define J11Pin1_IN PORTGbits.RG8 
#define J11Pin2_IN PORTFbits.RF6
#define J11Pin3_IN PORTGbits.RG3
#define J11Pin4_IN PORTGbits.RG2
#define J11Pin5_IN PORTDbits.RD8
#define J11Pin6_IN PORTDbits.RD9

// J12 terminal
#define J12Pin1_IN PORTDbits.RD10
#define J12Pin2_IN PORTDbits.RD11
#define J12Pin3_IN PORTDbits.RD0
#define J12Pin4_IN PORTDbits.RD5
#define J12Pin5_IN PORTDbits.RD6
#define J12Pin6_IN PORTDbits.RD7

/////////////////////////////////////
// IF USED AS OUTPUTS, BETTER TO USE
// LATx regs.
// J10 terminal 
#define J10Pin1_OUT LATEbits.LATE6 
#define J10Pin2_OUT LATEbits.LATE7
#define J10Pin3_OUT LATGbits.LATG6
#define J10Pin4_OUT LATGbits.LATG7
 
// J11 terminal
#define J11Pin1_OUT LATGbits.LATG8 
#define J11Pin2_OUT LATFbits.LATF6
#define J11Pin3_OUT LATGbits.LATG3
#define J11Pin4_OUT LATGbits.LATG2
#define J11Pin5_OUT LATDbits.LATD8
#define J11Pin6_OUT LATDbits.LATD9

// J12 terminal
#define J12Pin1_OUT LATDbits.LATD10
#define J12Pin2_OUT LATDbits.LATD11
#define J12Pin3_OUT LATDbits.LATD0
#define J12Pin4_OUT LATDbits.LATD5
#define J12Pin5_OUT LATDbits.LATD6
#define J12Pin6_OUT LATDbits.LATD7

#endif
