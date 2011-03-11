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
 *    Filename:       globals.c          	                      *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains the definition of global variables.
 *
 **********************************************************************/

#include "sys_hw.h" //defines DEVELOP_MODE
#include "extern_globals.h"

// FOR ADC samplings 
volatile unsigned int ADResult[8];
volatile int mcurrent1,mcurrent2,mcurrent1_filt,mcurrent2_filt;
volatile int rcurrent1 = 0;
volatile int rcurrent2 = 0;
volatile int rcurrent1_req = 0;
volatile int rcurrent2_req = 0;
int mcurrent1_offset = 10;
int mcurrent2_offset = 11;

// Current limit
int max_current = 800;

// For BASIC real-time scheduling
// As these are incremented in PWM ISR they are declared as volatile (???)
volatile unsigned int slow_event_count=0;
volatile unsigned int medium_event_count=0;
unsigned int slow_ticks_limit;
unsigned int medium_ticks_limit;

// for switches management
t_push_buttons push_buttons_state[5];

// flags
t_control_flags control_flags;
t_status_flags status_flags;
t_control_mode control_mode;

// FOR POSITION feedback
volatile int mvelocity1,mvelocity2,rvelocity1,rvelocity2;
volatile long mposition1,mposition2;

// FOR ODOMETRY estimate
long x_odom,y_odom,theta_odom;

long encoder_counts_rev = 86000; //TODO init with EEPROM
int wheel_radius = 500; // in 0.1mm
//int wheel_radius = 50; // in mm
int wheel_base = 4100;  // in 0.1mm
//int wheel_base = 410; // in mm
int robot_mass = 15; // in Kg
int robot_inertia = 41; // in 0.01 Kg m^2 ??
long ADC_torque_scale = 42000000; // = 25 600 000 000 / 438
					// since 438 is scaling ADC points / Nm
					// and Torque ref is in Nm * 10^-8 23.8 fixed-point

#ifdef DEVELOP_MODE 
// DATALOG buffers
#ifdef LOG_LONG
long dataLOG1[MAXLOG];
long dataLOG2[MAXLOG];
#else
int dataLOG1[MAXLOG];
int dataLOG2[MAXLOG];
int dataLOG3[MAXLOG];
int dataLOG4[MAXLOG];
#endif

unsigned int dataLOGIdx;
unsigned char dataLOGdecim;
#endif
