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
 *    Filename:       globals.c                                       *
 *    Date:           21/08/2011                                      *
 *    File Version:   0.9                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the definition of global variables.
 *
 **********************************************************************/

#include "sys_hw.h" //defines DEVELOP_MODE
#include "extern_globals.h"

// FOR ADC samplings 
volatile uint16_t ADResult[8];
volatile int16_t mcurrent1,mcurrent2,mcurrent1_filt,mcurrent2_filt;
volatile int16_t rcurrent1 = 0;
volatile int16_t rcurrent2 = 0;
volatile int16_t rcurrent1_req = 0;
volatile int16_t rcurrent2_req = 0;
int16_t mcurrent1_offset = 10;
int16_t mcurrent2_offset = 11;

// Current/velocity limits
int16_t max_current;
int16_t max_velocity;
int16_t max_velocity_scaled;

// For BASIC real-time scheduling
// As these are incremented in PWM ISR they are declared as volatile (???)
volatile uint16_t slow_event_count=0;
volatile uint16_t medium_event_count=0;
uint16_t slow_ticks_limit;
uint16_t medium_ticks_limit;

// for switches management
t_push_buttons push_buttons_state[5];

// flags
t_control_flags control_flags;
t_status_flags status_flags;
t_control_mode control_mode;
t_direction_flags direction_flags;
uint16_t direction_flags_prev;

// FOR POSITION feedback
volatile int16_t mvelocity1,mvelocity2,rvelocity1,rvelocity2;
volatile int32_t mposition1,mposition2;

// FOR ODOMETRY estimate
int32_t x_odom,y_odom,theta_odom;

int32_t encoder_counts_rev;// = 86000; //TODO init with EEPROM
int16_t wheel_diam;// = 500; // in 0.1mm
//int16_t wheel_diam = 50; // in mm
int16_t wheel_track;// = 4100;  // in 0.1mm
//int16_t wheel_track = 410; // in mm
int16_t robot_mass; // e.g. 15 Kg, NOW scaled in grams 
int16_t robot_inertia; // = 41; // in 0.01 Kg m^2 ?? NOW in Kg cm^2
int32_t ADC_torque_scale; // = 42000000; // = 25 600 000 000 / 438
                    // since 438 is scaling ADC points / Nm
                    // and Torque ref is in Nm * 10^-8 23.8 fixed-point
int32_t odom_left_corr; // Used as a XX / 10000 scaling factor for left wheel
                        // traveling distance.

#ifdef DEVELOP_MODE 
// DATALOG buffers
#ifdef LOG_LONG
int32_t dataLOG1[MAXLOG];
int32_t dataLOG2[MAXLOG];
#else
int16_t dataLOG1[MAXLOG];
int16_t dataLOG2[MAXLOG];
int16_t dataLOG3[MAXLOG];
int16_t dataLOG4[MAXLOG];
#endif

uint16_t dataLOGIdx;
uint8_t dataLOGdecim;
#endif
