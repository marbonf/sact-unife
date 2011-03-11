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
 *    Filename:       extern_globals.h 	                              *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *	Code Description
 *  
 *  This file contains the extern declarations for global variables.
 *
 **********************************************************************/
#ifndef EXTERN_GLOBALS_H
#define EXTERN_GLOBALS_H 

#include "PID.h" //for PID parm/flags data-types
#include "Trajectories.h" //for Trajectory gen. parm/flags types

// FOR ADC samplings 
extern volatile unsigned int ADResult[8];
extern volatile int mcurrent1,mcurrent2,mcurrent1_filt,mcurrent2_filt;
extern volatile int rcurrent1, rcurrent2,rcurrent1_req,rcurrent2_req;
extern int mcurrent1_offset,mcurrent2_offset;

// Current limit
extern int max_current;
// FOR CURRENT REFERENCES LOW-PASS FILTERING
extern int rcurrent1samp[],rcurrent2samp[];

// For BASIC real-time scheduling
extern volatile unsigned int slow_event_count;
extern volatile unsigned int medium_event_count;
extern unsigned int slow_ticks_limit;
extern unsigned int medium_ticks_limit;
#define SLOW_RATE  100 // in ms
#define MEDIUM_RATE 20 // in ms

// For push-buttons management and debounce (see medium_event_handler)
typedef union {
	struct {
	unsigned sw1 : 1;
	unsigned sw2 : 1;
	unsigned sw3 : 1;
	unsigned sw4 : 1;
	unsigned sw5 : 1;
	unsigned sw6 : 1;
	unsigned sw7 : 1;
	unsigned sw8 : 1;
	};
	unsigned char b;
} t_push_buttons;

extern t_push_buttons push_buttons_state[];

// CONTROL flags
typedef union{
	struct {
	unsigned first_scan 		 : 1;
	unsigned current_loop_active : 1;
	unsigned pos_loop_active 	 : 1;
	unsigned cart_loop_active 	 : 1;
	unsigned UNUSED : 4;
	};
	unsigned char b;
} t_control_flags;

extern t_control_flags control_flags;

// STATUS flags
typedef union{
	struct {
	unsigned battery_fault 		 : 1;
	unsigned overcurrent1        : 1;
	unsigned overcurrent2        : 1;
	unsigned track_error1        : 1;
	unsigned track_error2        : 1;
	unsigned sonar_fault		 : 1;
	unsigned bump_detect		 : 1;
	unsigned comm_error 		 : 1;
	};
	unsigned char b;
} t_status_flags;

extern t_status_flags status_flags;


// CONTROL MODE state and transitions
typedef struct{
	union {
	struct {
	unsigned torque_mode1_req : 1; // torque mode "independent"
	unsigned torque_mode2_req : 1; // torque mode "driving"
	unsigned vel_mode1_req    : 1; // velocity mode "independent"
	unsigned vel_mode2_req    : 1; // velocity mode "driving"
	unsigned cart_mode_req    : 1; 
	unsigned off_mode_req     : 1;
	unsigned UNUSED           : 2;
	};
	unsigned char trxs;
	};
	unsigned char state;
} t_control_mode;

// CONTROL MODE state values
#define OFF_MODE   		0
#define TORQUE_MODE1	1
#define TORQUE_MODE2	2
#define VEL_MODE1		3
#define VEL_MODE2		4
#define CART_MODE		5

extern t_control_mode control_mode;

// PID parameters and flags structures
// definitions are in the Controls.c source file
// NOT in globals.c
extern tPIDParm PIDCurrent1;
extern tPIDParm PIDCurrent2;
extern tPIDParm PIDSpeed1;
extern tPIDParm PIDSpeed2;
extern tPIDParm PIDPos1;
extern tPIDParm PIDPos2;

extern tPIDflags PIDCurrent1_f;
extern tPIDflags PIDCurrent2_f;
extern tPIDflags PIDSpeed1_f;
extern tPIDflags PIDSpeed2_f;
extern tPIDflags PIDPos1_f;
extern tPIDflags PIDPos2_f;

// Trajectory parameters and flags structures
// definitions are in the Controls.c source file
// NOT in globals.c
extern tTRAJParm TRAJMotor1;
extern tTRAJParm TRAJMotor2;

extern tTRAJflags TRAJMotor1_f;
extern tTRAJflags TRAJMotor2_f;

// FOR POSITION feedback
extern volatile int mvelocity1,mvelocity2,rvelocity1,rvelocity2;
extern volatile long mposition1,mposition2;

// FOR ODOMETRY estimate
extern long x_odom,y_odom,theta_odom;

// NONLINEAR FILTER structures
// definitions are in the Controls.c source file
// NOT in globals.c
#define TURNING  0
#define SLOWING	 1
#define ALIGNED  2
#define STOPPING 3
extern unsigned char NLFState;
extern tNLFStatus VelocityNLFStatus;
extern tNLFStatus OrientationNLFStatus;

extern tNLFOut VelocityNLFOut;
extern tNLFOut OrientationNLFOut;

// FINAL OUTPUTS OF THE NONLINEAR FILTER
// definitions are in the Controls.c source file
// NOT in globals.c
extern long x_target,y_target, x_set, y_set; 
// PATH WAYPOINTS
#define MAX_WAY 10
extern int x_way[];
extern int y_way[];
extern int r_way[];
extern unsigned char way_index;

// SYSTEM-WIDE PARAMETERS
// defined in SACT_Protocol.c,
// stored in EEPROM, can be updated by
// the user with SACT commands
extern volatile unsigned int parameters_RAM[];

// VARIABLES FOR RUN-TIME USE OF PARAMETERS
extern long encoder_counts_rev;
extern int wheel_radius;
extern int wheel_base;
extern int robot_mass;
extern int robot_inertia;
extern long ADC_torque_scale;

#ifdef DEVELOP_MODE 
// DATALOG buffers
#define MAXLOG 400
#define LOGDECIM 100
#ifdef LOG_LONG
extern long dataLOG1[];
extern long dataLOG2[];
#else
extern int dataLOG1[];
extern int dataLOG2[];
extern int dataLOG3[];
extern int dataLOG4[];
#endif
extern unsigned int dataLOGIdx;
extern unsigned char dataLOGdecim;
#endif

#endif

