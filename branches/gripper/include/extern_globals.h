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
 *    Filename:       extern_globals.h                                *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the extern declarations for global variables.
 *
 **********************************************************************/
#ifndef EXTERN_GLOBALS_H
#define EXTERN_GLOBALS_H 

#include "generic_defs.h" // for data types
#include "PID.h" //for PID parm/flags data-types
#include "Trajectories.h" //for Trajectory gen. parm/flags types

// FOR ADC samplings 
extern volatile uint16_t ADResult[8];
extern volatile int16_t mcurrent1,mcurrent2,mcurrent1_filt,mcurrent2_filt;
extern volatile int16_t rcurrent1, rcurrent2,rcurrent1_req,rcurrent2_req;
extern int16_t mcurrent1_offset,mcurrent2_offset;

// Current/velocity limit
extern int16_t max_current;
extern int16_t max_velocity;
extern int16_t max_velocity_scaled;
extern int16_t max_angle; // in 1/10 degrees
extern int16_t min_angle; // in 1/10 degrees
// FOR CURRENT REFERENCES LOW-PASS FILTERING
extern int16_t rcurrent1samp[],rcurrent2samp[];

// For BASIC real-time scheduling
extern volatile uint16_t slow_event_count;
extern volatile uint16_t medium_event_count;
extern uint16_t slow_ticks_limit;
extern uint16_t medium_ticks_limit;
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
    uint8_t b;
} t_push_buttons;

extern t_push_buttons push_buttons_state[];

// CONTROL flags
typedef union{
    struct {
    unsigned first_scan          : 1;
    unsigned current_loop_active : 1;
    unsigned pos_loop_active     : 1;
    unsigned jog_loop_active    : 1;
    unsigned EE_update_req       : 1;
    unsigned PAR_update_req      : 1;
    unsigned UNUSED              : 2;
    };
    uint8_t b;
} t_control_flags;

extern t_control_flags control_flags;

// STATUS flags
typedef union{
    struct {
    unsigned battery_fault       : 1;
    unsigned overcurrent1        : 1;
    unsigned overcurrent2        : 1;
    unsigned track_error1        : 1;
    unsigned track_error2        : 1;
    unsigned bump_detect         : 1;
    unsigned comm_error_code     : 2;
    };
    uint8_t b;
} t_status_flags;

extern t_status_flags status_flags;

// CONTROL MODE state and transitions
typedef struct{
    union {
    struct {
    unsigned torque_mode_req : 1; // torque mode
    unsigned vel_mode_req    : 1; // velocity mode
    unsigned pos_mode_req    : 1; 
    unsigned off_mode_req     : 1;
    unsigned UNUSED           : 4;
    };
    uint8_t trxs;
    };
    uint8_t state;
} t_control_mode;

// CONTROL MODE state values
#define OFF_MODE        0
#define TORQUE_MODE     1
#define VEL_MODE        2
#define POS_MODE        3

extern t_control_mode control_mode;

// DIRECTION flags
typedef union{
    struct {
    unsigned motor1_dir          : 1;
    unsigned motor2_dir          : 1;
    unsigned encoder1_chB_lead   : 1;
    unsigned encoder2_chB_lead   : 1;
    unsigned simulate_1          : 1;
    unsigned simulate_2          : 1;
    unsigned UNUSED              : 10;
    };
    uint16_t word;
} t_direction_flags;

// GRIP STATUS flags
typedef union{
    struct {
    unsigned endstroke_pos_1     : 1; //b0
    unsigned endstroke_neg_1     : 1; //b1
    unsigned endstroke_pos_2     : 1; //b2
    unsigned endstroke_neg_2     : 1; //b3
    unsigned grip_touch_1        : 1; //b4
    unsigned grip_touch_2        : 1; //b5
    unsigned grip_traj_exec_1    : 1; //b6
    unsigned grip_traj_exec_2    : 1; //b7
    unsigned UNUSED              : 8;
    };
    uint16_t word;
} t_grip_status_flags;

extern t_grip_status_flags grip_status_flags;

extern t_direction_flags direction_flags;
extern uint16_t direction_flags_prev;

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
extern volatile int16_t mvelocity1,mvelocity2,rvelocity1,rvelocity2;
extern volatile int32_t mposition1,mposition2;

// SYSTEM-WIDE PARAMETERS
// defined in SACT_Protocol.c,
// stored in EEPROM, can be updated by
// the user with SACT commands
extern uint16_t parameters_RAM[];

// Auxiliary variables
extern uint32_t encoder_counts_rev;
extern uint16_t decdeg_to_ticks;
extern int32_t ADC_torque_scale;


#ifdef DEVELOP_MODE 
// DATALOG buffers
#define MAXLOG 400
#define LOGDECIM 100
#ifdef LOG_LONG
extern int32_t dataLOG1[];
extern int32_t dataLOG2[];
#else
extern int16_t dataLOG1[];
extern int16_t dataLOG2[];
extern int16_t dataLOG3[];
extern int16_t dataLOG4[];
#endif
extern uint16_t dataLOGIdx;
extern uint8_t dataLOGdecim;
#endif

#endif

