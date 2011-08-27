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
 *    Filename:       Controls.c                                      *
 *    Date:           21/08/2011                                      *
 *    File Version:   0.9                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the control loops functions.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "extern_globals.h"
#include "Controls.h"
#include "Timers.h"
#include "Trajectories.h"
#include "WayPointQ.h"
#include "PID.h"
#include "PWM.h"

#include "my_fractmath.h" // for all math stuffs

/********************************************
 * GLOBAL VARIABLES
 *******************************************/
// PID parameters and flags structures
tPIDParm PIDCurrent1;
tPIDParm PIDCurrent2;
tPIDParm PIDSpeed1;
tPIDParm PIDSpeed2;
tPIDParm PIDPos1;
tPIDParm PIDPos2;

tPIDflags PIDCurrent1_f;
tPIDflags PIDCurrent2_f;
tPIDflags PIDSpeed1_f;
tPIDflags PIDSpeed2_f;
tPIDflags PIDPos1_f;
tPIDflags PIDPos2_f;

// TRAJ parameters and flags structures
tTRAJParm TRAJMotor1;
tTRAJParm TRAJMotor2;

tTRAJflags TRAJMotor1_f;
tTRAJflags TRAJMotor2_f;

// NONLINEAR FILTER structures
tNLFStatus VelocityNLFStatus;
tNLFStatus OrientationNLFStatus;

tNLFOut VelocityNLFOut;
tNLFOut OrientationNLFOut;

// linear control-loop parameters
int16_t dfl_K1;// 14 // x gain
int16_t dfl_K2;// 14 // xdot gain
int16_t dfl_K3;// 10 // xddot gain

/************************************************
 * LOCAL FUNCTIONS
 ***********************************************/
void UpdateEncoder1(void);
void UpdateEncoder2(void);

// NONLINEAR FILTER for 2D trajectory smoothing
// Logic states
//#define TURNING  0
//#define SLOWING  1
//#define ALIGNED  2
//#define STOPPING 3
uint8_t NLFState;
// Switching logic function
void NLFSwitchingLogic(void);

// RESET DFL if SwitchingLogic is IDLE
void ResetCartesianLoop(void);

// DYNAMIC FEEDBACK LINEARIZATION CONTROL LOOP
// 3rd order, control inputs = driving force/steering torque
void DynamicFLControl(void);

// just a test
void TestNLFilters(void);

/***********************************************
 * LOCAL VARIABLES
 ***********************************************/
// Temps for odometry calc.
int16_t PosCount,UpCount,DownCount;

// Odometry derivatives
int16_t xdot_odom, ydot_odom; // auxiliary only for FAST RATE odometry estimate
int32_t x_odom_prev, y_odom_prev, xdot_odom_time, ydot_odom_time,
    xdot_odom_prev, ydot_odom_prev, xddot_odom, yddot_odom,
    vel_odom, theta_odom_prev, omega_odom;
// Target and NLF outputs
int32_t x_target; 
int32_t y_target;
int32_t r_target;

int32_t x_set, y_set, xdot_set, ydot_set,
     xddot_set, yddot_set, xdddot_set, ydddot_set,
     theta_set, omega_set, alpha_set, vel_set, acc_set, jerk_set;

#define R_STOP 1000000L
#define R_SLOW  962250L
#define EPS_ANGLE 600
#define EPS_DIST 2560
#define EPS_VEL 2560

// aux for derivatives
int32_t xdot_set_prev,ydot_set_prev,xddot_set_prev,yddot_set_prev,
     xdddot_set_prev,ydddot_set_prev;
// Error variables 
int32_t x_delta, y_delta, target_dist;
int32_t theta_e, omega_e, theta_e_delta;
// set-points for Vel/Orient. NL Filters
int32_t theta_ref, omega_ref, vel_ref;

// "dynamic" constraints for Vel/Orient. NL Filters
int32_t vel_BOUND = 640000; // SCALING: 0.1mm/s 23.8 fixed-point
int32_t acc_lin_BOUND = 640000; //corresponds to maximum linear acceleration
int32_t acc_rad_BOUND = 128000; // SCALING: 0.1mm/s^2 23.8 fixed-point

int32_t omega_M;
int32_t omega_BOUND = 19660; // SCALING: Q16 rad (15.16 fixed-point)

// "static" constraints for Vel/Orient. NL Filters
uint8_t Umax_vel_SHIFT = 18; // corresponds to maximum linear jerk
                          // SCALING: 0.1mm/s^3, 23.8 fixed-point
                          // theoretical value: 1000 (2560000 in 23.8 fxp)
                          // 21 = round(log2(2560000)
uint8_t Umax_theta_SHIFT = 17; // corresponds to maximum angular acceleration
                            // theoretical value (motor limit): 393216
                            // 18 = round(log2(393216))

// variables for dynamic feedback linearization
int32_t V1, V2; //virtual inputs
int32_t U1, U2; //actual inputs
int32_t XI; //state of the control integrator
int32_t drive_force, steer_torque;

// TO MANAGE LMD18200 SIGN INVERSION
#define BLANKS 5
uint8_t DIR1_PREV, DIR2_PREV, DIR1_TMP, DIR2_TMP;
uint8_t DIR1_blank_count,DIR2_blank_count;

// FOR CURRENT REFERENCES LOW-PASS FILTERING
int16_t rcurrent1samp[RCURR_MAV_ORDER],rcurrent2samp[RCURR_MAV_ORDER];
uint8_t rcurrsampIdx = 0;

/*************************************
 * Current control loops
 * for both motors
 *************************************/
void CurrentLoops(void)
{
// FIRST MOTOR
    // MANAGE SIGN OF REFERENCE (sign/magnitude control of LMD18200)
    if(rcurrent1 < 0)
    {
        PIDCurrent1.qdInRef  = (int32_t)(-rcurrent1);
        DIR1_TMP = ~direction_flags.motor1_dir;
    }
    else
    {    
        PIDCurrent1.qdInRef  = (int32_t)rcurrent1;
        DIR1_TMP = direction_flags.motor1_dir;
    }
    
    // Manage sign inversion
    if(DIR1_PREV != DIR1_TMP)
    {
        InitPID(&PIDCurrent1, &PIDCurrent1_f,-1);
        DIR1_blank_count = BLANKS;
    }    

    if(DIR1_blank_count == 0)
    {
        PIDCurrent1.qdInMeas = (int32_t)(mcurrent1_filt - mcurrent1_offset);
        //PIDCurrent1.qdInMeas = (int32_t)(mcurrent1 - mcurrent1_offset);
    
        CalcPI(&PIDCurrent1, &PIDCurrent1_f);
    }
    else
        DIR1_blank_count--;
    
    DIR1_PREV = DIR1_TMP;
      
// SECOND MOTOR
    // MANAGE SIGN OF REFERENCE (sign/magnitude control of LMD18200)
    // 2ND MOTOR HAS SIGN INVERTED ('cause of Faulhaber motor)
    if(rcurrent2 < 0)
    {
        PIDCurrent2.qdInRef  = (int32_t)(-rcurrent2);
        DIR2_TMP = ~direction_flags.motor2_dir;
    }
    else
    {    
        PIDCurrent2.qdInRef  = (int32_t)rcurrent2;
        DIR2_TMP = direction_flags.motor2_dir;
    }
    
    // Manage sign inversion
    if(DIR2_PREV != DIR2_TMP)
    {
        InitPID(&PIDCurrent2, &PIDCurrent2_f,-1);
        DIR2_blank_count = BLANKS;
    }
    
    if(DIR2_blank_count == 0)
    {
        PIDCurrent2.qdInMeas = (int32_t)(mcurrent2_filt - mcurrent2_offset);
        //PIDCurrent2.qdInMeas = (int32_t)(mcurrent2 - mcurrent2_offset);
    
        CalcPI(&PIDCurrent2, &PIDCurrent2_f);
    }
    else
        DIR2_blank_count--;
    
    DIR2_PREV = DIR2_TMP;
      
// IMPORTANT: INVERTED FIRING!!
    DIR1 = DIR1_TMP;
    DIR2 = DIR2_TMP;
    PDC1 = FULL_DUTY - (PIDCurrent1.qOut + ZERO_DUTY);
    PDC2 = FULL_DUTY - (PIDCurrent2.qOut + ZERO_DUTY);
}

/*************************************
 * Low-pass filter current references
 * if TORQUE_MODE is active
 *************************************/
void RefCurrentFilter(void)
{
int16_t rcurrent_temp;
uint8_t idxtemp;

    rcurrent1samp[rcurrsampIdx] = rcurrent1_req;
    rcurrent2samp[rcurrsampIdx] = rcurrent2_req;
    
    //update index
    rcurrsampIdx++;
    if(rcurrsampIdx > (RCURR_MAV_ORDER - 1)) rcurrsampIdx = 0;
    
    rcurrent_temp = 0;
    idxtemp = 0;
    while(idxtemp < RCURR_MAV_ORDER)
    {
        rcurrent_temp += rcurrent1samp[idxtemp];
        idxtemp++;
    }
    //DISABLE the A/D interrupt
    IEC0bits.ADIE = 0;
    rcurrent1 = (rcurrent_temp >> RCURR_MAV_SHIFT);
    //RE-ENABLE the A/D interrupt
    IEC0bits.ADIE = 1;
    
    rcurrent_temp = 0;
    idxtemp = 0;
    while(idxtemp < RCURR_MAV_ORDER)
    {
        rcurrent_temp += rcurrent2samp[idxtemp];
        idxtemp++;
    }
    //DISABLE the A/D interrupt
    IEC0bits.ADIE = 0;
    rcurrent2 = (rcurrent_temp >> RCURR_MAV_SHIFT);
    //RE-ENABLE the A/D interrupt
    IEC0bits.ADIE = 1;

}


/*************************************
 * Speed control loops
 * for both motors
 *************************************/
void SpeedLoops(void)
{
// FIRST MOTOR
    //UpdateEncoder1();    

    PIDSpeed1.qdInMeas = (int32_t)mvelocity1;
    PIDSpeed1.qdInRef  = (int32_t)rvelocity1;
    CalcPI(&PIDSpeed1, &PIDSpeed1_f);
    
    rcurrent1 = PIDSpeed1.qOut;
    
// SECOND MOTOR
    //UpdateEncoder2();
    
    PIDSpeed2.qdInMeas = (int32_t)mvelocity2;
    PIDSpeed2.qdInRef  = (int32_t)rvelocity2;
    CalcPI(&PIDSpeed2, &PIDSpeed2_f);
    
    rcurrent2 = PIDSpeed2.qOut;

#ifdef DEVELOP_MODE
#ifdef LOG_SPEEDLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = (int16_t)PIDSpeed1.qdInRef;
        dataLOG2[dataLOGIdx] = mvelocity1;
        dataLOG3[dataLOGIdx] = (int16_t)PIDSpeed2.qdInRef;
        dataLOG4[dataLOGIdx] = mvelocity2;
        
        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
    
    
#endif //LOG_SPEEDLOOP
#endif //DEVELOP_MODE

}// END SPEED LOOPS

/*************************************
 * Position control loops
 * for both motors
 *************************************/
void PositionLoops(void)
{
// FIRST MOTOR
    //UpdateEncoder1();
    
    if(TRAJMotor1_f.enable && !TRAJMotor1_f.active)
        TRAJMotor1.qdPosition = mposition1;
    
    JogTRAJ(&TRAJMotor1, &TRAJMotor1_f);

    PIDPos1.qdInMeas = mposition1;
    PIDPos1.qdInRef  = TRAJMotor1.qdPosition;
    CalcP(&PIDPos1, &PIDPos1_f);
    
    //rvelocity1 = PIDPos1.qOut + (TRAJMotor1.qdVelocity.i[1] >> (TRAJMotor1.qVELshift + SPEED_POS_SHIFT));
    rvelocity1 = PIDPos1.qOut + (TRAJMotor1.qdVelocity.i[1] >> TRAJMotor1.qVELshift);
    
// SECOND MOTOR
    //UpdateEncoder2();

    if(TRAJMotor2_f.enable && !TRAJMotor2_f.active)
        TRAJMotor2.qdPosition = mposition2;

    JogTRAJ(&TRAJMotor2, &TRAJMotor2_f);

    PIDPos2.qdInMeas = mposition2;
    PIDPos2.qdInRef  = TRAJMotor2.qdPosition;
    CalcP(&PIDPos2, &PIDPos2_f);
    
    //rvelocity2 = PIDPos2.qOut + (TRAJMotor2.qdVelocity.i[1] >> (TRAJMotor2.qVELshift + SPEED_POS_SHIFT));
    rvelocity2 = PIDPos2.qOut + (TRAJMotor2.qdVelocity.i[1] >> TRAJMotor2.qVELshift);


#ifdef DEVELOP_MODE
#ifdef LOG_POSLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        //dataLOG1[dataLOGIdx] = (int16_t)mposition1;
        //dataLOG2[dataLOGIdx] = (int16_t)TRAJMotor1.qdPosition;
        dataLOG1[dataLOGIdx] =  x_odom;
        dataLOG2[dataLOGIdx] =  y_odom;
        dataLOG3[dataLOGIdx] =  theta_odom;
        dataLOG4[dataLOGIdx] =  xdot_odom;
             
        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
    
#endif //LOG_POSLOOP
#endif //DEVELOP_MODE

}// END POSITION LOOPS

/*************************************
 * Functions to update encoder counts
 * for both motors
 *************************************/
void UpdateEncoder1(void)
{
#ifdef SIMULATE
    if(DIR1)    
        mvelocity1 -= (mcurrent1_filt - mcurrent1_offset) >> 4;
    else
        mvelocity1 += (mcurrent1_filt - mcurrent1_offset) >> 4;
    if(mvelocity1 > 550)
        mvelocity1 = 550;
    else if(mvelocity1 < -550)
        mvelocity1 = -550;
    mposition1 += (int32_t)mvelocity1;
#endif

#ifdef REV2_BOARD
    mvelocity1 = PosCount;                            
    PosCount = POSCNT;
    mvelocity1 -= PosCount;
    mposition1 += (int32_t)mvelocity1;
#endif

#ifdef REV1_BOARD
    mvelocity1 = DownCount;
    mvelocity1 -=UpCount;
    //DownCount=TMR1;
    //UpCount=TMR4;
    if(direction_flags.encoder1_chB_lead)
    {
        DownCount=TMR4;
        UpCount=TMR1;
    }
    else
    {
        DownCount=TMR1;
        UpCount=TMR4;
    }
    mvelocity1+=UpCount;
    mvelocity1-=DownCount;
      
    mposition1+=(int32_t)mvelocity1;
#endif

}

void UpdateEncoder2(void)
{
#ifdef SIMULATE
    if(DIR2)    
        mvelocity2 += (mcurrent2_filt - mcurrent2_offset) >> 4;
    else
        mvelocity2 -= (mcurrent2_filt - mcurrent2_offset) >> 4;
    if(mvelocity2 > 550)
        mvelocity2 = 550;
    else if(mvelocity2 < -550)
        mvelocity2 = -550;
    mposition2 += (int32_t)mvelocity2;
#endif

#ifdef REV1_BOARD
    mvelocity2 = PosCount;                            
    PosCount = POSCNT;
    mvelocity2 -= PosCount;
    mposition2 += (int32_t)mvelocity2;
#endif

#ifdef REV2_BOARD
    mvelocity2 = DownCount;
    mvelocity2 -=UpCount;
    //DownCount=TMR4;
    //UpCount=TMR1;
    if(direction_flags.encoder2_chB_lead)
    {
        DownCount=TMR4;
        UpCount=TMR1;
    }
    else
    {
        DownCount=TMR1;
        UpCount=TMR4;
    }
    mvelocity2+=UpCount;    
    mvelocity2-=DownCount;
      
    mposition2+=(int32_t)mvelocity2;
#endif
}

/********************************************
 * Functions to update odometry
 * estimate from encoder counts
 *******************************************/
void UpdateOdometryFx(void)
{
    int16_t dS, dTH;
    int32_t distR,distL,temp, tempcs;
    int64_t templong;

    UpdateEncoder1();
    UpdateEncoder2();
    
////ESTIMATE TRAVELED DISTANCE
    if(direction_flags.motor1_right_side)
    {
        distR = (int32_t)wheel_diam*(int32_t)mvelocity1;
        temp = (int32_t)wheel_diam*(int32_t)mvelocity2;
    }
    else
    {
        distR = (int32_t)wheel_diam*(int32_t)mvelocity2;
        temp = (int32_t)wheel_diam*(int32_t)mvelocity1;
    }
    templong = (int64_t)temp * odom_left_corr;
    distL = (int32_t) ( templong / 10000 );
    
    temp = distR + distL; 
    
    // MULTIPLY BY PI_Q16 to convert into linear units
    temp = temp * (PI_Q16 >> 8); // * PI_Q8 to avoid overflow
    dS = (int16_t)(temp / encoder_counts_rev); // DISTANCE SCALING: 0.1mm, Q10 (5.10)
                                               // NO NEED TO SHIFT BACK SINCE DIAM. (NOT RADIUS) is used.. 
    
////ESTIMATE ROTATION
    temp = distR - distL;
    
    // MULTIPLY BY PI_Q16
    temp = temp * (PI_Q16 >> 8); // * PI_Q8 to avoid overflow
    temp = temp / wheel_track;
    dTH = (int16_t)((temp << 4) / encoder_counts_rev); // ANGLE INCREMENT IN Q13 (no need to scale)

////UPDATE ODOMETRY ESTIMATE
    temp = theta_odom + ((int32_t)dTH << 2);    // in Q16 radians, is theta_dom (Q16) + dTH (Q13) / 2 
    tempcs = _Q16cos(temp)*dS;                  // COS in Q16 (15.16), so we have 26 fractional bits
    xdot_odom = (int16_t)(tempcs >> 18);        // Xdot FINAL SCALING: 0.1mm/cycle 7.8 fixed-point
    x_odom += ((int32_t)xdot_odom);             // X FINAL SCALING: 0.1mm, 23.8 fixed-point
    tempcs = _Q16sin(temp)*dS;                  // SIN in Q16 (15.16), so we have 26 fractional bits
    ydot_odom = (int16_t)(tempcs >> 18);        // Ydot FINAL SCALING: 0.1mm/cycle 7.8 fixed-point
    y_odom += ((int32_t)ydot_odom);             // Y FINAL SCALING: 0.1mm, 23.8 fixed-point    
    
    theta_odom += ((int32_t)dTH << 3);          // FINAL UNITS: Q16 radians
    // KEEP ANGLE  between [-PI;PI]
    while(theta_odom < -PI_Q16)
        {theta_odom +=  (PI_Q16<<1);}
        
    while(theta_odom > PI_Q16)
        {theta_odom -=  (PI_Q16<<1);}
}// END UpdateOdometryFx


/*************************************
 * Intialization of 
 * Cartesian control loop based on
 * Nonlinear filter smoothing AND
 * Dynamic Feedback Linearization
 *************************************/
void InitCartesianLoop(void)
{
    int16_t tempx, tempy;
    
////INIT NLFILTERS
    InitNLFilter2Fx(&VelocityNLFOut, &VelocityNLFStatus);
    InitNLFilter2Fx(&OrientationNLFOut, &OrientationNLFStatus);
    OrientationNLFStatus.qdXint = theta_odom;
    NLFState = IDLE;
    
    // RESET TARGET POINT
    WayPointQ_Reset();
    tempx = (int16_t)((x_odom >> 8) / 10);
    tempy = (int16_t)((y_odom >> 8) / 10);
    WayPointQ_Put(tempx,tempy,(EPS_DIST>>8)/10);
    x_target = x_odom;
    y_target = y_odom;
    r_target = EPS_DIST;
    
    // RESET FILTER SET POINT
    x_set = x_odom;
    y_set = y_odom;
    theta_set = theta_odom;

////INIT D.F.L.
    XI = 0;
    
    x_odom_prev = x_odom;
    y_odom_prev = y_odom;
    theta_odom_prev = theta_odom;
    xdot_odom_prev = 0;
    ydot_odom_prev = 0;
                                       
}

/*************************************
 * Reset Dynamic Feedback Linearization
 *************************************/
void ResetCartesianLoop(void)
{
    
////INIT NLFILTERS
    InitNLFilter2Fx(&VelocityNLFOut, &VelocityNLFStatus);
    InitNLFilter2Fx(&OrientationNLFOut, &OrientationNLFStatus);
    OrientationNLFStatus.qdXint = theta_odom;
    
    // RESET TARGET POINT
    x_target = x_odom;
    y_target = y_odom;
    r_target = EPS_DIST;
    
    // RESET FILTER SET POINT
    x_set = x_odom;
    y_set = y_odom;
    theta_set = theta_odom;

////INIT D.F.L.
    //XI = 0;
    
    x_odom_prev = x_odom;
    y_odom_prev = y_odom;
    xdot_odom_prev = 0;
    ydot_odom_prev = 0;
                                       
}

/*************************************
 * Cartesian control loop based on
 * Nonlinear filter smoothing AND
 * Dynamic Feedback Linearization
 *************************************/
void CartesianLoop(void)
{

#ifdef DEVELOP_MODE     
// FOR TEST PROBE
J10Pin3_OUT = 1;
#endif

    NLFSwitchingLogic();

#ifdef DEVELOP_MODE     
// FOR TEST PROBE
J10Pin3_OUT = 0;
#endif

    
    DynamicFLControl();

#ifdef DEVELOP_MODE
#ifdef LOG_CARTLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = vel_set;
        dataLOG2[dataLOGIdx] = acc_set;
      
        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
    
#endif //LOG_POSLOOP
#endif //DEVELOP_MODE

}


/***********************************************************
 * TRAJECTORY GENERATOR WITH 2D SMOOTHING FILTER
 ***********************************************************/
 void NLFSwitchingLogic(void)
 {
    int64_t templong, stemplong, ctemplong, ftemplong;
    int32_t sintemp,costemp;
    int16_t tempx, tempy, tempr;
    
    // Distance from target
    x_delta = x_target - x_set;
    y_delta = y_target - y_set;
    templong = (int64_t)x_delta *(int64_t) x_delta;
    templong += (int64_t)y_delta * (int64_t)y_delta;
    
    templong >>= 16;
   
    target_dist = iSqrt((uint32_t)templong) << 8;
    
    if(target_dist <= (R_SLOW + r_target))
    {
        if(WayPointQ_Get(&tempx, &tempy, &tempr))
        {
            x_target = ((int32_t)tempx * 10 ) << 8; // 0.1 mm 23.8 fixed-point
            y_target = ((int32_t)tempy * 10 ) << 8; // 0.1 mm 23.8 fixed-point
            r_target = ((int32_t)tempr * 10 ) << 8; // 0.1 mm 23.8 fixed-point
            
            // RECALCULATE error and distance
            x_delta = x_target - x_set;
            y_delta = y_target - y_set;
            templong = (int64_t)x_delta *(int64_t) x_delta;
            templong += (int64_t)y_delta * (int64_t)y_delta;
    
            templong >>= 16;
   
            target_dist = iSqrt((uint32_t)templong) << 8;
        }
    }
    
////Pursuit-evasion equations:
    theta_e = _Q16atan2(x_delta,y_delta);
     
    theta_e_delta = theta_e - theta_set;

    // KEEP ANGLE error between [-PI;PI]
    while(theta_e_delta < -PI_Q16)
        {theta_e_delta +=  (PI_Q16<<1);}
        
    while(theta_e_delta > PI_Q16)
        {theta_e_delta -=  (PI_Q16<<1);}
    
    sintemp = _Q16sin(theta_e_delta);
    templong = (int64_t)vel_set * (int64_t) sintemp;
    if(target_dist != 0)
        omega_e = templong / target_dist; // assuming theta target = 0
                                          // NOTE: is scaled as Q16 already..
    else
        omega_e = 0;
     
    // MAIN STATE MACHINE LOGIC
    switch(NLFState)
    {
        case IDLE    :  
                        theta_ref = theta_odom;
                        omega_ref = 0;
                        
                        vel_ref = 0;
                        omega_M = 0;
                        
                        //InitNLFilter2Fx(&VelocityNLFOut, &VelocityNLFStatus);
                        
                        if(target_dist > R_SLOW ) 
                        {
                            NLFState = TURNING;
                        }
                        
                        break;
        case TURNING : 
                        theta_ref = theta_e;
                        omega_ref = omega_e;
                        // Fixed-point Division: 23.8 << 8 becomes 15.16
                        // 15.16 / 15.16 = integer (16-16 = 0 fractional bits)
                        // result << 8 = 23.8 again
                        vel_ref = ((acc_rad_BOUND << 8) / omega_BOUND) << 8;
                        omega_M = omega_BOUND;
                        
                        // 2000 is about 1,5 ° in Q16 rad
                        if((FxAbs(theta_e_delta) < EPS_ANGLE) && (FxAbs(omega_set) < EPS_ANGLE) )
                        {
                            NLFState = ALIGNED;
                        }
                        
                        break;
         
        case SLOWING : 
                        theta_ref = theta_set;
                        omega_ref = 0;
                        
                        vel_ref = ((acc_rad_BOUND << 8) / omega_BOUND) << 8;
                        omega_M = 0;
                        
                        sintemp = vel_set - (((acc_rad_BOUND << 8) / omega_BOUND) << 8);
                        // 2000 is about 1,5 deg. in Q16 rad
                        if(FxAbs(sintemp) < EPS_VEL)
                        {
                            NLFState = TURNING;
                        }
                        
                        break;
         
        case ALIGNED : 
                        theta_ref = theta_e;
                        omega_ref = omega_e;
                        
                        vel_ref = vel_BOUND;
                        // Fixed-point Division: 23.8 << 8 becomes 15.16
                        // 15.16 / 23.8 = 23.8 (16-8 = 8 fractional bits)
                        // result << 8 = Q16 rad
                        omega_M = ((acc_rad_BOUND << 8) / vel_BOUND) << 8;
                        
                        
                        if( target_dist <= R_STOP)
                        {
                            NLFState = STOPPING;
                        }
                        // 10000 is about 5 deg. in Q16 rad
                        else if(FxAbs(theta_e_delta) > (EPS_ANGLE<<1))
                            {
                                NLFState = SLOWING;
                            }

                        break;
         
        case STOPPING :
                        theta_ref = theta_e;
                        omega_ref = omega_e;
                        
                        vel_ref = 0;
                        //omega_M = 0;
                        omega_M = ((acc_rad_BOUND << 8) / vel_BOUND) << 8;

                        // 10000 is about 5 deg. in Q16 rad
                        if(target_dist > R_STOP + EPS_DIST)
                        {
                            if(FxAbs(theta_e_delta) > (EPS_ANGLE<<1))
                            {
                                if( vel_set <= (((acc_rad_BOUND << 8) / omega_BOUND) << 8) )
                                    NLFState = TURNING;
                                else
                                    NLFState = SLOWING;
                            }
                            else
                                NLFState = ALIGNED;
                        }
                        else if(target_dist < (EPS_DIST << 2))
                            {
                               NLFState = IDLE;
                               ResetCartesianLoop();
                            }
                        
                        break;
         
        default : break;
    } // END switch NLFState

    VelocityNLFStatus.MODE = 1;
    VelocityNLFStatus.qdRcommand = vel_ref;
    NLFilter2Fx(&VelocityNLFOut, &VelocityNLFStatus, acc_lin_BOUND, Umax_vel_SHIFT, POS_LOOP_FcSHIFT);

    if((VelocityNLFOut.qdX < EPS_VEL) && (VelocityNLFOut.qdXdot == 0) && (VelocityNLFOut.qdXddot == 0))
    {
        VelocityNLFStatus.qdXint = 0;
        VelocityNLFOut.qdX = 0;
    }
    
    OrientationNLFStatus.MODE = 2;
    OrientationNLFStatus.qdRcommand =
        OrientationNLFStatus.qdXint -
        theta_ref;
    // KEEP ANGLE error between [-PI;PI]
    while(OrientationNLFStatus.qdRcommand < -PI_Q16)
        {OrientationNLFStatus.qdRcommand +=  (PI_Q16<<1);}
        
    while(OrientationNLFStatus.qdRcommand > PI_Q16)
        {OrientationNLFStatus.qdRcommand -=  (PI_Q16<<1);}

    OrientationNLFStatus.qdRprev = omega_ref; // ANGULAR velocity reference

    NLFilter2Fx(&OrientationNLFOut, &OrientationNLFStatus, omega_M, Umax_theta_SHIFT , POS_LOOP_FcSHIFT );

    // KEEP ANGLE output between [-PI;PI]
    while(OrientationNLFStatus.qdXint < -PI_Q16)
        {OrientationNLFStatus.qdXint +=  (PI_Q16<<1);}
        
    while(OrientationNLFStatus.qdXint > PI_Q16)
        {OrientationNLFStatus.qdXint -=  (PI_Q16<<1);}

    OrientationNLFOut.qdX = OrientationNLFStatus.qdXint;
    
    vel_set = VelocityNLFOut.qdX;
    acc_set = VelocityNLFOut.qdXdot;
    jerk_set = VelocityNLFOut.qdXddot;

    theta_set = OrientationNLFOut.qdX;
    omega_set = OrientationNLFOut.qdXdot;
    alpha_set = OrientationNLFOut.qdXddot;
    
    sintemp = _Q16sin(theta_set);
    costemp = _Q16cos(theta_set);
    
    templong = (int64_t) vel_set * (int64_t) costemp;
    xdot_set = templong >> 16;
    templong = (int64_t) vel_set * (int64_t) sintemp;
    ydot_set = templong >> 16;
    
    templong = (int64_t) vel_set * (int64_t) omega_set;
    templong >>= 16;
    stemplong = - templong * (int64_t) sintemp;
    stemplong >>= 16;
    stemplong += ((int64_t) acc_set * (int64_t) costemp) >> 16;
    xddot_set = (int32_t) stemplong;
    //xddot_set = acc_set * costemp - vel_set * omega_set * sintemp;
    stemplong = templong * (int64_t) costemp;
    stemplong >>= 16;
    stemplong += ((int64_t) acc_set * (int64_t) sintemp) >> 16;
    yddot_set = (int32_t) stemplong;
    //yddot_set = acc_set * sintemp + vel_set * omega_set * costemp;
    
//    xddot_set_prev = (xdot_set - xdot_set_prev) << POS_LOOP_FcSHIFT;
//    yddot_set_prev = (ydot_set - ydot_set_prev) << POS_LOOP_FcSHIFT;
//    xdot_set_prev = xdot_set;
//    ydot_set_prev = ydot_set;
    
    
    templong = templong * (int64_t) omega_set; // is already vel_set * omega_set
    templong >>= 16;
    stemplong = (int64_t)acc_set * (int64_t)omega_set;
    stemplong >>= 15; // is also *2
    ctemplong = (int64_t)vel_set * (int64_t)alpha_set;
    ctemplong >>= 16;
    ftemplong = (- templong * costemp) >> 16;
    ftemplong -= (ctemplong * sintemp) >> 16;
    ftemplong -= (stemplong * sintemp) >> 16;
    ftemplong += ((int64_t)jerk_set * (int64_t) costemp) >> 16;
    xdddot_set = (int32_t) ftemplong;

    //xdddot_set = jerk_set * costemp - 2*acc_set * omega_set * sintemp - vel_set * alpha_set * sintemp
    //             - vel_set * omega_set * omega_set * costemp;

    ftemplong = (- templong * sintemp) >> 16;
    ftemplong += (ctemplong * costemp) >> 16;
    ftemplong += (stemplong * costemp) >> 16;
    ftemplong += ((int64_t)jerk_set * (int64_t) sintemp) >> 16;
    ydddot_set = (int32_t) ftemplong;
    //ydddot_set = jerk_set * sintemp + 2*acc_set * omega_set * costemp + vel_set * alpha_set * costemp
    //             - vel_set * omega_set * omega_set * sintemp;

//    xdddot_set_prev = (xddot_set - xddot_set_prev) << POS_LOOP_FcSHIFT;
//    ydddot_set_prev = (yddot_set - yddot_set_prev) << POS_LOOP_FcSHIFT;
//    xddot_set_prev = xddot_set;
//    yddot_set_prev = yddot_set;
    
    x_set += (xdot_set >> POS_LOOP_FcSHIFT);
    y_set += (ydot_set >> POS_LOOP_FcSHIFT);
    
 }// END NLFSwitchingLogic()

/*************************************************************
 * DYNAMIC FEEDBACK LINEARIZATION CONTROL LOOP
 * 3rd order, control outputs = driving force/steering torque
 *************************************************************/
void DynamicFLControl(void)
{
    int32_t sintemp, costemp, x_err, y_err,theta_delta;
    int64_t templong, stemplong, ctemplong;
    int16_t tempcurr;

    // complete full state measurement (x/y dot TIME scaled, x/y ddot)
    xdot_odom_time = (x_odom - x_odom_prev) << POS_LOOP_FcSHIFT;
    ydot_odom_time = (y_odom - y_odom_prev) << POS_LOOP_FcSHIFT;
    x_odom_prev = x_odom;
    y_odom_prev = y_odom;

    xddot_odom = (xdot_odom_time - xdot_odom_prev) << POS_LOOP_FcSHIFT;
    yddot_odom = (ydot_odom_time - ydot_odom_prev) << POS_LOOP_FcSHIFT;
    xdot_odom_prev = xdot_odom_time;
    ydot_odom_prev = ydot_odom_time;

    templong = (int64_t) xdot_odom_time * (int64_t)xdot_odom_time;
    templong += (int64_t) ydot_odom_time * (int64_t)ydot_odom_time;
    templong >>= 16;
    vel_odom = iSqrt((uint32_t)templong) << 8;
    
    theta_delta = theta_odom - theta_odom_prev;
    // KEEP ANGLE difference between [-PI;PI]
    while(theta_delta < -PI_Q16)
        {theta_delta +=  (PI_Q16<<1);}
        
    while(theta_delta > PI_Q16)
        {theta_delta -=  (PI_Q16<<1);}
    
    omega_odom = theta_delta << POS_LOOP_FcSHIFT;
    theta_odom_prev = theta_odom;

    sintemp = _Q16sin(theta_odom);
    costemp = _Q16cos(theta_odom); 

    x_err = x_set - x_odom;
    y_err = y_set - y_odom;

    // LINEAR FEEDBACK LOOP
    templong = xdddot_set;
    templong += dfl_K3 * (int64_t)(xddot_set - xddot_odom);
    templong += dfl_K2 * (int64_t)(xdot_set - xdot_odom_time);
    templong += dfl_K1 * (int64_t)x_err;
    V1 = (int32_t) templong;
    
    templong = ydddot_set;
    templong += dfl_K3 * (int64_t)(yddot_set - yddot_odom);
    templong += dfl_K2 * (int64_t)(ydot_set - ydot_odom_time);
    templong += dfl_K1 * (int64_t)y_err;
    V2 = (int32_t) templong;

    // FEEDBACK LINEARIZATION LOOP
    // U1 = Linear Jerk = vel * omega * omega + V1 * costemp + V2 * sintemp
    // U2 = Ang. Accel. = ( - 2 * XI * omega + V2 * costemp - V1 * sintemp ) / vel
    // XI = is the state of the integrator included in the dynamic feedback loop
    //      and it integrates U1
    // ==> SO THAT the final control inputs are XI (linear accel.)
    //     and U2 (angular acceleration) which can be transformed into torque/current
    //     references for the wheels according to the differential drive kinematics
    
    templong = (int64_t) omega_odom * (int64_t)omega_odom;
    templong = templong * (int64_t) vel_odom; 
    templong >>= 16; // 16+8 frac. bits

    ctemplong = (int64_t)V1 * (int64_t) costemp;
    stemplong = (int64_t)V2 * (int64_t) sintemp;
    
    templong += ctemplong; // 16+8 frac. bits
    templong += stemplong; // 16+8 frac. bits
    
    U1 = (int32_t) (templong >> 16); // 23.8 fixed-point

    templong = - ((int64_t) XI * (int64_t)omega_odom); // 16+8 frac. bits
    templong <<= 1; // is also *2

    stemplong = (int64_t)V1 * (int64_t) sintemp;
    ctemplong = (int64_t)V2 * (int64_t) costemp;
    
    templong += ctemplong; // 16+8 frac. bits
    templong -= stemplong; // 16+8 frac. bits
    
    //if(vel_set != 0)
    if(vel_odom > (EPS_VEL << 3))
    {
        U2 = (int32_t) (templong / vel_odom); // SCALING: Q16 rad/s^2
    }
    else
    {
        U2 = 0;
    }
    
    // update the state of the integrator embedded in the dynamic feedback loop
    XI += (U1 >> POS_LOOP_FcSHIFT);
    
    // FORCE / TORQUE SCALING
    // driving force = XI * M_robot
    // steering torque = U2 * J_robot
    templong = (int64_t) XI * robot_mass;     // robot_mass is in grams
    drive_force = (int32_t)(templong / 100);  //in 0.0001 N 23.8 fixed-point
    templong = (int64_t) U2 * robot_inertia;  // robot_inertia is in Kg cm^2
    steer_torque = (int32_t)(templong / 100); //in 0.01 Nm 15.16 fixed-point
    
    // Differential-drive kinematics
    // Torque right wheel = (driving force * (wheel_radius / 2) + (steering torque * wheelradius) / (2 * wheeltrack)
    // Torque left wheel  = (driving force * (wheel_radius / 2) - (steering torque * wheelradius) / (2 * wheeltrack)
    
    stemplong = (int64_t)drive_force * (int64_t)(wheel_diam >> 2); // Nm * 10^-8 23.8
    ctemplong = (int64_t)steer_torque * (int64_t)(wheel_diam >> 2); 
    ctemplong = ctemplong / wheel_track; // Nm * 10^-2 15.16
    ctemplong >>= 8;
    ctemplong *= 1000000; // Nm * 10^-6 23.8
    
    templong = stemplong + ctemplong; // Wheel torque in Nm * 10^-8 23.8
    tempcurr = (int16_t)( templong / ADC_torque_scale);
    
    if(tempcurr > max_current)
        tempcurr = max_current;
    else if(tempcurr < -max_current)
        tempcurr = -max_current;
    rcurrent1_req = tempcurr;
    
    templong = stemplong - ctemplong;
    tempcurr = (int16_t) ( templong / ADC_torque_scale);
    if(tempcurr > max_current)
        tempcurr = max_current;
    else if(tempcurr < -max_current)
        tempcurr = -max_current;
    rcurrent2_req = tempcurr;

}// END DynamicFLControl()

