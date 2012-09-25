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


/************************************************
 * LOCAL FUNCTIONS
 ***********************************************/
void UpdateEncoder1(void);
void UpdateEncoder2(void);


/***********************************************
 * LOCAL VARIABLES
 ***********************************************/
// Temps for odometry calc.
int16_t PosCount,UpCount,DownCount;

// TO MANAGE LMD18200 SIGN INVERSION
#define BLANKS 2
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
    //rcurrent1 = (rcurrent_temp >> RCURR_MAV_SHIFT);
    rcurrent1 = RSH(rcurrent_temp, RCURR_MAV_SHIFT);
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
    //rcurrent2 = (rcurrent_temp >> RCURR_MAV_SHIFT);
    rcurrent2 = RSH(rcurrent_temp, RCURR_MAV_SHIFT);
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
 * Jogging control loops
 * for both motors
 *************************************/
void JoggingLoops(void)
{
// FIRST MOTOR    
    if(TRAJMotor1_f.enable && !TRAJMotor1_f.active)
        TRAJMotor1.qdPosition = mposition1;
    
    JogTRAJ(&TRAJMotor1, &TRAJMotor1_f);

    PIDPos1.qdInMeas = mposition1;
    PIDPos1.qdInRef  = TRAJMotor1.qdPosition;
    CalcP(&PIDPos1, &PIDPos1_f);
    
    //rvelocity1 = PIDPos1.qOut + (TRAJMotor1.qdVelocity.i[1] >> (TRAJMotor1.qVELshift + SPEED_POS_SHIFT));
    //rvelocity1 = PIDPos1.qOut + (TRAJMotor1.qdVelocity.i[1] >> TRAJMotor1.qVELshift);
    rvelocity1 = PIDPos1.qOut + RSH(TRAJMotor1.qdVelocity.i[1], TRAJMotor1.qVELshift);
    
// SECOND MOTOR
    if(TRAJMotor2_f.enable && !TRAJMotor2_f.active)
        TRAJMotor2.qdPosition = mposition2;

    JogTRAJ(&TRAJMotor2, &TRAJMotor2_f);

    PIDPos2.qdInMeas = mposition2;
    PIDPos2.qdInRef  = TRAJMotor2.qdPosition;
    CalcP(&PIDPos2, &PIDPos2_f);
    
    //rvelocity2 = PIDPos2.qOut + (TRAJMotor2.qdVelocity.i[1] >> (TRAJMotor2.qVELshift + SPEED_POS_SHIFT));
    //rvelocity2 = PIDPos2.qOut + (TRAJMotor2.qdVelocity.i[1] >> TRAJMotor2.qVELshift);
    rvelocity2 = PIDPos2.qOut + RSH(TRAJMotor2.qdVelocity.i[1], TRAJMotor2.qVELshift);


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
 * Position control loops
 * for both motors
 *************************************/
void PositionLoops(void)
{
// FIRST MOTOR    
    if(TRAJMotor1_f.enable && !TRAJMotor1_f.active)
        TRAJMotor1.qdPosition = mposition1;
    
    PosTRAJ(&TRAJMotor1, &TRAJMotor1_f); //trapezoidal motion

    if(TRAJMotor1_f.done)       //motion completed, back exec=0
       TRAJMotor1_f.exec = 0;


    PIDPos1.qdInMeas = mposition1;
    PIDPos1.qdInRef  = TRAJMotor1.qdPosition;
    CalcP(&PIDPos1, &PIDPos1_f);
    
    //rvelocity1 = PIDPos1.qOut + (TRAJMotor1.qdVelocity.i[1] >> (TRAJMotor1.qVELshift + SPEED_POS_SHIFT));
    if(TRAJMotor1_f.neg_move)
        rvelocity1 = PIDPos1.qOut - RSH(TRAJMotor1.qdVelocity.i[1], TRAJMotor1.qVELshift);
    else
        rvelocity1 = PIDPos1.qOut + RSH(TRAJMotor1.qdVelocity.i[1], TRAJMotor1.qVELshift);

// SECOND MOTOR
    if(TRAJMotor2_f.enable && !TRAJMotor2_f.active)
        TRAJMotor2.qdPosition = mposition2;

    PosTRAJ(&TRAJMotor2, &TRAJMotor2_f);

    if(TRAJMotor2_f.done)
        TRAJMotor2_f.exec = 0;

    PIDPos2.qdInMeas = mposition2;
    PIDPos2.qdInRef  = TRAJMotor2.qdPosition;
    CalcP(&PIDPos2, &PIDPos2_f);
    
    //rvelocity2 = PIDPos2.qOut + (TRAJMotor2.qdVelocity.i[1] >> (TRAJMotor2.qVELshift + SPEED_POS_SHIFT));
    if(TRAJMotor2_f.neg_move)
        rvelocity2 = PIDPos2.qOut - RSH(TRAJMotor2.qdVelocity.i[1], TRAJMotor2.qVELshift);
    else
        rvelocity2 = PIDPos2.qOut + RSH(TRAJMotor2.qdVelocity.i[1], TRAJMotor2.qVELshift);


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
if(direction_flags.simulate_1)
  {
    mvelocity1 = rvelocity1;
    mposition1 = TRAJMotor1.qdPosition;
  }
else
  {
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
  }// END ELSE of IF SIMULATE
}

void UpdateEncoder2(void)
{
if(direction_flags.simulate_2)
  {
    mvelocity2 = rvelocity2;
    mposition2 = TRAJMotor2.qdPosition;
  }
else
  {
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
  }// END ELSE of IF SIMULATE
}

