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
 *    Filename:       main.c          	                              *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 
 *	Code Description
 *  
 *  This file contains the main() function.
 *
 ********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "generic_defs.h"
#include "Comms.h"
#include "PWM.h"
#include "ADC.h"
#include "QEI.h"
#include "Timers.h"
#include "Controls.h" // for RCURR_MAV_ORDER
#include "SACT_protocol.h"

// CONFIGURATION BITS fuses (see dspic specific .h)
_FOSC(CSW_FSCM_OFF & XT_PLL16);  // Clock-switching and monitor off
								// eXTernal w/PLL8
_FWDT(WDT_OFF);                 // Turn off the Watch-Dog Timer.
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable MCLR reset pin and turn off the
                                // power-up timers.
_FGS(CODE_PROT_OFF);            // Disable Code Protection
_FBS(CODE_PROT_OFF);
_FSS(CODE_PROT_OFF);
 
// Function prototypes for "soft" real-time event handlers
void medium_event_handler(void);
void slow_event_handler(void);
void debounce_switches(void);
void control_mode_manager(void);
void diagnostics(void);

// WELCOME MESSAGE!
const unsigned char WelcomeMsg[] = 
{"\r\n-----  Sabot ACTuator Board  -----\r\n"
     "-----   HW REV.2 - FW v0.1   -----\r\n"
     "Type the following sequence\r\n"
     "SYNC0+cr/lf SYNC1+cr/lf SYNCA+cr/lf\r\n"
     "to enter ASCII mode:\r\n"};


/*******************************************************
* MAIN function, just setup some inits and loops
* "soft" real-time event handlers, defined hereafter
********************************************************/
int main(void)
{	
	control_flags.first_scan = 1;
	slow_ticks_limit = SLOW_RATE * (FCY_PWM / 1000) - 1 ;
	medium_ticks_limit = MEDIUM_RATE * (FCY_PWM / 1000) - 1;
 
 	// UARTs init
 	// no need to set TRISx, they are "Module controlled"
	UART1_Init();
	UART2_Init();
	
	// Setup control pins and PWM module,
	// which is needed also to schedule "soft"
	// real-time tasks w/PWM interrupt tick counts
	DIR1 = 0;
 	DIR2 = 1; // inverted sign, 'cause of Faulhaber motors 
 			
    BRAKE1 = 0;
    BRAKE2 = 0;
 
	DIR1_TRIS = OUTPUT;
 	DIR2_TRIS = OUTPUT;
    BRAKE1_TRIS = OUTPUT;
    BRAKE2_TRIS = OUTPUT;
    
    THERMFLG1_TRIS = INPUT;
    THERMFLG2_TRIS = INPUT;
    CURRSENSE1_TRIS = INPUT;
    CURRSENSE2_TRIS = INPUT;
    
	PWM_Init();
	
	// Setup LEDs and switches
	LED1 = 1;
 	LED2 = 0;
 	
	LED1_TRIS = OUTPUT;
 	LED2_TRIS = OUTPUT;
 	SW_TEST1_TRIS = INPUT;
    SW_TEST2_TRIS = INPUT;
    
    // MUST SETUP ALSO ANALOG PINS AS INPUTS
    AN8_TRIS = INPUT;
    AN9_TRIS = INPUT;
    AN10_TRIS = INPUT;
    AN11_TRIS = INPUT;
    AN12_TRIS = INPUT;
    AN13_TRIS = INPUT;
    AN14_TRIS = INPUT;
    AN15_TRIS = INPUT;
    
    ADC_Init();
    
    // SETUP ENCODER INPUTS, TIMERS AND QEI
    T1CK_TRIS = INPUT;
    T4CK_TRIS = INPUT;
    // QEI inputs are "module controlled"
    // -> no need to set TRISx
    QEI_Init();
    // UP/DN counters
    Timer1_Init();
    Timer4_Init();
    
    // Timer5 used to schedule speed loops
    Timer5_Init(); // AND POSITION LOOP!!!
    
///////////////////////////////////////////////////////////////////
// CONTROL LOOPS and TRAJ PLANNERS INIT
////INIT PID CURRENT 1
	PIDCurrent1.qKp = 600;
    PIDCurrent1.qKi = 80;              
    PIDCurrent1.qKd = 0;
    PIDCurrent1.qN	= 9; // SHIFT FINAL RESULT >> qN
    PIDCurrent1.qdOutMax =  (int32_t)(FULL_DUTY << (PIDCurrent1.qN-1));
    PIDCurrent1.qdOutMin = -(int32_t)(FULL_DUTY << (PIDCurrent1.qN-1));

 	InitPID(&PIDCurrent1, &PIDCurrent1_f,-1);
 	
////INIT PID CURRENT 2
    PIDCurrent2.qKp = 600;
    PIDCurrent2.qKi = 80;              
    PIDCurrent2.qKd = 0;       
    PIDCurrent2.qN	= 9; // SHIFT FINAL RESULT >> qN
 	PIDCurrent2.qdOutMax =  (int32_t)(FULL_DUTY << (PIDCurrent2.qN-1));
    PIDCurrent2.qdOutMin = -(int32_t)(FULL_DUTY << (PIDCurrent2.qN-1));

    InitPID(&PIDCurrent2, &PIDCurrent2_f,-1);		
    
////INIT PID SPEED 1
	PIDSpeed1.qKp = 1000;
    PIDSpeed1.qKi = 20;              
    PIDSpeed1.qKd = 0;
    PIDSpeed1.qN	= 9;  // SHIFT FINAL RESULT >> qN
    PIDSpeed1.qdOutMax = ((int32_t)max_current << PIDSpeed1.qN);
    PIDSpeed1.qdOutMin = -PIDSpeed1.qdOutMax;

    InitPID(&PIDSpeed1, &PIDSpeed1_f,0);
    
////INIT PID SPEED 2
	PIDSpeed2.qKp = 1000;
 	PIDSpeed2.qKi = 20;              
    PIDSpeed2.qKd = 0;
    PIDSpeed2.qN	= 9; // SHIFT FINAL RESULT >> qN
    PIDSpeed2.qdOutMax = ((int32_t)max_current << PIDSpeed2.qN);
    PIDSpeed2.qdOutMin = -PIDSpeed2.qdOutMax;

    InitPID(&PIDSpeed2, &PIDSpeed2_f,0);

////INIT PID POSITION 1
	PIDPos1.qKp = 400;
    PIDPos1.qKi = 0;              
    PIDPos1.qKd = 0;
    PIDPos1.qN	= 18;  // SHIFT FINAL RESULT >> qN
    PIDPos1.qdOutMax = ((int32_t)600 << PIDPos1.qN);
    PIDPos1.qdOutMin = -PIDPos1.qdOutMax;

    InitPID(&PIDPos1, &PIDPos1_f,0);
    
////INIT PID POSITION 2
	PIDPos2.qKp = 400;
 	PIDPos2.qKi = 0;              
    PIDPos2.qKd = 0;
    PIDPos2.qN	= 18; // SHIFT FINAL RESULT >> qN
    PIDPos2.qdOutMax = ((int32_t)600 << PIDPos2.qN);
    PIDPos2.qdOutMin = -PIDPos2.qdOutMax;

    InitPID(&PIDPos2, &PIDPos2_f,0);
    
////INIT TRAJ PLANNER 1
    TRAJMotor1_f.enable = 0;
	TRAJMotor1.qVLIM = 17500;
	TRAJMotor1.qACC = 10000;
	TRAJMotor1.qVELshift = 5;
	TRAJMotor1.qACCshift = 8;
		
////INIT TRAJ PLANNER 2
    TRAJMotor2_f.enable = 0;
	TRAJMotor2.qVLIM = 17500;
	TRAJMotor2.qACC = 10000;
	TRAJMotor2.qVELshift = 5;
	TRAJMotor2.qACCshift = 8;
	
////INIT NLFILTER TEST
    InitNLFilter2Fx(&VelocityNLFOut, &VelocityNLFStatus);
////INIT NLFILTER TEST
	InitNLFilter2Fx(&OrientationNLFOut, &OrientationNLFStatus);
    //OrientationNLFStatus.qdXint = PI_Q16-(PI_Q16>>2);
	NLFState = TURNING;
///////////////////////////////////////////////////////////
 
#ifdef DEVELOP_MODE   
    // SETUP A FEW PINS FOR TEST PROBES
    J10Pin1_TRIS = OUTPUT;
    J10Pin2_TRIS = OUTPUT;
    J10Pin3_TRIS = OUTPUT;
    J10Pin4_TRIS = OUTPUT;
#endif
    
	while(1)
		{
			medium_event_handler();
			slow_event_handler();
		}
	
	return 0; //code should never get here
}// END MAIN()

/*******************************************************
* "Soft" real-time event handler for medium rate
********************************************************/
void medium_event_handler(void)
{
	if(medium_event_count > medium_ticks_limit)
	{
#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin3_OUT = 1;
#endif

		medium_event_count = 0;
		
		diagnostics();
		
		debounce_switches();

#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin3_OUT = 0;
#endif	
	}// END IF medium_event_count..
}// END medium_event_handler

/****************************************************
* This function manages diagnostic checks: 
* overcurrents, tracking errors, etc.
****************************************************/
void diagnostics(void)
{
	static uint8_t overcurrent1_count = 0;
	static uint8_t overcurrent2_count = 0;
	int32_t temp1,temp2;
	
	// ACCUMULATE (SORT OF I^2T)
	if(mcurrent1_filt > (max_current + 100))
		overcurrent1_count++;
	else
		overcurrent1_count = 0;
		
	if(overcurrent1_count > 5)
	{
		// OVERCURRENT:
		status_flags.overcurrent1 = 1;
	}
	
	// ACCUMULATE (SORT OF I^2T)
	if(mcurrent2_filt > (max_current + 100))
		overcurrent2_count++;
	else
		overcurrent2_count = 0;
		
	if(overcurrent2_count > 5)
	{
		// OVERCURRENT:
		status_flags.overcurrent2 = 1;
	}
	
	if(control_flags.pos_loop_active)
	{
		temp1 = TRAJMotor1.qdPosition - mposition1;
		temp2 = TRAJMotor2.qdPosition - mposition2;
	}
	else
	{
		temp1 = 0;
		temp2 = 0;
	}
	
	if ((temp1 > 80000) || (temp1 < -80000))
	{
		// TRACKING ERROR:
		status_flags.track_error1 = 1;
	}
	
	if((temp2 > 80000) || (temp2 < -80000))
	{
		// TRACKING ERROR:
		status_flags.track_error2 = 1;
	}
	
	if(status_flags.b != 0)
	{
// switch off immediately and raise OFF_MODE req.
//		control_flags.current_loop_active = 0;
//		control_flags.pos_loop_active = 0;
//		PDC1 = FULL_DUTY;
//		PDC2 = FULL_DUTY;
		
		control_mode.off_mode_req = 1;
	}
} // END diagnostics

/*******************************************************
* "Soft" real-time event handler for slow rate
********************************************************/
void slow_event_handler(void)
{
	if(slow_event_count > slow_ticks_limit)
	{
#ifdef DEVELOP_MODE 
// FOR TEST PROBE
J10Pin4_OUT = 1;
#endif

		slow_event_count = 0;
		
		if(control_flags.first_scan)
		{
			putsUART((unsigned char *)WelcomeMsg,&UART1);
			putsUART((unsigned char *)WelcomeMsg,&UART2);
			
			control_flags.first_scan = 0;
		}
		
		// get rising edge of push-button 1
		if((!push_buttons_state[0].sw1)&&(push_buttons_state[1].sw1))
		{
			if(control_mode.state == OFF_MODE)
			{
    			control_mode.vel_mode1_req = 1;
   			}
   			if(TRAJMotor1.qVelCOM  == 0)
   			{
	   			TRAJMotor1.qVelCOM  = 10000;
	   		}	
   			else
   			{
	   			TRAJMotor1.qVelCOM  = 0;
   			}				
  		}  	 	
    	
    	// get rising edge of push-button 2
		if((!push_buttons_state[0].sw2)&&(push_buttons_state[1].sw2))
    	{
	    	if(control_mode.state == OFF_MODE)
			{
    			control_mode.vel_mode1_req = 1;
   			}
   			if(TRAJMotor2.qVelCOM  == 0)
   			{
	   			TRAJMotor2.qVelCOM  = 10000;
	   		}	
   			else
   			{
	   			TRAJMotor2.qVelCOM  = 0;
   			}		
		}
		
		// UPDATE Switch states for next cycle
		push_buttons_state[0].b = push_buttons_state[1].b;

		// SACT protocol timeout manager (see SACT_protocol.c)
		SACT_timeout();

		// CONTROL MODE STATE MANAGER
		control_mode_manager();

		// SACT protocol SSP/SDP (see SACT_protocol.c)
		SACT_SendSSP();
		SACT_SendSDP();

		// Toggle LEDs
		LED1 = !LED1;
		LED2 = !LED2;

#ifdef DEVELOP_MODE 	
// FOR TEST PROBE
J10Pin4_OUT = 0;
#endif
	} // END IF slow_event_count..
}// END slow_event_handler

/****************************************************
* This function manages the control mode state and
* transitions, including necessary PID init, resets..
****************************************************/
void control_mode_manager(void)
{
	uint8_t idxtemp;
	
	switch(control_mode.state)
	{
//////////////////////////////////////////////////////////////////////
//  OFF MODE
		case OFF_MODE	:  	TRAJMotor1_f.enable = 0;
						   	TRAJMotor2_f.enable = 0;
						   	TRAJMotor1_f.active = 0;
						   	TRAJMotor2_f.active = 0;
						   	TRAJMotor1.qVelCOM  = 0;
						   	TRAJMotor2.qVelCOM  = 0;
						   	rcurrent1 = 0;
							rcurrent2 = 0;
							rcurrent1_req = 0;
							rcurrent2_req = 0;
							while(idxtemp < RCURR_MAV_ORDER)
							{
								rcurrent1samp[idxtemp] = 0;
								idxtemp++;
							}
							control_flags.current_loop_active = 0;
							control_flags.pos_loop_active = 0;
							control_flags.cart_loop_active = 0;
							PDC1 = FULL_DUTY;
							PDC2 = FULL_DUTY;
						 // STATE TRANSITIONS
							if(control_mode.vel_mode1_req)
							{   
								control_mode.state = VEL_MODE1;
							}
							// STATE TRANSITIONS
							else if(control_mode.torque_mode1_req)
								{
									control_mode.state = TORQUE_MODE1;	
								}
							else if(control_mode.cart_mode_req)
								{
									control_mode.state = CART_MODE;
								}
							// IF there is ANY transition, RESETS PIDs
							if(control_mode.trxs)
							{
								// RESET ERROR FLAGS
								status_flags.b = 0;
								
								//RESETS PIDs
								InitPID(&PIDCurrent1, &PIDCurrent1_f,-1);
								InitPID(&PIDCurrent2, &PIDCurrent2_f,-1);
								InitPID(&PIDSpeed1, &PIDSpeed1_f,0);
								InitPID(&PIDSpeed2, &PIDSpeed2_f,0);
								InitPID(&PIDPos1, &PIDPos1_f,0);
								InitPID(&PIDPos2, &PIDPos2_f,0);
								
								control_mode.trxs = 0;
							}
							
							break;
/////////////////////////////////////////////////////////////////////
//  TORQUE MODE 1
		case TORQUE_MODE1 :	control_flags.current_loop_active = 1;

						// STATE TRANSITIONS
						   	if(control_mode.off_mode_req)
							{
								control_mode.state = OFF_MODE;
								control_mode.trxs = 0;
							}
							break;
/////////////////////////////////////////////////////////////////////
//  TORQUE MODE 2
		case TORQUE_MODE2 : break;
/////////////////////////////////////////////////////////////////////
//  VELOCITY MODE 1
		case VEL_MODE1 :   control_flags.current_loop_active = 1;
						   control_flags.pos_loop_active = 1;
						   TRAJMotor1_f.enable = 1;
						   TRAJMotor2_f.enable = 1;
						// STATE TRANSITIONS
						   if(control_mode.off_mode_req)
							{
								control_mode.state = OFF_MODE;
								control_mode.trxs = 0;
							}
								
							break;
/////////////////////////////////////////////////////////////////////
//  VELOCITY MODE 2
		case VEL_MODE2 : break;
/////////////////////////////////////////////////////////////////////
//  CART MODE
		case CART_MODE : control_flags.current_loop_active = 1;
						 control_flags.cart_loop_active = 1;
						 // STATE TRANSITIONS
						 if(control_mode.off_mode_req)
						 {
							control_mode.state = OFF_MODE;
							control_mode.trxs = 0;
						 }
						 break;
/////////////////////////////////////////////////////////////////////
//  ERROR!!!!
		default	:	break;
	}//end SWITCH
}// END control mode manager


/****************************************************
* This function does a simple switch debounce by not
* updating the global variable for valid switch states
* (NOTE: mapped in push_buttons_state[1])
* unless all push buttons have been in the same
* state for 3 calls of the function
* NOTE: push_buttons_state[0] is used in slower
* event handler to detect rising edges
****************************************************/
void debounce_switches(void)
{
	// Take into account if push buttons have pull
	// up resistors (making a logic 0 equal to a button press)
	// or not ..
	push_buttons_state[2].sw1 = !SW_TEST1;
	push_buttons_state[2].sw2 = !SW_TEST2;	
	
// DEBOUNCE LOGIC:
	if (push_buttons_state[2].b != push_buttons_state[3].b)
	{
		push_buttons_state[4].b=push_buttons_state[3].b;
		push_buttons_state[3].b=push_buttons_state[2].b;
		return;
	}

	if (push_buttons_state[3].b != push_buttons_state[4].b)
	{
		push_buttons_state[4].b=push_buttons_state[3].b;
		push_buttons_state[3].b=push_buttons_state[2].b;
	}
	else
	{
		//UPDATE VALID switch state
		push_buttons_state[1].b=push_buttons_state[2].b;
		
		push_buttons_state[4].b=push_buttons_state[3].b;
		push_buttons_state[3].b=push_buttons_state[2].b;
	}
	return;
}
