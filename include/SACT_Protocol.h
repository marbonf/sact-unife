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
 *    Filename:       SACT_Protocol.h                                 *
 *    Date:           20/08/2011                                      *
 *    File Version:   0.8                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for the management of SACT interface protocol
 *
 **********************************************************************/
 
#ifndef SACT_PROT_H
#define SACT_PROT_H

// SACT connection state
#define SACT_NOSYNC     0
#define SACT_ASCII_U1   1
#define SACT_ASCII_U2   2
#define SACT_BIN_U1     3
#define SACT_BIN_U2     4

// HEADER PARTS
#define SACT_HEAD1 0xAA
#define SACT_HEAD2 0x55
#define SACT_EOP   0xFF
#define SACT_SSP   0xA1
#define SACT_SDP   0xB1
#define SACT_SPP   0xC1

// ASCII MODE LIMITS
#define MAX_ASCIILEN 100

#define COMMANDPARA 0
#define MOTORPARA   1
#define ROBOTPARA   2
#define CONTROLPARA 3
#define HWIOPARA    4
#define MAX_HELPMSG 5


// flags
typedef struct {
    unsigned valid_header : 1;
    unsigned packet_full  : 1;
    unsigned param_limit  : 1; //a parameter update request
                               //exceeds limits
    unsigned cr_rec1       : 1;
    unsigned cr_rec2       : 1;
    unsigned help_req      : 1;
    unsigned valid_idx     : 1; //In ASCII mode: valid Index detected 
                                //In BIN mode:command processed 

    unsigned wrong_mode    : 1; //received a command NOT consistent
                                //with control mode
                                //e.g.: set vel in torque mode..
    unsigned timeout       : 8;
} tSACT_flags;    

// For Sabot Sensor Packet configuration
typedef union {
    struct {
    unsigned encoders    : 1; //b0
    unsigned grip_status : 1; //b1
    unsigned trajectory  : 1; //b2
    unsigned analogs     : 1; //b3
    unsigned digitals    : 1; //b4
    unsigned sonars      : 1; //b5
    unsigned currents    : 1; //b6
    unsigned motor_vel   : 1; //b7
    unsigned UNUSED      : 8;
    };
    int16_t word;
} tSSP_config;

// COMMAND/PARAMETERS STRUCTURE
typedef struct {
    uint16_t min;    //minimum allowed parameter value 
    uint16_t max;   //maximum allowed parameter value
    uint8_t args; //number of command arguments
       char *line1_msg;    //line 1 parameter screen message 
    char *quick_msg;    //abbriviation for message 
    } t_command_data; 
   
#define N_COMMANDS 13
#define N_PARAMS   32
#define MAXARGS 3

// PUBLIC FUNCTIONS FOR PROTOCOL MANAGEMENT
// AND TIMEOUT INCREMENT (to be called periodically)
#define SACT_TIME_LIMIT 3000 //in ms
void U1_SACT_Parser(void);
void U2_SACT_Parser(void);
void SACT_timeout(void);
void SACT_SendSSP(void);
void SACT_SendSDP(void);


#endif

