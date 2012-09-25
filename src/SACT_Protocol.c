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
*****************************************************************************
 *                                                                    *
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       SACT_Protocol.c                                 *
 *    Date:           20/01/2011                                      *
 *    File Version:   0.8                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the management of SACT interface protocol
 *  and all the related configuration parameters.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "generic_defs.h"
#include "extern_globals.h"
#include "Comms.h"
#include "SACT_Protocol.h"
#include "my_fractmath.h"
#include "lib_crc.h"

#include <string.h> //for memcmp()
#include <stdlib.h> //for atol()
                    //NOTE: itoa() is not supported natively by C30!!!
#include <limits.h> //for INT_MAX and INT_MIN


/**********************************************
 * COMMANDS / PARAMETERS DEFINITION:
 *********************************************/

const t_command_data command_data [N_COMMANDS+N_PARAMS] =    {
// Min,Max,Args, Line1 msg, Quick msg 
{0,0,0,             "Disconnect SACT ","DIS"},//0
{0,0,1,             "CONTROL MODE    ","CMO"},//1
{0,0,2,             "SET TORQUE Refs.","STR"},//2
{0,0,2,             "SET VELOC. Refs.","SVR"},//3
{0,0,2,             "SET POSIT. Refs ","SPR"},//4
{0,0,1,             "To Be DEFINED 1 ","TB1"},//5
{0,0,1,             "To Be DEFINED 2 ","TB2"},//6
{0,0,1,             "To Be DEFINED 3 ","TB3"},//7
{0,0,0,             "PULSE (BIN.only)","PUL"},//8
{0,0,0,             "UPDATE EEPROM   ","UEE"},//9
{0,0,2,             "RESET Mot.ANGLEs","RMA"},//10
{0,0,1,             "RESET T.B.D.    ","RTB"},//11
{0,0,1,             "SET SSP FLAGS   ","SSF"},//12
{1,32767,1,         "MAX CURRENT     ","MXC"},//13
{1,32767,1,         "MAX VELOCITY    ","MXV"},//14
{1,32767,1,         "MAX ACCELERATION","MXA"},//15
{0,15,1,            "VEL. SCALING N. ","VSN"},//16
{0,15,1,            "ACC. SCALING N. ","ASN"},//17
{1,32767,1,         "CURR.Loop P GAIN","CLP"},//18
{1,32767,1,         "CURR.Loop I GAIN","CLI"},//19
{1,32767,1,         "CURR.Loop D GAIN","CLD"},//20
{0,15,1,            "CURR.Loop SCALE ","CLS"},//21
{1,32767,1,         "VEL. Loop P GAIN","VLP"},//22
{1,32767,1,         "VEL. Loop I GAIN","VLI"},//23
{1,32767,1,         "VEL. Loop D GAIN","VLD"},//24
{0,15,1,            "VEL. Loop SCALE ","VLS"},//25
{1,32767,1,         "POS. Loop P GAIN","PLP"},//26
{1,32767,1,         "POS. Loop I GAIN","PLI"},//27
{1,32767,1,         "POS. Loop D GAIN","PLD"},//28
{0,20,1,            "POS. Loop SCALE ","PLS"},//29
{1,32767,1,         "POSIT.ANGLE Lim.","PAL"},//30
{1,32767,1,         "NEGAT.ANGLE Lim.","NAL"},//31
{1,32767,1,         "ENC. Count. Rev.","ECR"},//32
{0,63,1,            "DIRECTION flags ","DRF"},//33
{1,32767,1,         "ADC TORQUE SCALE","ATS"},//34
}; 


// PARAMETERS stored in RAM.. default values..
uint16_t parameters_RAM[N_PARAMS]=
{    
    800,            // 0: MAX CURRENT (Command 13)
    17500,          // 1: MAX VELOCITY (Command 14)
    10000,          // 2: MAX ACCELERATION (Command 15)
    5,              // 3: VELOCITY SCALING SHIFT (Command 16)
    8,              // 4: ACCELERATION SCALING SHIFT (Command 17)
    600,            // 5: CURRENT LOOP P GAIN (Command 18)
    80,             // 6: CURRENT LOOP I GAIN (Command 19)
    0,              // 7: CURRENT LOOP D GAIN (Command 20)
    9,              // 8: CURRENT LOOP SCALING SHIFT (Command 21)
    1000,           // 9:  VELOCITY LOOP P GAIN (Command 22)
    20,             // 10: VELOCITY LOOP I GAIN (Command 23)
    0,              // 11: VELOCITY LOOP D GAIN (Command 24)
    9,              // 12: VELOCITY LOOP SCALING SHIFT (Command 25)
    400,            // 13: POSITION LOOP P GAIN (Command 26)
    0,              // 14: POSITION LOOP I GAIN (Command 27)
    0,              // 15: POSITION LOOP D GAIN (Command 28)
    18,             // 16: POSITION LOOP SCALING SHIFT (Command 29)
    900,            // 17: POSITIVE ANGLE LIMIT (Command 30)
    200,            // 18: NEGATIVE ANGLE LIMIT (Command 31)
    21500,          // 19: ENCODER COUNTS/REV (Command 32)
    32,             // 20: DIRECTION flags (Command 33)
    450,            // 21: ADC TORQUE SCALE (Command 34)
    0,              // 22: UNUSED
    0,              // 23: UNUSED
    0,              // 24: UNUSED
    0,              // 25: UNUSED
    0,              // 26: UNUSED
    0,              // 27: UNUSED
    0,              // 28: UNUSED
    0,              // 29: UNUSED
    0,              // 30: UNUSED
    0,              // 31: UNUSED
};        

// HELP MESSAGES
const unsigned char ErrorMsg[] = {"\r\nIncorrect Command! Type '??' for command set.\r\n"};
const unsigned char ErrorParaMsg[] = {"\r\nIncorrect Command! Wrong N. of parameters.\r\n"};
const unsigned char ErrorParaLengthMsg[] = {"\r\nIncorrect Command! Wrong param. length.\r\n"};
const unsigned char ErrorParaLimitMsg[] = {"\r\nError: parameter out of range!\r\n"};
const unsigned char ErrorControlModeMsg[] = {"\r\nError: command invalid in current "};
const unsigned char CRLFMsg[] = {"\r\nCommands must be terminated with CR/LF sequence...\r\n"};
const unsigned char SyncMsg[] = {"\r\nWaiting for SYNC0+cr/lf SYNC1+cr/lf SYNCA+cr/lf\r\n"};
const unsigned char ModeErrorMsg[] = {"\r\nControl Mode active! No params update allowed.\r\n"};
const unsigned char CommandParaHeader[] = {"\r\nAction Commands:\r\n"};
const unsigned char MotorParaHeader[] = {"\r\nMotor Parameters:\r\n"};
const unsigned char RobotParaHeader[] = {"\r\nRobot Parameters:\r\n"};
const unsigned char ControlParaHeader[] = {"\r\nControl Parameters:\r\n"};
const unsigned char IOParaHeader[] = {"\r\nHW I/Os Parameters:\r\n"};
const unsigned char ParaHeader[] = 
            {"Description\t\tAbbreviation\t\tCurrent Value\r\n"};
const unsigned char ParaHeader_cmd[] = 
            {"Description\t\tAbbreviation\t\tArguments\r\n"};

const unsigned char HelpMsg_data [MAX_HELPMSG][32] =    
{    
    {"For Action Commands\tType '?A'\r\n"},
    {"For Motor Params\tType '?M'   \r\n"},
    {"For Robot Params\tType '?R'   \r\n"},
    {"For Control Params\tType '?C' \r\n"},
    {"For HW I/Os Params\tType '?I' \r\n"},
};

const unsigned char ControlModeMsg [4][15] =    
{    
    {"OFF_MODE    \r\n"},
    {"TORQUE_MODE\r\n"},
    {"VEL_MODE   \r\n"},
    {"POS_MODE   \r\n"},
};

const unsigned char FaultMsg[4][30] =
{    
    {"FAULT: overcurrent motor 1 \r\n"},
    {"FAULT: overcurrent motor 2 \r\n"},
    {"FAULT: track error motor 1 \r\n"},
    {"FAULT: track error motor 2 \r\n"},
};

// TABLE ASSOCIATING EACH COMMAND/PARAM TO RELATED GROUP FOR HELP INFO:
// - each line corresponds to a group
// - each number in a line corresponds to the index of a command/parameter
//   associated to that group
const uint8_t help_info[MAX_HELPMSG][15] =
{
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,50,50}, // COMMANDS
    {13,14,15,16,17,30,31,32,33,34,50,50,50,50,50}, // MOTOR
    {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50}, // ROBOT
    {18,19,20,21,22,23,24,25,26,27,28,29,50,50,50}, // CONTROL
    {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50}, // HW I/Os
};

// LOCAL VARIABLES
unsigned char rx1buf[MAX_ASCIILEN];
uint8_t rx1cnt = 0;
unsigned char rx2buf[MAX_ASCIILEN];
uint8_t rx2cnt = 0;
unsigned char BINRXbuf[MAX_ASCIILEN];
uint8_t BINRXcnt = 0;
unsigned char BINTXbuf[MAX_ASCIILEN];
uint8_t BINTXcnt = 0;
unsigned char u1temp = 0;
unsigned char u1prev = 0;
unsigned char u2temp = 0;
unsigned char u2prev = 0;

uint8_t SACT_state = SACT_NOSYNC;
uint8_t SYNC_U1_step = 0;
uint8_t SYNC_U2_step = 0;

uint8_t BINLastCommand;

tSACT_flags SACT_flags;

tSSP_config SSP_config;

// LOCAL FUNCTIONS
void process_SYNC_U1(void);
void process_SYNC_U2(void);
// for ASCII mode
void process_ASCII(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg);
void CheckHelp(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg);
void SendHelpInfo(uint8_t table,volatile UART *ureg);
unsigned char GetMsgIndex(unsigned char *rxbuf);
void GetParamASCII(uint8_t idx, volatile UART *ureg);
void GetParamBIN(uint8_t idx, volatile UART *ureg);

// for ASCII mode
void process_BIN(unsigned char *rxbuf, uint8_t rxcnt);
void ParseBINCommand(void);


void ExecCommand(uint8_t idx,int16_t *args);


/****************************************
 * UART1 Buffer parser for SACT Protocol
 ***************************************/
void U1_SACT_Parser(void)
{
#ifdef DEVELOP_MODE     
// FOR TEST PROBE
//J10Pin3_OUT = 1;
#endif

while(u1bufhead != u1buftail) // Data Available in the buffer
{
    u1prev = u1temp;
    u1temp = u1tmpbuf[u1buftail++];
    
    switch(SACT_state)
    {
//////////////////////////////////////////////////////////////////////
// NO SYNC STATE
        case SACT_NOSYNC:   rx1buf[rx1cnt++]=u1temp;
                            switch(u1temp)
                            {
                                case CR : SACT_flags.cr_rec1 = 1;
                                          break;
                                case LF : if (u1prev == CR)
                                            {
                                                // TERMINATE STRING
                                                rx1buf[rx1cnt]=NULLC;
                                                process_SYNC_U1();
                                                rx1cnt = 0;
                                                SACT_flags.cr_rec1 = 0;
                                            }
                                          else
                                            {
                                                putsUART((unsigned char*)CRLFMsg,&UART1);
                                                rx1cnt = 0;
                                            }    
                                          break;
                                default : //rx1buf[rx1cnt++]=u1temp;
                                          if(rx1cnt > (MAX_ASCIILEN-1))
                                          {
                                                putsUART((unsigned char*)SyncMsg,&UART1);
                                                rx1cnt = 0;
                                          }
                                          
                                          if(SACT_flags.cr_rec1)
                                          {
                                                putsUART((unsigned char*)CRLFMsg,&UART1);
                                                rx1cnt = 0;
                                                SACT_flags.cr_rec1 = 0;
                                          }
                                          break;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART1
        case SACT_ASCII_U1: rx1buf[rx1cnt++]=u1temp;
                            switch(u1temp)
                            {
                                case CR : SACT_flags.cr_rec1 = 1;
                                          break;
                                case LF : if (u1prev == CR)
                                            {
                                                // TERMINATE STRING
                                                rx1buf[rx1cnt]=NULLC;
                                                process_ASCII(rx1buf,rx1cnt,&UART1);
                                                rx1cnt=0;
                                                SACT_flags.cr_rec1 = 0;
                                            }
                                          else
                                            {
                                                putsUART((unsigned char*)CRLFMsg,&UART1);
                                                rx1cnt = 0;
                                            }
                                          break;
                                default : //rx1buf[rx1cnt++]=u1temp;
                                          if(rx1cnt > (MAX_ASCIILEN-1))
                                          {
                                                putsUART((unsigned char*)ErrorMsg,&UART1);
                                                rx1cnt = 0;
                                          }
                                          
                                          if(SACT_flags.cr_rec1)
                                          {
                                                putsUART((unsigned char*)CRLFMsg,&UART1);
                                                rx1cnt = 0;
                                                SACT_flags.cr_rec1 = 0;
                                          }
                                          break;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART1
        case SACT_BIN_U1:   rx1buf[rx1cnt++]=u1temp;
                            process_BIN(rx1buf,rx1cnt);
                            if(SACT_flags.valid_idx)
                            {
                                rx1cnt = 0; //Reset count if a command has been processed!
                                SACT_flags.valid_idx = 0;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART2, NOT MANAGED HERE (see U2RXInterrupt)
        case SACT_ASCII_U2: 
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART2, NOT MANAGED HERE (see U2RXInterrupt)
        case SACT_BIN_U2:   U1TXREG = u1temp;
                            rx1cnt = 0;
                            break;
//////////////////////////////////////////////////////////////////////
// ERROR!!!!
        default: break;
    }// END SWITCH
}// END WHILE Data Available

#ifdef DEVELOP_MODE     
// FOR TEST PROBE
//J10Pin3_OUT = 0;
#endif
}// END U1 SACT Parser

/****************************************
 * UART1 Buffer parser for SACT Protocol
 ***************************************/
void U2_SACT_Parser(void)
{    

while(u2bufhead != u2buftail) // Data Available in the buffer
{
    u2prev = u2temp;
    u2temp = u2tmpbuf[u2buftail++];
    
    switch(SACT_state)
    {
//////////////////////////////////////////////////////////////////////
// NO SYNC STATE
        case SACT_NOSYNC:   rx2buf[rx2cnt++]=u2temp;
                            switch(u2temp)
                            {
                                case CR : SACT_flags.cr_rec2 = 1;
                                          break;
                                case LF : if (u2prev == CR)
                                            {
                                                // TERMINATE STRING
                                                rx2buf[rx2cnt]=NULLC;
                                                process_SYNC_U2();
                                                rx2cnt = 0;
                                                SACT_flags.cr_rec2 = 0;
                                            }
                                          else
                                            {
                                                putsUART((unsigned char*)CRLFMsg,&UART2);
                                                rx2cnt = 0;
                                            }
                                          break;
                                default : //rx2buf[rx2cnt++]=u2temp;
                                          if(rx2cnt > (MAX_ASCIILEN-1))
                                          {
                                              putsUART((unsigned char*)SyncMsg,&UART2);
                                              rx2cnt = 0;
                                          }
                                          
                                          if(SACT_flags.cr_rec2)
                                          {
                                              putsUART((unsigned char*)CRLFMsg,&UART2);
                                              rx2cnt = 0;
                                              SACT_flags.cr_rec2 = 0;
                                          }
                                          break;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART2
        case SACT_ASCII_U2: rx2buf[rx2cnt++]=u2temp;
                            switch(u2temp)
                            {
                                case CR : SACT_flags.cr_rec2 = 1;
                                          break;
                                case LF : if (u2prev == CR)
                                            {
                                                // TERMINATE STRING
                                                rx2buf[rx2cnt]=NULLC;
                                                process_ASCII(rx2buf,rx2cnt,&UART2);
                                                rx2cnt = 0;
                                                SACT_flags.cr_rec2 = 0;
                                            }
                                          else
                                              {
                                                  putsUART((unsigned char*)CRLFMsg,&UART2);
                                                  rx2cnt = 0;
                                              }
                                          break;
                                default : //rx2buf[rx2cnt++]=u2temp;
                                          if(rx2cnt > (MAX_ASCIILEN-1))
                                          {
                                              putsUART((unsigned char*)ErrorMsg,&UART2);
                                              rx2cnt = 0;
                                          }
                                          if(SACT_flags.cr_rec2)
                                          {
                                              putsUART((unsigned char*)CRLFMsg,&UART2);
                                              rx2cnt = 0;
                                              SACT_flags.cr_rec2 = 0;
                                          }
                                          break;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART2
        case SACT_BIN_U2:   rx2buf[rx2cnt++]=u2temp;
                            process_BIN(rx2buf,rx2cnt);
                            if(SACT_flags.valid_idx)
                            {
                                rx2cnt = 0; //Reset count if a command has been processed!
                                SACT_flags.valid_idx = 0;
                            }
                            break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART1, NOT MANAGED HERE (see U1RXInterrupt)
        case SACT_ASCII_U1: 
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART1, NOT MANAGED HERE (see U1RXInterrupt)
        case SACT_BIN_U1:   U2TXREG = u2temp;
                            rx2cnt=0;
                            break;
//////////////////////////////////////////////////////////////////////
// ERROR!!!!
        default: break;
    }// END SWITCH
} // END WHILE Data Available

    
}// END U2 SACT Parser

/****************************************
 * function to detect SYNC on UART1
 ***************************************/
void process_SYNC_U1(void)
{
    int16_t cmpres;
    
    switch(SYNC_U1_step)
    {
        case 0: cmpres = memcmp(rx1buf,"SYNC0",5);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    SYNC_U1_step++;
                    putsUART(rx1buf,&UART1);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                }
                break;
        case 1: cmpres = memcmp(rx1buf,"SYNC1",5);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    SYNC_U1_step++;
                    putsUART(rx1buf,&UART1);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                    SYNC_U1_step = 0;
                }    
                break;
        case 2: cmpres = memcmp(rx1buf,"SYNC",4);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    if (rx1buf[4]=='A')
                    {
                        putsUART(rx1buf,&UART1);
                        SACT_state = SACT_ASCII_U1;
                    }    
                    else if(rx1buf[4]=='B')
                        {
                            putsUART(rx1buf,&UART1);
                            SACT_state = SACT_BIN_U1;
                        }
                }
                
                if(SACT_state == SACT_NOSYNC) putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                
                SYNC_U1_step = 0;    
                break;
        default: break;
    }// END switch SYNC_.._step    
}//END process_SYNC

/****************************************
 * function to detect SYNC on UART2
 ***************************************/
void process_SYNC_U2(void)
{
    int16_t cmpres;
    
    switch(SYNC_U2_step)
    {
        case 0: cmpres = memcmp(rx2buf,"SYNC0",5);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    SYNC_U2_step++;
                    putsUART(rx2buf,&UART2);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                }
                break;
        case 1: cmpres = memcmp(rx2buf,"SYNC1",5);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    SYNC_U2_step++;
                    putsUART(rx2buf,&UART2);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                    SYNC_U2_step = 0;
                }    
                break;
        case 2: cmpres = memcmp(rx2buf,"SYNC",4);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    if (rx2buf[4]=='A')
                    {
                        putsUART(rx2buf,&UART2);
                        SACT_state = SACT_ASCII_U2;
                    }    
                    else if(rx2buf[4]=='B')
                        {
                            putsUART(rx2buf,&UART2);
                            SACT_state = SACT_BIN_U2;
                        }
                }
                
                if(SACT_state == SACT_NOSYNC) putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                
                SYNC_U2_step = 0;    
                break;
        default: break;
    }// END switch SYNC_.._step    
}//END process_SYNC


/******************************************************************************
 * functions to manage ASCII mode protocol:
 * - process_ASCII: calls parse functions on the whole packet
 * - CheckHelp: checks if a help request is received
 * - GetMsgIndex: scans the table of valid commands to identify index
 *                corresponding to received one
 * - GetParamASCII: shows to UART the value of requested parameter
 *****************************************************************************/
void process_ASCII(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg)
{
    uint8_t idx = 0;
    uint8_t argcount = 0;
    uint8_t count = 0;
    uint8_t accum = 0;
    int16_t args[MAXARGS];
    int32_t temparg;
    unsigned char tempstr[8]; //ONLY INT VAL EXPECTED

////TOGGLE LED IF CONTROL MODE ACTIVE
    if(control_mode.state != OFF_MODE)
        LED2 = !LED2;

    CheckHelp(rxbuf,rxcnt,ureg);
    if(!SACT_flags.help_req)
    {
        idx = GetMsgIndex(rxbuf);
    }
    else
    {
        SACT_flags.help_req = 0;
        return;
    }
    
    if(SACT_flags.valid_idx)
    {    
        //RESET valid_idx flag
        SACT_flags.valid_idx = 0;
        
        //received COMMAND is short, it is a a "GET param", a command without args
        //OR an invalid command 
        if((rxbuf[3] != SPACE)||(rxcnt < 5 + command_data[idx].args*2))
        {
            if(idx >= N_COMMANDS)
            {
                //Command is a parameter request, without args -> GET Param!!
                GetParamASCII(idx,ureg);
                return;
            }
            else if(command_data[idx].args != 0) //received command requires NO args
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO FEW
                    return;
                }
        }
        
        //SOMETHING THAT SEEMS GOOD in the buffer..
        while(argcount < command_data[idx].args)
        {    
            // search for a space or CR
            // if we are here we should have at least a space or a CR in the buffer
            // from index 4 to ..
            while((rxbuf[count+accum+4] != SPACE)&&(rxbuf[count+accum+4] != CR)) count++;
            
            if(count < 8) //ONLY INT VAL EXPECTED
            {
                memcpy(tempstr,&rxbuf[accum+4],count);
                // TERMINATE STRING
                tempstr[count] = NULLC;
                temparg = atol((char *)tempstr);
                // CHECK BOUNDS
                if((temparg < INT_MIN) || (temparg > INT_MAX))
                {
                    putsUART((unsigned char*)ErrorParaLimitMsg,ureg);
                    return;
                }

                args[argcount] = (int16_t)temparg;
                accum += count+1; //discard space or CR
                count = 0;
            }
            else
            {
                putsUART((unsigned char*)ErrorParaLengthMsg,ureg); //args too long
                return;
            }

            count = 0;
            argcount++;

            //CHECK N.ARGS
            if((rxbuf[accum+4] == LF)&&(argcount < command_data[idx].args))
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO FEW
                    return;
                }
        }//END while argcount

        //CHECK N.ARGS
        if((accum + 4) != (rxcnt-1))
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO MUCH
                    return;
                }

        //NOW WE HAVE VALID COMMAND AND VALID ARGUMENTS!!!
        ExecCommand(idx,args);
        
        //ERROR IN A PARAMETER VALUE
        if(SACT_flags.param_limit)
            {
                putsUART((unsigned char*)ErrorParaLimitMsg,ureg);
                SACT_flags.param_limit = 0;
            }

        //COMMAND NOT CONSISTENT WITH CONTROL MODE
        if(SACT_flags.wrong_mode)
            {
                putsUART((unsigned char*)ErrorControlModeMsg,ureg);
                putsUART((unsigned char*)ControlModeMsg[control_mode.state],ureg);
                SACT_flags.wrong_mode = 0;
            }

        //ECHO COMMAND???
        putsUART(rxbuf,ureg);

    }//END if valid_idx
    else
    {
        putsUART((unsigned char*)ErrorMsg,ureg); //invalid command
    }
    
}//END process_ASCII

// GET MESSAGE INDEX
unsigned char GetMsgIndex(unsigned char *rxbuf)
{
    uint8_t tempidx = 0;
    int16_t cmpres;
        
    while(tempidx < (N_COMMANDS + N_PARAMS))
    {
        cmpres = memcmp(rxbuf,command_data[tempidx].quick_msg,3);
        if(cmpres == 0)
        {
            SACT_flags.valid_idx = 1;
            return tempidx;
        }
            
        tempidx++;
    }
    if(tempidx == (N_COMMANDS + N_PARAMS))    return 0xFF;

    return 0xFE; // should never get here, just to suppress warning
}//END GetMsgIndex

// GET PARAMETER in ASCII MODE
void GetParamASCII(uint8_t idx, volatile UART *ureg)
{
    putsUART((unsigned char*)command_data[idx].line1_msg,ureg);
    putcUART(HT,ureg);
    putuiUART(parameters_RAM[idx-N_COMMANDS],ureg);
    putcUART(CR,ureg);putcUART(LF,ureg);
}

// Check if a request for help is received
void CheckHelp(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg)
{
  uint8_t idx = 0;
    
  
    if (rxbuf[0] == '?')
    {
      if(rxcnt == 4)
          {
        switch (rxbuf[1])
        {
        case '?':   do
                    putsUART((unsigned char *)HelpMsg_data[idx],ureg);
                    while (++idx < MAX_HELPMSG);
                    break;

        case 'M':   putsUART((unsigned char *)MotorParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(MOTORPARA,ureg);
                    break;

        case 'R':   putsUART((unsigned char *)RobotParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(ROBOTPARA,ureg);
                    break;

        case 'C':   putsUART((unsigned char *)ControlParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(CONTROLPARA,ureg);
                    break;

        case 'I':   putsUART((unsigned char *)IOParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(HWIOPARA,ureg);
                    break;

        case 'A':   putsUART((unsigned char *)CommandParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader_cmd,ureg);
                    SendHelpInfo(COMMANDPARA,ureg);
                    break;

        default:    putsUART((unsigned char*)ErrorMsg,ureg);
                    break;
        }//END switch
       
       } // END if rxcnt
      else
      {
         putsUART((unsigned char*)ErrorMsg,ureg);
      }

    // SET that a help req is received
    SACT_flags.help_req = 1;

  }//END if rxbuf[0]
}//END CheckHelp

// SEND HELP AND PARAMS searching in the commanda_data table
// for matching indexes
void SendHelpInfo(uint8_t table,volatile UART *ureg)
{

uint8_t idx = 0;
uint8_t count = 0;

do
    if (idx == help_info[table][count])
        {
        putsUART((unsigned char *)command_data[idx].line1_msg,ureg);
        putcUART(HT,ureg);putcUART(HT,ureg);
        putsUART((unsigned char *)command_data[idx].quick_msg,ureg);
        putcUART(HT,ureg);putcUART(HT,ureg);
        if(table == COMMANDPARA)
            putuiUART(command_data[idx].args,ureg); // Send Args
        else
            putuiUART(parameters_RAM[idx-N_COMMANDS],ureg); // SendVALUE
        putcUART(CR,ureg);putcUART(LF,ureg);
        count++;
        } 
while (++idx < (N_COMMANDS + N_PARAMS));
}

/******************************************************************************
 * functions to manage BINARY mode protocol:
 * - process_BIN: detects header and counts bytes 
 * - ParseBINCommand: parse the data packet
 * - GetParamBIN: sends current value of a parameter
 *****************************************************************************/
void process_BIN(unsigned char *rxbuf, uint8_t rxcnt)
{
    if((!SACT_flags.valid_header) && (rxcnt>1))
    {
        if((rxbuf[rxcnt-2]==SACT_HEAD1)&&(rxbuf[rxcnt-1]==SACT_HEAD2)) SACT_flags.valid_header = 1;
        return;
    }
    
    if(SACT_flags.valid_header && !SACT_flags.packet_full)
    {
        BINRXbuf[BINRXcnt++] = rxbuf[rxcnt-1]; //rxcnt-1 because if we are here it is the latest char..
        if(BINRXcnt == (BINRXbuf[0]+1)) //BINRXbuf includes byte count itself
        {
            if(BINRXbuf[BINRXcnt-1] == SACT_EOP)
            {
                SACT_flags.packet_full = 1;
            }
            else 
            {
                SACT_flags.packet_full = 0;
                SACT_flags.valid_header = 0;
                return;
            }
        }
    }
    
    // NOT IN THE ELSE.. must check immediately!
    if(SACT_flags.packet_full)
    {
        ParseBINCommand();
        BINRXcnt=0;
        SACT_flags.packet_full = 0;
        SACT_flags.valid_header = 0;
        SACT_flags.valid_idx = 1;
    }
}

// CALCULATES CRC AND UNPACK COMMAND/ARGS
void ParseBINCommand(void)
{
    
    uint8_t idx = 0;
    uint8_t argcount = 0;
    uint8_t count = 0;
    uint8_t accum = 0;
    int16_t args[MAXARGS];
    WRD temparg;

////TOGGLE LED IF CONTROL MODE ACTIVE
    if(control_mode.state != OFF_MODE)
        LED2 = !LED2;

    
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;

    while(count < (BINRXbuf[0]-3)) //BINRXbuf includes byte count itself at 0 index
    {
        crc_16 = update_crc_16(crc_16,BINRXbuf[count+1]);
        count++;
    }

    temparg.ui = crc_16;

    if((temparg.uc[0] == BINRXbuf[BINRXcnt-3]) && (temparg.uc[1] == BINRXbuf[BINRXcnt-2]))
    {
        idx = BINRXbuf[1]; //BINRXbuf includes byte count itself
                           //index 1 is the command id
        
        // IF COMMAND IS A PARAMETER NUMBER WITHOUT ARGS
        // IT IS A 'GET' REQUEST           
        if((idx >= N_COMMANDS)&&(BINRXcnt==5))
        {
            if(SACT_state == SACT_BIN_U1)
                GetParamBIN(idx,&UART1);
            else //if we are here we have certainly SACT_BIN_U2
                GetParamBIN(idx,&UART2);
            return;
        }
        
        while(argcount < command_data[idx].args)
        {
            temparg.uc[0] = BINRXbuf[2+accum];
            temparg.uc[1] = BINRXbuf[3+accum];
            args[argcount] = temparg.i;
            argcount++;
            accum += 2;
            
            if((accum == BINRXcnt-4)&&(argcount < command_data[idx].args))
            {
                //Wrong number of params (TOO FEW)
                status_flags.comm_error_code = 1;
                return;
            }
        }

        if(accum != (BINRXbuf[0]-4))
        {
            //TODO: wrong number of params (TOO FEW)
            status_flags.comm_error_code = 1;
            return;
        }

        //NOW WE HAVE VALID COMMAND AND VALID ARGUMENTS!!!
        ExecCommand(idx,args);

        BINLastCommand = idx;
        
        //ERROR IN A PARAMETER VALUE
        if(SACT_flags.param_limit)
            {
                //TODO... diag update
                SACT_flags.param_limit = 0;
                status_flags.comm_error_code = 1;
            }

        //COMMAND NOT CONSISTENT WITH CONTROL MODE
        if(SACT_flags.wrong_mode)
            {
                //TODO.. diag update
                SACT_flags.wrong_mode = 0;
                status_flags.comm_error_code = 3;
            }
    }//END if crc OK
    else
    {
        //BAD CRC
        status_flags.comm_error_code = 2;
    }
}//END ParseBINCommand()

// GET PARAMETER VALUE IN BINARY MODE
void GetParamBIN(uint8_t idx, volatile UART *ureg)
{
    WRD temp;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;
    uint8_t count = 0;
    
////////PREPARE CONSTANT PART
        BINTXbuf[0] = SACT_HEAD1;
        BINTXbuf[1] = SACT_HEAD2;
        BINTXbuf[2] = 7;
        BINTXbuf[3] = SACT_SPP;
        BINTXbuf[4] = idx;
    
///////PARAM VALUE
        temp.ui = parameters_RAM[idx-N_COMMANDS];
        BINTXbuf[5] = temp.uc[0];
        BINTXbuf[6] = temp.uc[1];
        
////////CALCULATE CRC    
        while(count < 4) 
        {
            crc_16 = update_crc_16(crc_16,BINTXbuf[count+3]);
            count++;
        }

        temp.ui = crc_16;
        BINTXbuf[7] = temp.uc[0];
        BINTXbuf[8] = temp.uc[1];

////////END OF PACKET
        BINTXbuf[9] = SACT_EOP;

////////SEND PACKET
        if(SACT_state == SACT_BIN_U1)
        {
            SendNUART(BINTXbuf,&UART1,10);
        }
        else //if we are here we have certainly SACT_BIN_U2
        {
            SendNUART(BINTXbuf,&UART2,10);
        }                        
}//END GetParamBIN

/******************************************************************************
 * CORE FUNCTION for commands execution
 *****************************************************************************/
void ExecCommand(uint8_t idx,int16_t *args)
{    
    int16_t temp1,temp2;
    
    if(idx >= N_COMMANDS)
    { // IT IS A PARAMETER UPDATE REQUEST
        if(control_mode.state == OFF_MODE)
        {
            if((args[0] >= command_data[idx].min)&&(args[0] <= command_data[idx].max))
                {
                    parameters_RAM[idx-N_COMMANDS] = args[0];
                    control_flags.PAR_update_req = 1;
                }    
            else
                SACT_flags.param_limit = 1;
        }
        else
            SACT_flags.wrong_mode = 1;
    }
    else
    { // IT IS AN ACTION COMMAND    

        switch(idx)
        {
            case 0: // DISCONNECT
                    SACT_state = 0;
                    control_mode.off_mode_req = 1;
                     break;
            case 1: // CONTROL MODE
                    if(control_mode.state == OFF_MODE)
                    {
                      switch(args[0])
                      {
                        case OFF_MODE    :     
                                            break;
                        case TORQUE_MODE :  control_mode.torque_mode_req = 1;
                                            break;
                        case VEL_MODE    : control_mode.vel_mode_req = 1;
                                            break;
                        case POS_MODE    : control_mode.pos_mode_req = 1;
                                            break;
                        default : break; // SHOULD NEVER HAPPEN
                       }//END switch args[0]
                    }
                     else
                    {
                        if(args[0] == OFF_MODE) control_mode.off_mode_req = 1;
                    }
                    break;
            case 2: // SET TORQUE REF
                    if(control_mode.state == TORQUE_MODE)
                    {
                        temp1 = args[0];
                        temp2 = args[1];
                        if(temp1 < 0) temp1 = -temp1;
                        if(temp2 < 0) temp2 = -temp2;
                        
                        if((temp1 > max_current)||(temp2 > max_current))
                        {
                            SACT_flags.param_limit = 1;
                        }
                        else
                        {
                        rcurrent1_req = args[0];
                        rcurrent2_req = args[1];
                        }
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 3: // SET VELOCITY REFS
                    if(control_mode.state == VEL_MODE)
                    {
                        temp1 = args[0];
                        temp2 = args[1];
                        if(temp1 < 0) temp1 = -temp1;
                        if(temp2 < 0) temp2 = -temp2;
                        
                        if((temp1 > max_velocity)||(temp2 > max_velocity))
                        {
                            SACT_flags.param_limit = 1;
                        }
                        else
                        {
                        TRAJMotor1.qVelCOM = args[0];
                        TRAJMotor2.qVelCOM = args[1];
                        }
                        
                    }
                    else 
                        SACT_flags.wrong_mode = 1;
                            
                    break;
            case 4: // SET POSITION REFs.
                    if(control_mode.state == POS_MODE)
                    {
                        temp1 = args[0];
                        temp2 = args[1];
                        
                        if((temp1 < min_angle)||(temp1 > max_angle)||(temp2 < min_angle)||(temp2 > max_angle))
                        {
                            SACT_flags.param_limit = 1;
                        }
                        else
                        {
                            if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec) //if exec=1 it doesn't execute any command
	                        {
	                            TRAJMotor1.qdPosCOM = RSH(((int32_t)temp1 * decdeg_to_ticks),8); // deg to ticks is 8.8 fixed-point so RightSHift!
                                TRAJMotor2.qdPosCOM = RSH(((int32_t)temp2 * decdeg_to_ticks),8); // deg to ticks is 8.8 fixed-point so RightSHift!

		                        TRAJMotor1_f.exec = 1;
    	                        TRAJMotor2_f.exec = 1;
	                        }
                        }
                        
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 5: break;
            case 6: break;
            case 7: break;
            case 8: // PULSE
                    Nop();
                    break;
            case 9: // UPDATE EEPROM;
                    if(control_mode.state == OFF_MODE)
                    {
                        if(!control_flags.EE_update_req)
                            control_flags.EE_update_req = 1;
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 10: // RESET Encoder counts
                    if(control_mode.state == OFF_MODE)
                    {
                        temp1 = args[0];
                        temp2 = args[1];
                        
                        if((temp1 < min_angle)||(temp1 > max_angle)||(temp2 < min_angle)||(temp2 > max_angle))
                        {
                            SACT_flags.param_limit = 1;
                        }
                        else
                        {
                            mposition1 = RSH(((int32_t)temp1 * decdeg_to_ticks),8); // deg to ticks is 8.8 fixed-point so RightSHift!
                            if(direction_flags.simulate_1)
                                TRAJMotor1.qdPosition = mposition1;
                            mposition2 = RSH(((int32_t)temp2 * decdeg_to_ticks),8); // deg to ticks is 8.8 fixed-point so RightSHift!
                            if(direction_flags.simulate_2) 
                                TRAJMotor2.qdPosition = mposition2;
		                }
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                     break;
            case 11: // RESET ...
                     
                     break;
            case 12:// SET SSP configuration
                    SSP_config.word = args[0]; 
                    break;
            default : break;
            
        }//END switch idx for action commands
         
        // CAN SAFELY RESET TIMEOUT (done all checks on commands and args)
        SACT_flags.timeout = 0;
    }
}//END ExecCommand


/******************************************************************************
 * TIMEOUT MANAGEMENT (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_timeout(void)
{
    if((SACT_state == SACT_BIN_U1)||(SACT_state == SACT_BIN_U2))
    {
        SACT_flags.timeout++;
        if(SACT_flags.timeout > SACT_TIME_LIMIT/SLOW_RATE)
        {
	        SACT_flags.timeout = 0;
            SACT_state = SACT_NOSYNC;
            control_mode.off_mode_req = 1;
        }    
    }
}

/******************************************************************************
 * SENDS Sabot Sensor Packet (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_SendSSP(void)
{
    volatile UART *ureg;
    WRD temp;
    LNG templong;
    uint8_t accum = 0;
    uint8_t count = 0;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;
    
    if(SSP_config.word != 0)
    {
////////////////////////////////////////////////////////////////////////
////////IF SACT state ASCII send human readable data
        if((SACT_state == SACT_ASCII_U1)||(SACT_state == SACT_ASCII_U2))
        {//ASCII MODE
        if(SACT_state == SACT_ASCII_U1)
            ureg = &UART1;
        else //if we are here we have certainly SACT_BIN_U2
            ureg = &UART2;

        temp.i = SSP_config.word;
        putuiUART(temp.ui,ureg);
        putcUART(SC,ureg);

/////////SENSOR DATA
        if(SSP_config.encoders)
        {
            templong.l = mposition1;
            putuiUART(templong.ui[1],ureg);
            //putcUART(VL,ureg);
            putcUART(SC,ureg);
            putuiUART(templong.ui[0],ureg);
            putcUART(SC,ureg);

            templong.l = mposition2;
            putuiUART(templong.ui[1],ureg);
            //putcUART(VL,ureg);
            putcUART(SC,ureg);
            putuiUART(templong.ui[0],ureg);
            putcUART(SC,ureg);
        }// END if encoders

        if(SSP_config.grip_status)
        {
            temp.ui = grip_status_flags.word;
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);
        }// END if grip status

        if(SSP_config.trajectory)
        {
            templong.l = TRAJMotor1.qdPosition;
            putuiUART(templong.ui[1],ureg);
            //putcUART(VL,ureg);
            putcUART(SC,ureg);
            putuiUART(templong.ui[0],ureg);
            putcUART(SC,ureg);

            templong.l = TRAJMotor2.qdPosition;
            putuiUART(templong.ui[1],ureg);
            //putcUART(VL,ureg);
            putcUART(SC,ureg);
            putuiUART(templong.ui[0],ureg);
            putcUART(SC,ureg);
        }// END if trajectory
        
        if(SSP_config.analogs)
        {
            temp.ui = ADResult[0];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);
            
            temp.ui = ADResult[1];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[2];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[3];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[4];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[5];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[6];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);

            temp.ui = ADResult[7];
            putuiUART(temp.ui,ureg);
            putcUART(SC,ureg);
        }// END if analogs

        if(SSP_config.digitals)
        {
            
        }// END if digitals

        if(SSP_config.sonars)
        {
            
        }// END if sonars

        if(SSP_config.currents)
        {
            if(DIR1 == !direction_flags.motor1_dir)
                temp.i = -(mcurrent1_filt - mcurrent1_offset);
            else
                temp.i = mcurrent1_filt - mcurrent1_offset;
            putiUART(temp.i,ureg);
            putcUART(SC,ureg);

            if(DIR2 == !direction_flags.motor2_dir)
                temp.i = -(mcurrent2_filt - mcurrent2_offset);
            else
                temp.i = mcurrent2_filt - mcurrent2_offset;
            putiUART(temp.i,ureg);
            putcUART(SC,ureg);
        }// END if currents

        if(SSP_config.motor_vel)
        {
            temp.i = mvelocity1;
            putiUART(temp.i,ureg);
            putcUART(SC,ureg);
            
            temp.i = mvelocity2;
            putiUART(temp.i,ureg);
            putcUART(SC,ureg);
        }// END if motor vel
     
////////ALL DATA SENT IN ASCII MODE
        putcUART(CR,ureg);putcUART(LF,ureg);        
            
        }// END if SACT_state ASCII..
/////////////////////////////////////////////////////////////////////////
////////IF SACT state BIN send binary data according to SACT protocol
        else if((SACT_state == SACT_BIN_U1)||(SACT_state == SACT_BIN_U2))
        { //BINARY MODE
        if(SACT_state == SACT_BIN_U1)
            ureg = &UART1;
        else //if we are here we have certainly SACT_BIN_U2
            ureg = &UART2;
////////PREPARE CONSTANT PART
        BINTXbuf[0] = SACT_HEAD1;
        BINTXbuf[1] = SACT_HEAD2;
        BINTXbuf[3] = SACT_SSP;
        temp.i = SSP_config.word;
        BINTXbuf[4] = temp.uc[0];
        BINTXbuf[5] = temp.uc[1];

////////PREPARE SENSOR DATA
        if(SSP_config.encoders)
        {
            templong.l = mposition1;
            BINTXbuf[accum+6] = templong.uc[0];
            accum++;
            BINTXbuf[accum+6] = templong.uc[1];
            accum++;
            BINTXbuf[accum+6] = templong.uc[2];
            accum++;
            BINTXbuf[accum+6] = templong.uc[3];
            accum++;

            templong.l = mposition2;
            BINTXbuf[accum+6] = templong.uc[0];
            accum++;
            BINTXbuf[accum+6] = templong.uc[1];
            accum++;
            BINTXbuf[accum+6] = templong.uc[2];
            accum++;
            BINTXbuf[accum+6] = templong.uc[3];
            accum++;
        }// END if encoders

        if(SSP_config.grip_status)
        {
            temp.ui = grip_status_flags.word;
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
        }// END if grip status

        if(SSP_config.trajectory)
        {
            templong.l = TRAJMotor1.qdPosition;
            BINTXbuf[accum+6] = templong.uc[0];
            accum++;
            BINTXbuf[accum+6] = templong.uc[1];
            accum++;
            BINTXbuf[accum+6] = templong.uc[2];
            accum++;
            BINTXbuf[accum+6] = templong.uc[3];
            accum++;

            templong.l = TRAJMotor1.qdPosition;
            BINTXbuf[accum+6] = templong.uc[0];
            accum++;
            BINTXbuf[accum+6] = templong.uc[1];
            accum++;
            BINTXbuf[accum+6] = templong.uc[2];
            accum++;
            BINTXbuf[accum+6] = templong.uc[3];
            accum++;
            
        }// END if trajectory
       
        if(SSP_config.analogs)
        {
            temp.ui = ADResult[0];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
            
            temp.ui = ADResult[1];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[2];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[3];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[4];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[5];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[6];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            temp.ui = ADResult[7];
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
        }// END if analogs

        if(SSP_config.digitals)
        {
            
        }// END if digitals

        if(SSP_config.sonars)
        {
            
        }// END if sonars

        if(SSP_config.currents)
        {
            if(DIR1 == !direction_flags.motor1_dir)
                temp.i = -mcurrent1_filt;
            else
                temp.i = mcurrent1_filt;
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;

            if(DIR2 == !direction_flags.motor2_dir)
                temp.i = -mcurrent2_filt;
            else
                temp.i = mcurrent2_filt;
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
        }// END if currents

        if(SSP_config.motor_vel)
        {
            temp.i = mvelocity1;
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
            
            temp.i = mvelocity2;
            BINTXbuf[accum+6] = temp.uc[0];
            accum++;
            BINTXbuf[accum+6] = temp.uc[1];
            accum++;
            
        }// END if motor vel
       
////////SENSOR DATA PREPARED, proceed with rest
        BINTXbuf[2] = accum+6; //BYTE COUNT
    
////////CALCULATE CRC    
        while(count < (accum+3)) 
        {
            crc_16 = update_crc_16(crc_16,BINTXbuf[count+3]);
            count++;
        }

        temp.ui = crc_16;
        BINTXbuf[accum+6] = temp.uc[0];
        BINTXbuf[accum+7] = temp.uc[1];

////////END OF PACKET
        BINTXbuf[accum+8] = SACT_EOP;

////////SEND PACKET
        SendNUART(BINTXbuf,ureg,accum+9);        
        
     } // END else if SACT_state BIN..
    }//END if config.word != 0
}//END SACT_SenSSP


/******************************************************************************
 * SENDS Sabot Diagnostic Packet (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_SendSDP(void)
{
    static t_status_flags status_flags_prev;
    
    WRD temp;
    uint8_t count = 0;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;

////IF SACT state ASCII send human readable info about error
    if((SACT_state == SACT_ASCII_U1)||(SACT_state == SACT_ASCII_U2))
        {
            if((status_flags.b != status_flags_prev.b)&&(status_flags.b != 0))
            {
                // TODO: better fault messages..
                if(SACT_state == SACT_ASCII_U1)
                    putsUART((unsigned char*)"FAULT DETECTED!\r\n",&UART1);
                else
                    putsUART((unsigned char*)"FAULT DETECTED!\r\n",&UART2);
            }
            status_flags_prev.b = status_flags.b;
            
        }
    else if((SACT_state == SACT_BIN_U1)||(SACT_state == SACT_BIN_U2))
    {    
////////PREPARE CONSTANT PART
        BINTXbuf[0] = SACT_HEAD1;
        BINTXbuf[1] = SACT_HEAD2;
        BINTXbuf[2] = 10;
        BINTXbuf[3] = SACT_SDP;

////////PREPARE DIAG data    
        BINTXbuf[4] = status_flags.b;
        
        BINTXbuf[5] = control_mode.state;

        BINTXbuf[6] = BINLastCommand;

        BINTXbuf[7] = 0; // FIRMWARE REVISION v0.x
        BINTXbuf[8] = 99; // FIRMWARE REVISION vx.99

#ifdef SIMULATE
        BINTXbuf[9] = 0; // BOARD REVISION X
#endif

#ifdef REV1_BOARD 
        BINTXbuf[9] = 1; // BOARD REVISION 1
#endif

#ifdef REV2_BOARD 
        BINTXbuf[9] = 2; // BOARD REVISION 2
#endif

////////CALCULATE CRC    
        while(count < 7) 
        {
            crc_16 = update_crc_16(crc_16,BINTXbuf[count+3]);
            count++;
        }

        temp.ui = crc_16;
        BINTXbuf[10] = temp.uc[0];
        BINTXbuf[11] = temp.uc[1];

////////END OF PACKET
        BINTXbuf[12] = SACT_EOP;

////////SEND PACKET
        if(SACT_state == SACT_BIN_U1)
        {
            SendNUART(BINTXbuf,&UART1,13);
        }
        else //if we are here we have certainly SACT_BIN_U2
        {
            SendNUART(BINTXbuf,&UART2,13);
        }
    }//END ELSE if SACT_state BIN..
}//END SACT_SenSSP

