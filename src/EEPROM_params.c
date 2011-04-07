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
 *    Filename:       EEPROM_params.c                                 *
 *    Date:           2/4/2011                                        *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the management of parameters saving/restor
 *  from EEPROM
 *
 **********************************************************************/

#include "sys_hw.h"
#include "generic_defs.h"
#include "extern_globals.h"
#include "EEPROM_params.h"
#include "SACT_protocol.h"

#include <libpic30.h>

//Global var definition
uint8_t EEPROM_UpdReq;

//LOCALS
uint16_t _EEDATA(2) EE_is_blank = 0;
uint16_t EE_is_blank_RAM;

//ADDRESS VARIABLES
_prog_addressT EE_addr;
_prog_addressT EE_is_blank_addr;

//PARAMS
uint16_t _EEDATA(2*N_PARAMS) parameters_EEPROM[N_PARAMS];




/*************************************
 * Initialize EEPROM
 *************************************/
void InitEEPROM(void)
{
    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr, parameters_EEPROM);
    
    _init_prog_address(EE_is_blank_addr, EE_is_blank);
    
    /*Copy blank status flag from from DataEEPROM to RAM*/
    _memcpy_p2d16(&EE_is_blank_RAM, EE_is_blank_addr, _EE_WORD);
    
    if(EE_is_blank_RAM != 0x3FFF)
    {
        StoreEEPROMparams();
        
        EE_is_blank_RAM = 0x3FFF;
        _erase_eedata(EE_is_blank_addr, _EE_WORD);
        _wait_eedata();
        _write_eedata_word(EE_is_blank_addr, EE_is_blank_RAM);
        _wait_eedata();
    }
}


/*************************************
 * Loads params from EEPROM to RAM
 *************************************/
void LoadEEPROMparams(void)
{
    /*Copy array "parameters_EEPROM" from DataEEPROM to "parameters_RAM" in RAM*/
    _memcpy_p2d16(parameters_RAM, EE_addr, _EE_ROW);
    _memcpy_p2d16(parameters_RAM+16, EE_addr+32, _EE_ROW);
}


/*************************************
 * Store params from RAM to EEPROM
 *************************************/
void StoreEEPROMparams(void)
{
    /*Erase a row in Data EEPROM at array "parameters_EEPROM" */
    _erase_eedata(EE_addr, _EE_ROW);
    _wait_eedata();
    _erase_eedata(EE_addr+32, _EE_ROW);
    _wait_eedata();

    /*Write a row to Data EEPROM from array "parameters_RAM" */
    _write_eedata_row(EE_addr, parameters_RAM);
    _wait_eedata();
    _write_eedata_row(EE_addr+32, parameters_RAM+16);
    _wait_eedata();
}



