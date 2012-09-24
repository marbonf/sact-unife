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
 *    Filename:       ADC_DMA.c                                       *
 *    Date:           20/05/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization and ISR for ADC and DMA module.
 *
 **********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "generic_defs.h"
#include "my_fractmath.h" //for RSH
#include "ADC_DMA.h"
#include "PWM.h"
#include "Controls.h"

// LOCALS
int16_t *dma_pointer;

//AUX variable for CH0 A/D result, which is NOT USED
int16_t discard_result;

//per la gestione del DMA
int16_t buffer_dma[DMA_TOTAL_LENGTH] __attribute__((space(dma),aligned(128))); //lunghezza vettore dma

int16_t mcurrent1samp[MCURR_MAV_ORDER],mcurrent2samp[MCURR_MAV_ORDER];
int16_t mcurrent_temp;
uint8_t mcurr_idxtemp;
uint8_t mcurrsampIdx = 0;


// ADC_Init() is used to configure A/D to convert 
// AN0-1 (Current sense) on CH1-CH2
// PWM Special Event is used to provide sampling time delay.
void ADC_Init(void)
{
//AD1CON1
	AD1CON1bits.ADON = 0;   //disattivo modulo ADC
	AD1CON1bits.ADSIDL = 0; //continua anche in stand by
	AD1CON1bits.ADDMABM = 1;//ordine di scrittura dati sul buffer. cosi imposto la scrittura in ordine di conversione

    /*IMPORTANTISSIMO!!!: in scatter/gather mode, la DMA RAM è organizzata in buffer, al max di 128 word, uno per ogni canale ADC
    quindi se uso anche solo 2 canali, tutta la DMARAM è comunque usata per gli adc. MEMORIA ORGANIZZATA IN MODO ASSOLUTAMENTE INEFFICENTE!!!
	E' NECESSARIO usare il Conversion Order Mode, per cui i dati saranno memorizzati in ordine del tipo AN00,AN10,AN20,AN30,AN01,AN11,AN21,AN31.... 
	ES. avendo 4 canali e 256 sample per canale, occuperemp 256+4=2KB di DMA RAM
	*/
	
	AD1CON1bits.AD12B = 0;      //ADC a 10 bit
	AD1CON1bits.FORM = 0b00;    //formato numeri interi
	AD1CON1bits.SSRC = 0b011;   //imposto PWM1 come evento di start sample per ADC1
	AD1CON1bits.SIMSAM = 1;     //Camp parallelo tra CH0,CH1,CH2,CH3
	AD1CON1bits.ASAM = 1;       //inizia la conversione immediatamente dopo la fine del campionamento
	//AD1CON1bits.SAMP è superfluo visto che AD1CON1bits.ASAM è impostato a 1
	//SAMP = 1 -> campiona
	//SAMP = 0 -> converte in digitale
	//AD1CON1bits.DONE controlla che abbia finito la conversione

//AD1CON2
    AD1CON2bits.VCFG = 0b111;    //riferimenti AVdd e AVss
    AD1CON2bits.CSCNA = 0;       //No scan input 
    AD1CON2bits.CHPS = 0b11;     //scelgo i canali da convertire. converto in parallelo CH0,CH1,CH2,CH3
    AD1CON2bits.SMPI = 0b0000;   //Incrementa l'indirizzo della RAM del DMA ogni campione convertito 	
    /*  NB: valido per campionamenti simultanei, solo su un MUX. Se si usa ALT MUX deve essere impostato a 00001 */
    AD1CON2bits.BUFM = 0;       //No split e non inizia sempre a mettere i dati dall'inizio ad ogni nuovo campionemento
    AD1CON2bits.ALTS = 0;       //SOLO MUX A

//AD1CON3
    AD1CON3bits.ADRC = 0;           //clock AD derivata Fcy
    AD1CON3bits.SAMC = 0b00011;	    //Tsample = 3Tad
    AD1CON3bits.ADCS = 0b00000011;  //periodo di conversione del dato (Tad = 4 Tcy)
	
//ADCON4 
    //AD1CON4bits.DMABL = 0b111;    // 128 words per ogni canali analogico
    //NB = serve solo per il gather/scatter mode perchè identifica il numero di valori da scrivere dentro la RAM del DMA per ogni blocco (che è di 128 campioni)

//AD1CHS123
    AD1CHS123bits.CH123SA = 0; //CH1 = AN0, CH2 = AN2. CH3 = AN2
    AD1CHS123bits.CH123SB = 0; //CH1 = AN0, CH2 = AN2. CH3 = AN2, MA NON USATO
	
//AD1CHS0
    AD1CHS0bits.CH0SA = 0b11111; //CH0 = AN31, UNUSED
    AD1CHS0bits.CH0SB = 0b11111; //CH0 = AN31, UNUSED

//AD1CSSH -> Scansione dei pin analogici (HIGH). (di default non scansiona nessun pin)

//AD1CSSL -> Scansione dei pin analogici (LOW). (di default non scansiona nessun pin)
	
//AD1PCFGL
    AD1PCFGL = 0xFFFF;
	    CURRSENSE1_PCFG = ANALOG; 
	    CURRSENSE2_PCFG = ANALOG;

	IPC3bits.AD1IP = 7;
    AD1CON1bits.ADON = 1;   //attivo modulo ADC

    // FOR ARRAY STORAGE OF SAMPLING:
    mcurrsampIdx = 0;

}

/******************************************************************************
Function:     
DMA0_Init(void)
legge dalla periferica ADC (ADC1BUF0) e scrive al DMA RAM 
AMODE: Peripheral Indirect Addressing Mode
MODE: Continuous, Ping-Pong Mode disabled
IRQ: ADC Interrupt
*******************************************************************************/

void DMA0_Init(void)
{
//registro DMA0CON
    DMA0CONbits.CHEN = 0;   //canale disabilitato
    DMA0CONbits.SIZE = 0;   //grandezza dei dati:Word
    DMA0CONbits.DIR  = 0;   //legge l'indirizzo dalla periferica e scrive al DMA 
    DMA0CONbits.HALF = 0;   //inizia l'interrupt quando tutti i dati son stati spostati
    DMA0CONbits.NULLW = 0;  //Operationi Normali 
    DMA0CONbits.AMODE = 0;  //Configura il DMA per modalità ad indrizzamento indiretta
    DMA0CONbits.MODE = 0;   //Azioni Continue, senza modalità Ping-Pong

//registro DMA0REQ
    DMA0REQbits.FORCE = 0;          //trasferimento automatico al DMA  inizializzato dalla domanda del DMA
    DMA0REQbits.IRQSEL = 0b0001101; //Select ADC1 as DMA Request source

//registro DMA0PAD
    DMA0PAD = (int)&ADC1BUF0;
	
//registro DMA0CNT
    DMA0CNT = (DMA_TOTAL_LENGTH - 1);   //4 transfers				

//registro DMA0STA
    DMA0STA = __builtin_dmaoffset(buffer_dma);		

    IPC1bits.DMA0IP = 7;    //priorità 5
    IFS0bits.DMA0IF = 0;    //Clear the DMA interrupt flag bit 
    IEC0bits.DMA0IE = 1;    //Set the DMA interrupt enable bit

    DMA0CONbits.CHEN = 1;   //canale abilitato
}



/************************************************************
* _ADCInterrupt() is the A/D interrupt service routine (ISR).
* The routine must have global scope in order to be an ISR.
* The ISR name is chosen from the device linker script.
************************************************************/
void __attribute__((interrupt,no_auto_psv)) _DMA0Interrupt(void)
{
    dma_pointer = &buffer_dma[0];

#if defined(SIMULATE_FULL) || defined(SIMULATE_BASIC)
    if(control_flags.current_loop_active)
    {
        //mcurrent1 = ((FULL_DUTY - P1DC1) >> 2) + mcurrent1_offset; 
        //mcurrent2 = ((FULL_DUTY - P1DC2) >> 2) + mcurrent2_offset;
        if(rcurrent1 < 0)
            mcurrent1 = -rcurrent1 + mcurrent1_offset;
        else
            mcurrent1 = rcurrent1 + mcurrent1_offset;
        if(rcurrent2 < 0)
            mcurrent2 = -rcurrent2 + mcurrent2_offset;
        else
            mcurrent2 = rcurrent2 + mcurrent2_offset;
    }
    else
    {
        mcurrent1_filt = mcurrent1_offset;
        mcurrent2_filt = mcurrent2_offset;
    }
#else
	
//UNPACK DMA buffer
    discard_result = *dma_pointer;
    dma_pointer++;
    mcurrent1 = *dma_pointer;
    dma_pointer++;
    mcurrent2 = *dma_pointer;
    dma_pointer++;
    discard_result = *dma_pointer;

#endif

    // moving average filtering
    mcurrent1samp[mcurrsampIdx] = mcurrent1;
    mcurrent2samp[mcurrsampIdx] = mcurrent2;
    //update index
    mcurrsampIdx++;
    if(mcurrsampIdx > (MCURR_MAV_ORDER - 1)) mcurrsampIdx = 0;

    //execute CURRENT CONTROL LOOP (if active)
    if(control_flags.current_loop_active && (mcurrsampIdx == 0))
    {
        mcurrent_temp = 0;
        mcurr_idxtemp = 0;
        while(mcurr_idxtemp < MCURR_MAV_ORDER)
        {
            mcurrent_temp += mcurrent1samp[mcurr_idxtemp];
            mcurr_idxtemp++;
        }
        //mcurrent1_filt = (mcurrent_temp >> MCURR_MAV_SHIFT);
        mcurrent1_filt = RSH(mcurrent_temp, MCURR_MAV_SHIFT);

        mcurrent_temp = 0;
        mcurr_idxtemp = 0;
        while(mcurr_idxtemp < MCURR_MAV_ORDER)
        {
            mcurrent_temp += mcurrent2samp[mcurr_idxtemp];
            mcurr_idxtemp++;
        }
        //mcurrent2_filt = (mcurrent_temp >> MCURR_MAV_SHIFT);
        mcurrent2_filt = RSH(mcurrent_temp, MCURR_MAV_SHIFT);

        CurrentLoops();
    }

#ifdef DEVELOP_MODE 
#ifdef LOG_ADCINT
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = rcurrent1;
        if(DIR1)
            dataLOG2[dataLOGIdx] = -(mcurrent1_filt-mcurrent1_offset);
        else
            dataLOG2[dataLOGIdx] = mcurrent1_filt-mcurrent1_offset;
 
       dataLOG3[dataLOGIdx] = rcurrent2;
        if(DIR2)
            dataLOG4[dataLOGIdx] = mcurrent2_filt-mcurrent2_offset;
        else
            dataLOG4[dataLOGIdx] = -(mcurrent2_filt-mcurrent4_offset);

        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
#endif //LOG_ADCINT

#endif

//ricavo offset del vettore del dma
    DMA0STA = __builtin_dmaoffset(buffer_dma); 

//resetta flag di interrupt
	IFS0bits.DMA0IF = 0;
}

