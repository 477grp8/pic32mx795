/*********************************************************************
 *
 *      Example Binky LEDs
 *
 *********************************************************************
 * FileName:        main.c
 * Dependencies:    plib.h
 *
 * Processor:       PIC32MX
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the �Company�) for its PIC32MX Microcontroller is intended
 * and supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip Microcontroller products.
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *********************************************************************
 * This program uses Explorer-16 to blink all of its LEDs at once.
 *
 * Platform:     Explorer-16 with PIC32MX PIM
 *
 ********************************************************************/
#include <plib.h>
#include <p32xxxx.h>
#include "CONFIG.h"
#include "UART.h"
#include "LCD.h"
#include "SDCARD.h"
//#include "TIMER.h"

int main(void)
{
    int i;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);


    // Explorer16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);

    // Make all lower 8-bits of PORTA as output. Turn them off before changing
    // direction so that we don't have unexpected flashes

    mPORTESetPinsDigitalOut( BIT_7 | BIT_6 | BIT_5 );
    mPORTESetBits(BIT_7 | BIT_6 | BIT_5);


    initializeLCD();
    initializeUART();
    configureInterrupts();
    // enable interrupts
    INTEnableInterrupts();
    InitSPI();
    testSPI();
    setup_SDSPI();
    SD_setStart();
    /* Fill tempBuffer[] with int 0 to 63
     * Write it to the current block.
     * Empty tempBuffer[] to all 0.
     * Read from the current block to make sure that it returns the right value.
     */
    fillTempBuffer();
    testSDReadWrite(tempBuffer);

    //curr_read_block = curr_block;

   // ConfigTimer1(); // Enable Timer1 for second counts
   // configureInterrupts();
    // Now blink all LEDs ON/OFF forever.
    while(1)
    {

            // Insert some delay
            i = 100000;
            while(i--) {
                if(i == 0) {
                  mPORTEToggleBits(BIT_7 | BIT_6 | BIT_5);
                 // mPORTDToggleBits(BIT_1 | BIT_2 | BIT_3 | BIT_9);
                    WriteString("test");
                }
            };
    }
}
