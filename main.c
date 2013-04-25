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
#include "ADC.h"
#include "TIMER.h"
#include "RPG.h"

int main(void)
{
    int i = 0;

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
    initializeADC();
    initializeRPG();
    configureInterrupts();
    // enable interrupts
    INTEnableInterrupts();

    setup_SDSPI();
    SD_setStart();
    /* Fill tempBuffer[] with int 0 to 63
     * Write it to the current block.
     * Empty tempBuffer[] to all 0.
     * Read from the current block to make sure that it returns the right value.
     */
    fillTempBuffer();
    testSDReadWrite(tempBuffer);

    curr_read_block = curr_block;

    ConfigTimer1(); // Enable Timer1 for second counts
    configureInterrupts();
    // Now blink all LEDs ON/OFF forever.
    while(1)
    {
        if(controlLCDFlag == 1) {
            controlLCDFlag = 0;
            LCDMenuControl();
        }
        if (getPrintToUARTFlag() == 1){
            mPORTEToggleBits(BIT_5);
            

            //mPORTAToggleBits( LED_MASK );
            convertAndPrintIntegerToString("i => ", i++);
            /*
            convertAndPrintIntegerToString("timeElapse => ", timeElapsed);
            convertAndPrintIntegerToString("timeElapsedLEDSample => ", timeElapsedLEDSample);
            convertAndPrintIntegerToString("timeElapsedLEDTurnedOff => ", timeElapsedLEDTurnedOff);
            convertAndPrintIntegerToString("sampleLEDNow => ", sampleLEDNow);
            */
            //convertAndPrintIntegerToString(" ADC Value (Channel 5 WRONG!) => ", getChannel5Value());
            convertAndPrintIntegerToString(" ADC Value => ", getChannel4Value());
            
            printShadowDetect();
            printLightLevel();
            drawLightDetectedBar();
            controlPowerRelay();

            
            switch(curr_state) {
            case READY     : WriteString(" State => READY     ");
                             break;
            case SLEEP     : WriteString(" State => SLEEP     ");
                             break;
            case HIBERNATE : WriteString(" State => HIBERNATE ");
                             break;
            case BUSY      : WriteString(" State => BUSY      ");
                             break;
            }
            
            WriteString("\r");

            setPrintToUARTFlag(0);
        }
        if (NEW_BYTE_RECEIVED == 1){
            curr_state = READY;
            NEW_BYTE_RECEIVED = 0;
            //mPORTAToggleBits( LED_MASK );
            //char tempArray[] = "g";
            //tempArray[0] = characterByteReceived;
            //WriteString(tempArray);
            if(curr_state = HIBERNATE) {
               addByteToBuffer(characterByteReceived, 0);
            }
            else {
                PutCharacter(characterByteReceived);
            }
        }
        if(bufferIndex == 512) {
            SDWriteBlock(currBlock);
            currBlock++;
            if(curr_block > 55000) {
               curr_block = 0;
            }
            bufferIndex = 0;
        }
         if((curr_state == READY) && (timeElapsed >= SLEEP_TIMEOUT) && (timeElapsed < HIBERNATE_TIMEOUT)) {
             curr_state = SLEEP;
         }
         else if((curr_state == SLEEP) && (timeElapsed >= HIBERNATE_TIMEOUT)) {
             curr_state = HIBERNATE;
             timeElapsed = 0;
         }
        if (transmitDataFromSDCard == 1) {
            transmitDataFromSDCard = 0;
            forwardDataToPrinter();
            bufferIndex = 0;
        }
    }
}
