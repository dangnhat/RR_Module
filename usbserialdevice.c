/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== usbserialdevice.c ========
 */

#include <string.h>
#include <stdbool.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Mailbox.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"

/* USB Reference Module Header file */
#include "USBCDCD.h"

/* RR module */
#include "driverlib.h"

#define TASKSTACKSIZE   512

Task_Struct task0Struct, task1Struct;
Char task0Stack[TASKSTACKSIZE], task1Stack[TASKSTACKSIZE];

#define USB_BUFFER_MAX_SIZE 1024
#define ADC_MSG_SIZE        2

#define MIC_ADC_HDR     0x0000
#define NTC_ADC_HDR     0x4000
#define PZ_ADC_HDR     0x8000

#define DOWN_SAMPLING   14 // 14k/14 = 1k

uint8_t usb_buffer[USB_BUFFER_MAX_SIZE];
bool adc2usb_flag = false;
uint32_t nsamples = 0;

typedef struct MsgObj {
    uint16_t ntc_adc_val;
    uint16_t mic_adc_val;
    uint16_t piezo_adc_val;
} MsgObj;

/* Clock */
void initClocks();

// Desired MCLK frequency

#define MCLK_FREQ_KHZ 25000

// On board crystals frequencies (in Hz)

#define XT1_FREQ 32768
#define XT2_FREQ 4000000

// Crystal frequencies in Hz

#define XT1_KHZ XT1_FREQ/1000
#define XT2_KHZ XT2_FREQ/1000

// Ratio used to set DCO (Digitally Controlled Oscillator)
// Using 1 MHz (XT2/4)

#define MCLK_FLLREF_RATIO MCLK_FREQ_KHZ/(XT2_KHZ/4)

uint32_t mclk = 0;
uint32_t smclk = 0;
uint32_t aclk = 0;

/*
 *  ======== Initialize ADC ========
 */
//******************************************************************************
//!
//!                MSP430F552x
//!             -----------------
//!         /|\|                 |
//!          | |          P6.3/A3|<- Vin0
//!          --|RST       P6.5/A5|<- Vin1
//!            |          P6.7/A7|<- Vin2
//!            |                 |
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC12_A peripheral
//! - GPIO Port peripheral
//! - A3
//! - A5
//! - A7
//!
//******************************************************************************
void init_ADC(void)
{
    //Enable A/D channel inputs
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 +
    GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 +
    GPIO_PIN6 + GPIO_PIN7);

    //Initialize the ADC12_A Module
    /*
     * Base address of ADC12_A Module
     * Use internal ADC12_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC12_A_init(ADC12_A_BASE,
    ADC12_A_SAMPLEHOLDSOURCE_SC,
                 ADC12_A_CLOCKSOURCE_ADC12OSC,
                 ADC12_A_CLOCKDIVIDER_1);

    ADC12_A_enable(ADC12_A_BASE);

    /*
     * Base address of ADC12_A Module
     * For memory buffers 0-7 sample/hold for 96 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Enable Multiple Sampling
     */
    ADC12_A_setupSamplingTimer(ADC12_A_BASE,
                               ADC12_A_CYCLEHOLD_96_CYCLES,
                               ADC12_A_CYCLEHOLD_4_CYCLES,
                               ADC12_A_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffers
    /*
     * Base address of the ADC12_A Module
     * Configure memory buffer 0
     * Map input A3 to memory buffer 0
     * Vref+ = AVcc
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_A_configureMemoryParam param0 = { 0 };
    param0.memoryBufferControlIndex = ADC12_A_MEMORY_0;
    param0.inputSourceSelect = ADC12_A_INPUT_A3;
    param0.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param0.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param0.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE, &param0);

    /*
     * Base address of the ADC12_A Module
     * Configure memory buffer 1
     * Map input A5 to memory buffer 1
     * Vref+ = AVcc
     * Vref- = AVss
     * Memory buffer 1 is not the end of a sequence
     *
     */
    ADC12_A_configureMemoryParam param1 = { 0 };
    param1.memoryBufferControlIndex = ADC12_A_MEMORY_1;
    param1.inputSourceSelect = ADC12_A_INPUT_A5;
    param1.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param1.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param1.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE, &param1);
    /*
     * Base address of the ADC12_A Module
     * Configure memory buffer 2
     * Map input A7 to memory buffer 2
     * Vref+ = AVcc
     * Vref- = AVss
     * Memory buffer 2 is the end of a sequence
     */
    ADC12_A_configureMemoryParam param2 = { 0 };
    param2.memoryBufferControlIndex = ADC12_A_MEMORY_2;
    param2.inputSourceSelect = ADC12_A_INPUT_A7;
    param2.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    param2.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    param2.endOfSequence = ADC12_A_ENDOFSEQUENCE;
    ADC12_A_configureMemory(ADC12_A_BASE, &param2);

    //Enable memory buffer 2 interrupt
    ADC12_A_clearInterrupt(ADC12_A_BASE,
    ADC12IFG2);
    ADC12_A_enableInterrupt(ADC12_A_BASE,
    ADC12IE2);
}

/*
 *  ======== stop_ADC ========
 *  Stop ADC conversion in repeated mode.
 */
void start_ADC(void)
{
    //Enable/Start first sampling and conversion cycle
    /*
     * Base address of ADC12_A Module
     * Start the conversion into memory buffer 0
     * Use the repeated sequence of channels
     */
    ADC12_A_startConversion(ADC12_A_BASE,
                            ADC12_A_MEMORY_0,
                            ADC12_A_REPEATED_SEQOFCHANNELS);
}

/*
 *  ======== stop_ADC ========
 *  Stop ADC conversion.
 */
void stop_ADC(void)
{
    ADC12_A_disableConversions(ADC12_A_BASE, ADC12_A_PREEMPTCONVERSION);
}

/*
 *  ======== get_one_ADC_sequence_reading ========
 *  Read a number of ADC reading sequence from memory starting with ADC12_A_MEMORY_0
 *
 *  Make sure that the buffer has enough spaces for reading.
 */
void get_one_ADC_sequence_reading(uint16_t* buffer)
{
    ADC12_A_startConversion(ADC12_A_BASE,
                            ADC12_A_MEMORY_0,
                            ADC12_A_SEQOFCHANNELS);

    while (ADC12_A_isBusy(ADC12_A_BASE) == ADC12_A_BUSY)
        ;

    buffer[0] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0);
    buffer[1] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_1);
    buffer[2] = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_2);
}

/*
 *  ======== ADC interrupt handler ========
 */
void ADC12ISR(void)
{
    MsgObj  msg;
    static uint16_t down_sample_count = 0;

    switch (__even_in_range(ADC12IV, 34))
    {
    case 0:
        break;   //Vector  0:  No interrupt
    case 2:
        break;   //Vector  2:  ADC overflow
    case 4:
        break;   //Vector  4:  ADC timing overflow
    case 6:
        break;   //Vector  6:  ADC12IFG0
    case 8:
        break;   //Vector  8:  ADC12IFG1
    case 10:

        msg.ntc_adc_val = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0) | NTC_ADC_HDR;
        msg.mic_adc_val = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_1) | MIC_ADC_HDR;
        msg.piezo_adc_val = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_2) | PZ_ADC_HDR;

        // Post the message
        if (Mailbox_getNumFreeMsgs(mailbox0) > 0) {
            Mailbox_post(mailbox0, &msg.mic_adc_val, BIOS_NO_WAIT);
            GPIO_toggle(MSP_EXP430F5529LP_P13);
        }

        down_sample_count = (down_sample_count + 1) % DOWN_SAMPLING;
        if (down_sample_count == 0) {
            if (Mailbox_getNumFreeMsgs(mailbox0) >= 2) {
                Mailbox_post(mailbox0, &msg.ntc_adc_val, BIOS_NO_WAIT);
                Mailbox_post(mailbox0, &msg.piezo_adc_val, BIOS_NO_WAIT);
            }
        }

        break;   //Vector 10:  ADC12IFG2
    case 12:
        break;    //Vector 12:  ADC12IFG3
    case 14:
        break;  //Vector 14:  ADC12IFG4
    case 16:
        break;   //Vector 16:  ADC12IFG5
    case 18:
        break;   //Vector 18:  ADC12IFG6
    case 20:    //Vector 20:  ADC12IFG7
        break;
    case 22:
        break;   //Vector 22:  ADC12IFG8
    case 24:
        break;   //Vector 24:  ADC12IFG9
    case 26:
        break;   //Vector 26:  ADC12IFG10
    case 28:
        break;   //Vector 28:  ADC12IFG11
    case 30:
        break;   //Vector 30:  ADC12IFG12
    case 32:
        break;   //Vector 32:  ADC12IFG13
    case 34:
        break;   //Vector 34:  ADC12IFG14
    default:
        break;
    }
}

/*
 *  ======== millis ========
 *  Get the current time in milliseconds
 */
uint32_t millis(void)
{
    return Clock_getTicks() / (1000 / Clock_tickPeriod);
}

/*
 *  ======== transmitFxn ========
 *  Task to transmit serial data.
 *
 *  This task periodically sends data to the USB host once it's connected.
 */
Void transmitFxn(UArg arg0, UArg arg1)
{
    uint16_t msg;
    uint32_t start_time;
    uint32_t cur_time = 0;
    uint32_t running_time = 10000; //ms
    uint16_t count, nmsgs = 0, index;

    // Init ADC
    init_ADC();

    /* Block while the device is NOT connected to the USB */
    USBCDCD_waitForConnect(BIOS_WAIT_FOREVER);

    GPIO_write(Board_LED0, Board_LED_OFF);
    /* Sleep for 10s */
    Task_sleep(10000);

    GPIO_write(Board_LED0, Board_LED_ON);
    GPIO_write(MSP_EXP430F5529LP_P12, Board_LED_ON);

    // Start ADC
    start_ADC();

    // Mark starting time
    start_time = millis();

    while (true)
    {
        /* Get number of messages currently in mailbox */
        nmsgs = Mailbox_getNumPendingMsgs(mailbox0);
        if (nmsgs > 0) {
            index = 0;
            for (count = 0; count < nmsgs; count++) {
                /* Put data to usb buffer */
                if (index + ADC_MSG_SIZE + 1 < USB_BUFFER_MAX_SIZE) {
                    /* Get data from mailbox */
                    Mailbox_pend(mailbox0, &msg, BIOS_WAIT_FOREVER);
                    memcpy(&usb_buffer[index], &msg, sizeof(msg));
                    index += sizeof(msg);
                }
                else {
                    // Not enough space in USB_BUFFER
                    break;
                }
            }

            /* Send data to USB */
            USBCDCD_sendData((uint8_t*) &usb_buffer, index, BIOS_WAIT_FOREVER);

            /* Toggle IO */
            GPIO_toggle(MSP_EXP430F5529LP_P12);
        }

        /* Check running time */
        if (running_time > 0)
        {
            cur_time = millis();
            if (cur_time - start_time > running_time)
            {
                break;
            }
        }

        /* Send data periodically */
        //Task_sleep(5);
    }

    stop_ADC();
    GPIO_write(Board_LED0, Board_LED_OFF);
    while (1)
    {
        Task_sleep(1000);
    }
}

/*
 *  ======== receiveFxn ========
 *  Task to receive serial data.
 *
 *  This task will receive data when data is available and block while the
 *  device is not connected to the USB host or if no data was received.
 */
Void receiveFxn(UArg arg0, UArg arg1)
{
    unsigned int received;
    unsigned char data[32];

    while (true)
    {

        /* Block while the device is NOT connected to the USB */
        USBCDCD_waitForConnect(BIOS_WAIT_FOREVER);

        received = USBCDCD_receiveData(data, 31, BIOS_WAIT_FOREVER);
        data[received] = '\0';
        //GPIO_toggle(Board_LED1);
        if (received)
        {
            System_printf("Received \"%s\" (%d bytes)\r\n", data, received);
        }

    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Stop watchdog */
    WDT_A_hold(WDT_A_BASE);

    /* Init Clock */
    initClocks();
    aclk=UCS_getACLK();
    mclk=UCS_getMCLK();
    smclk=UCS_getSMCLK();

    /* Construct BIOS objects */
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUSB(Board_USBDEVICE);

    USBCDCD_init();

    /* Construct tx/rx Task threads */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.priority = 2;
    Task_construct(&task0Struct, (Task_FuncPtr) transmitFxn, &taskParams, NULL);

    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr) receiveFxn, &taskParams, NULL);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the USB Serial Device example\nSystem provider is "
                  "set to SysMin. Halt the target to view any SysMin contents"
                  " in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void initClocks()
{
    // Set core power mode
    PMM_setVCore(PMM_CORE_LEVEL_3);

    // Configure pins for crystals
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                               GPIO_PIN4 + GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,
                                                GPIO_PIN5 + GPIO_PIN3);

    // Inform the system of the crystal frequencies
    UCS_setExternalClockSource(XT1_FREQ,  // Frequency of XT1 in Hz.
                               XT2_FREQ   // Frequency of XT2 in Hz.
                                );

    // Initialize the crystals
    UCS_turnOnXT2( // used to be UCS_XT2Start in previous driverlib version
            UCS_XT2_DRIVE_4MHZ_8MHZ);

    UCS_turnOnLFXT1( //used to be UCS_LFXT1Start in previous driverlib version
            UCS_XT1_DRIVE_0,
            UCS_XCAP_3);

    UCS_initClockSignal(UCS_FLLREF,         // The reference for Frequency Locked Loop
                        UCS_XT2CLK_SELECT,  // Select XT2
                        UCS_CLOCK_DIVIDER_4 // The FLL reference will be 1 MHz (4MHz XT2/4)
                        );

    // Start the FLL and let it settle
    // This becomes the MCLCK and SMCLK automatically

    UCS_initFLLSettle(MCLK_FREQ_KHZ, MCLK_FLLREF_RATIO);

    // Optional: set SMCLK to something else than full speed
    UCS_initClockSignal(UCS_SMCLK,
                        UCS_DCOCLKDIV_SELECT,
                        UCS_CLOCK_DIVIDER_1);

    // Set auxiliary clock
    UCS_initClockSignal(UCS_ACLK,
                        UCS_XT1CLK_SELECT,
                        UCS_CLOCK_DIVIDER_1);
}
