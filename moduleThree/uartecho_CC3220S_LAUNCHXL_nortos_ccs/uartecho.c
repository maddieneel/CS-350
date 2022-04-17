/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== uartecho.c ========
 */
//#include <stdint.h>
//#include <stddef.h>
//
///* Driver Header files */
//#include <ti/drivers/GPIO.h>
//#include <ti/drivers/UART.h>
//
///* Driver configuration */
//#include "ti_drivers_config.h"
//
///*
// *  ======== mainThread ========
// */
//void *mainThread(void *arg0)
//{
//    char        input;
//    const char  echoPrompt[] = "Echoing characters:\r\n";
//    UART_Handle uart;
//    UART_Params uartParams;
//
//    /* Call driver init functions */
//    GPIO_init();
//    UART_init();
//
//    /* Configure the LED pin */
//    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//
//    /* Create a UART with data processing off. */
//    UART_Params_init(&uartParams);
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readReturnMode = UART_RETURN_FULL;
//    uartParams.baudRate = 115200;
//
//    uart = UART_open(CONFIG_UART_0, &uartParams);
//
//    if (uart == NULL) {
//        /* UART_open() failed */
//        while (1);
//    }
//
//    /* Turn on user LED to indicate successful initialization */
//    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
//
//    UART_write(uart, echoPrompt, sizeof(echoPrompt));
//
//
//    /* Loop forever echoing */
//    while (1) {
//        UART_read(uart, &input, 1);
//        UART_write(uart, &input, 1);
//    }
//}


#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

//zyBooks 3.4 implementing an SM in C
enum state {LED_SMStart, state_O, state_N, state_F, ledOn, ledOff} state;
void LedController(input){
    switch(input){ //transitions
        case LED_SMStart: //initial transition
            state = ledOn; //set state to ledOn
            break;
        case 'O':
            state = state_O; //set state to O
            break;
        case 'N':
            if (state == state_O){ //if state is already O - i.e. ON
                state = ledOn; //set state to ledOn
            }
            break;
        case 'F':
            if(state == state_O){ //if state is O
                state = state_F; //set state to F - i.e. OF
            }
            else if(state == state_F){ //if state is already F - i.e. OFF
                state = ledOff; //set state to ledOff
            }
            break;
        default: //if any other variations or different letters
            break;
        }

        switch(state){ //state actions
        case ledOn:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); //turn led on
            break;
        case ledOff:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); //turn led off
            break;
        default:
            break;
        }
};

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;

    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_write(uart, echoPrompt, sizeof(echoPrompt));


    state = LED_SMStart; //indicates initial call

    /* Loop forever echoing */
    while (1) {
        UART_read(uart, &input, 1);
        LedController(input);
        UART_write(uart, &input, 1);
    }
}
