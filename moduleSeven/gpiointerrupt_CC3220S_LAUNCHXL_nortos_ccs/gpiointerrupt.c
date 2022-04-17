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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

                                        //Global Variables
// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// UART Global Variables
char output[64];
int bytesToSend;

//Task scheduler constants for task periods; ZyBooks 4.1 - figure 4.1.2
const unsigned char tasksNum = 3;
const unsigned long tasksPeriodGCD = 1000000; //driver defaults to timerCallback() 1 second
const unsigned checkTemp_period = 500000; //check temperature every 500ms
const unsigned long setPoint_period = 200000; //check buttons every 200ms
const unsigned long countSeconds_period = 1000000; //count seconds every 1 second

//Task scheduler struct; ZyBooks 4.2 - figure 4.2.1/4.2.7
typedef struct task {
    char state;
    unsigned long period;
    unsigned long elapsedTime;
    char (*TickFct)(char);
} task;
task tasks[3];

int buttonPressed;
int tempSetPoint;
int currentTemp;
int seconds;
unsigned char heat;


                                        //Driver Handles
// Driver Handles - Global variables
UART_Handle uart;
I2C_Handle i2c;

                                        //INIT
// Make sure you call initUART() before calling initI2C
void initUART(void){
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

void initI2C(void){
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL){
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
                while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i){
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else{
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void){
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)){
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80){
            temperature |= 0xF000;
        }
    }
    else{
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

//volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status){
    //ZyBooks 4.3 - figure 4.3.2 Task Scheduler
    unsigned int i = 0;
    for(i = 0; i < tasksNum; ++i){
        if(tasks[i].elapsedTime >= tasks[i].period){
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += tasksPeriodGCD;
    }
}

void initTimer(void){
    Timer_Handle timer0;
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = tasksPeriodGCD; //default
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    buttonPressed = 1; //left button
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    buttonPressed = 2; //right button

}

//state machine states
enum SP_state {SP_start, SP_noChange, SP_tempUp, SP_tempDown} SP_state; //increasing or decreasing temperature set point
char setPoint(char state);
enum T_state {T_start, T_ledOn, T_ledOff} T_state; //temperature controlling LED on/off
char tempLed(char state);
enum S_state {S_start, S_add} S_state; //count seconds since board has been reset
char countSeconds(char state);

                                        //STATE MACHINES
char setPoint(char state){
    switch(buttonPressed){
    case 1: //if button 1 is pressed - left
        state = SP_tempUp; //set state to tempUp
        buttonPressed = 0; //reset button
        break;
    case 2: //if button 2 is pressed - right
        state = SP_tempDown; //set state to tempDown
        buttonPressed = 0; //reset button
        break;
    default:
        state = SP_noChange; //set state to noChange
        break;
    }

    switch(state){
    case SP_start:
        tempSetPoint = 0; //initialize tempSetPoint with 0
        break;
    case SP_noChange: //when state is noChange
        break; //do not change anything when a button has not been clicked
    case SP_tempUp: //when state is tempUp
        tempSetPoint = tempSetPoint += 1; //tempSetPoint increases by 1 degree
        break;
    case SP_tempDown: //when state is tempDown
        tempSetPoint = tempSetPoint -= 1; //tempSetPoint decreases by 1 degree
        break;
    default:
        break;
    }
    return state;
}

char tempLed(char state){
    switch(state){
    case T_start:
    case T_ledOff:
    case T_ledOn:
        if(currentTemp >= tempSetPoint){ //if I2C temp is greater than or equal to set point
            state = T_ledOff; //heat is off
        }
        else{
            state = T_ledOn; //if I2C temp is less than set point, heat is on
        }
    }

    switch(state){
    case T_start:
        break;
    case T_ledOff:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); //red light off
        heat = 0; //heat is off
        break;
    case T_ledOn:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); //red light on
        heat = 1; //heat is on
        break;
    }
    return state;
}


char countSeconds(char state){
    switch(state){
    case S_start:
        state = S_add; //state is set to add
        break;
    case S_add:
        ++seconds; //add second
        break;
    }
    return state;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    //Zybooks 4.2 - figure 4.2.7 task holder
    unsigned int i = 0;

    //set point task scheduler
    tasks[i].state = SP_start;
    tasks[i].period = setPoint_period;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &setPoint;
    ++i;

    //check/led temp task scheduler
    tasks[i].state = T_start;
    tasks[i].period = checkTemp_period;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tempLed;
    ++i;

    //count seconds task scheduler
    tasks[i].state = S_start;
    tasks[i].period = countSeconds_period;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &countSeconds;
    seconds = 0;

    //calling drivers and interrupts
    initUART(); //UART before I2C
    initI2C();
    initTimer();

    while(1){
        currentTemp = readTemp(); //I2C temp
        DISPLAY (snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", currentTemp, tempSetPoint, heat, seconds ))
    }
}
