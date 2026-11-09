/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author Daniel Khrlobian
 *
 */

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL             2100
#define LEFT_TURN_TIME_MS       1280
#define RIGHT_TURN_TIME_MS      1200
#define T_INTERSECTION_TIME_MS  2800
#define PWM_CORRECTION          500
#define PWM_MAJOR_CORRECTION    600

// Initialize a global variable for Timer A1 to keep track of elapsed time in milliseconds
static uint32_t Timer_A1_ms_elapsed = 0;
static uint32_t turn_ms_remaining = 0;

typedef enum {
    CENTER = 0x18,
    SLIGHTLY_LEFT_FROM_CENTER = 0x1C,
    SLIGHTLY_RIGHT_FROM_CENTER = 0x38,
    FAR_LEFT_FROM_CENTER = 0x01,
    FAR_RIGHT_FROM_CENTER = 0x80,
    LEFT_TURN = 0xF8,
    RIGHT_TURN = 0x1F,
    T_INTERSECTION = 0xFF,
    DEAD_END = 0x00
} Line_Position;

void Recenter(uint8_t reflectance_sensor_data);

void Detect_Line_Position(uint8_t reflectance_sensor_data){
    if(turn_ms_remaining > 0){
        return;
    }

    switch(reflectance_sensor_data){
        case CENTER:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_GREEN);
           Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
           break;
        case SLIGHTLY_LEFT_FROM_CENTER:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_YELLOW);
           Motor_Stop();
           Recenter(reflectance_sensor_data);
           break;
        case SLIGHTLY_RIGHT_FROM_CENTER:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_PINK);
           Motor_Stop();
           Recenter(reflectance_sensor_data);
           break;
        case FAR_LEFT_FROM_CENTER:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_WHITE);
           Motor_Stop();
           Recenter(reflectance_sensor_data);
           break;
        case FAR_RIGHT_FROM_CENTER:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_SKY_BLUE);
           Motor_Stop();
           Recenter(reflectance_sensor_data);
           break;
        case LEFT_TURN:
           LED1_Output(RED_LED_ON);
           LED2_Output(RGB_LED_WHITE);
           Clock_Delay1ms(150);
           Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
           turn_ms_remaining = LEFT_TURN_TIME_MS;
           break;
        case RIGHT_TURN:
           LED1_Output(RED_LED_ON);
           LED2_Output(RGB_LED_SKY_BLUE);
           Clock_Delay1ms(150);
           Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
           turn_ms_remaining = RIGHT_TURN_TIME_MS;
           break;
        case T_INTERSECTION:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_BLUE);
           Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
           turn_ms_remaining = T_INTERSECTION_TIME_MS;
           break;
        case DEAD_END:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_RED);
           Motor_Stop();
           break;
        default:
           LED1_Output(RED_LED_OFF);
           LED2_Output(RGB_LED_OFF);
           Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
}

void Recenter(uint8_t reflectance_sensor_data){
    if (reflectance_sensor_data == SLIGHTLY_LEFT_FROM_CENTER){
        Motor_Forward(PWM_NOMINAL+PWM_CORRECTION, PWM_NOMINAL-PWM_CORRECTION);
    }
    else if (reflectance_sensor_data == SLIGHTLY_RIGHT_FROM_CENTER){
        Motor_Forward(PWM_NOMINAL-PWM_CORRECTION, PWM_NOMINAL+PWM_CORRECTION);
    }
    else if (reflectance_sensor_data == FAR_LEFT_FROM_CENTER){
        Motor_Forward(PWM_NOMINAL+PWM_MAJOR_CORRECTION, PWM_NOMINAL-PWM_MAJOR_CORRECTION);
    }
    else if (reflectance_sensor_data == FAR_RIGHT_FROM_CENTER){
        Motor_Forward(PWM_NOMINAL-PWM_MAJOR_CORRECTION, PWM_NOMINAL+PWM_MAJOR_CORRECTION);
    }
}

void Timer_A1_Periodic_Task(void)
{
    // Increment Timer_A1_ms_elaped by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    if (turn_ms_remaining > 0) {
        turn_ms_remaining--;
        if(turn_ms_remaining == 0){
            Motor_Stop();
        }
    }

    // Start the process of reading the reflectance sensor array every 10 ms(i.e. 10, 20, 30, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0){
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor array after 1 ms (i.e. 11, 21, 31, ...)
    if ((Timer_A1_ms_elapsed % 10) == 1){
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
    }
}

int main(void)
{
    // Ensure interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize sensor
    Reflectance_Sensor_Init();

    // Initialize SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 Periodic Interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize Motor
    Motor_Init();

    // Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {

    }
}
