# ECE 528/L - Robotics and Embedded Systems with Lab
**CSU Northridge**

**Department of Electrical and Computer Engineering**

# Pre-Lab
# Question 1
void Chassis_Board_LEDs_Init()
{
    P8->SEL0 &= ~0xE1;
    P8->SEL1 &= ~0xE1;
    P8->OUT  &= ~0xE1;
    P8->DIR  |=  0xE1;
}

# Question 2
void Print_Tenth()
{
    for (int i = 10; i <= 51; i++) {
        if (i % 10 == 0) {
            printf("Index %d: Every 10th iteration.\n", i);
        } else if (i % 10 == 1) {
            printf("Index %d: Following the 10th iteration.\n", i);
        }
    }
}

# Overview
This lab involves the MSP432 and its eight-channel QTRX reflectance sensor array to follow a track of black tape along a white surface. The sensors signals determine what position it is in while sending commands in response to those positions such right, left, or dead ends. SysTick and Timer_A1 interrupts support periodic tasks for sensing and control, leaving the main while loop empty. The LEDs are also programmed to enable when the robot is performing the actions. The ultimate goal is for the robot to follow a line maze with a T-intersection and move accordingly.

# Components Used
- MSP432 LaunchPad
- USB-A to Micro-USB Cable
- TI-RSLK MAX Chassis
- 8-Channel QTRX Sensor Array
- Black Tape

# Analysis and Results
The robot followed the line accordingly in the presence of the Professor. All LEDs and functionalities operated correctly and the robot moved correctly according to the line maze.

# Known Issues or Limitations
At certain runs the robot would misinterpret certain positions but other runs it was read correctly. Could amount to the sensitivity of the sensor array as the commands performed correct actions. Besides that, there was no other issue that I witnessed.

# References
- TI DRV8838 Motor Driver Datasheet
- 8-Channel QTRX Sensor Array Product Page
- MSP432P4xx SimpleLink Microcontrollers Technical Reference Manual