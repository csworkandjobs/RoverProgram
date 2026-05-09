

//
// ConstantsAndEnums.h
// ======================================================================
// This file contains the constants and enums used across project files.
//



// This is a guard condition so that contents of this file are not included
// more than once. (Default guard condition provided by MPLAB X)
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#include <xc.h> // include processor files - each processor file is guarded.
#endif	/* XC_HEADER_TEMPLATE_H */


// Only include this file once per compilation
#pragma once


// LOGIC DEFINITIONS
// BOOLEAN LOGIC - Define boolean logic.
#define TRUE 1
#define FALSE 0

// ON-OFF LOGIC - Define ON-OFF logic.
#define ON 1
#define OFF 0



// ENUM DEFINITIONS
// DIRECTION - Direction arguments used by various functions
enum direction {LEFT, RIGHT, FORWARD, BACKWARD};



// OSCILLATOR SETUP - Define oscillator and define constants so functions can adjust.
#pragma config FNOSC = FRCDIV
#define OSCILLATOR_FREQUENCY 8000000
#define POSTSCALAR 1
#pragma config OSCIOFNC = OFF		//Disable pin 8 clock output so IO can be used on pin 8
#pragma config SOSCSRC = DIG		//Disable secondary oscillator on Pins 9 and 10 so IO can be used


// MOTOR CONTROL AND PIN ASSIGNMENT
// DRIVE MOTOR CONSTANTS
#define STEPPER_MAX_PWM_FREQUENCY 800	//Approximate max duty cycle in full step
#define STEP_TYPE 2                     //Motors configured for 1/2 step use
#define STEPS_PER_ROTATION 200		//Steps for 1 rotation in full step use
//DRIVE MOTOR DERIVED CONSTANTS
#define MAX_STEP_FREQUENCY STEPPER_MAX_PWM_FREQUENCY*STEP_TYPE //Functional max frequency taking step type into account
#define MAX_STEP_SPEED (OSCILLATOR_FREQUENCY/POSTSCALAR/2) / (STEPPER_MAX_PWM_FREQUENCY*STEP_TYPE) - 1 //Max allowed speed in steps
//WHEEL SPECIFICATON
#define PI 3.141592635
#define WHEEL_DIAMETER 3.5               //Wheel diameter measured in inches
#define WHEEL_CIRCUMFERENCE PI*(WHEEL_DIAMETER/2)*(WHEEL_DIAMETER/2.0) //Wheel circumerence measured in inches

// LEFT MOTOR CONTROL
#define LEFT_STEPPER_DIRECTION _LATA1	//Pin 3
#define LEFT_STEPPER_PERIOD OC2RS       //Pin 4 (Period)
#define LEFT_STEPPER_DUTY_CYCLE OC2R	//Pin 4 (Duty Cycle)
#define LEFT_STEPPER_SETUP_CON1 OC2CON1	//Register reference 1 for Pin 4 PWM config
#define LEFT_STEPPER_SETUP_CON2 OC2CON2 //Register reference 2 for Pin 4 PWM config

// RIGHT MOTOR CONTROL
#define RIGHT_STEPPER_DIRECTION _LATB4	//Pin 9
#define RIGHT_STEPPER_PERIOD OC3RS      //Pin 5 (Period)
#define RIGHT_STEPPER_DUTY_CYCLE OC3R	//Pin 5 (Duty Cycle)
#define RIGHT_STEPPER_SETUP_CON1 OC3CON1//Register reference 1 for Pin 5 PWM config
#define RIGHT_STEPPER_SETUP_CON2 OC3CON2//Register reference 2 for Pin 5 PWM config



// SERVO SPECS AND CONTROL
// SERVO MOTOR CONSTANTS
#define SERVO_MAX_ANGLE 180.0		//Total angle range of servo in degrees
#define SERVO_MAX_PULSE 2.5		//Maximum servo pulse
#define SERVO_MIN_PULSE 0.5		//Minimum servo pulse
#define SERVO_PULSE_RANGE SERVO_MAX_PULSE-SERVO_MIN_PULSE //Calculating max pulse range

// SERVO CONTROL
#define SERVO_PERIOD_PWM OC1RS		//Pin 14 (Period)
#define SERVO_DUTY_CYCLE_PWM OC1R	//Pin 14 (Duty Cycle)
#define SERVO_PULSE_REFRESH_RATE 20.0	//Servo refresh rate (in ms)
#define SERVO_SETUP_CON1 OC1CON1	//Register reference 1 for Pin 14 PWM config
#define SERVO_SETUP_CON2 OC1CON2	//Register reference 2 for Pin 14 PWM config



//SENSOR PIN ASSIGNMENT
// QRD SENSOR PIN ASSIGNMENT
#define MIDDLE_LEFT_QRD_STATE _RA3		//Pin 8
#define MIDDLE_RIGHT_QRD_STATE _RB15	//Pin 18
#define FAR_LEFT_QRD_STATE _RB8 		//Pin 12
#define FAR_RIGHT_QRD_STATE _RB9		//Pin 13
#define SAMPLE_COLOR_DETECT _RB12       //Pin 15
// TRIS ASSIGNMENTS FOR SETTING PINS AS INPUT
#define MIDDLE_LEFT_QRD_TRIS TRISAbits.TRISA3
#define MIDDLE_RIGHT_QRD_TRIS TRISBbits.TRISB15
#define FAR_LEFT_QRD_TRIS TRISBbits.TRISB8
#define FAR_RIGHT_QRD_TRIS TRISBbits.TRISB9
#define SAMPLE_QRD_TRIS TRISBbits.TRISB12


// DISTANCE SENSORS PIN ASSIGNMENT
#define RIGHT_DIST_STATE ADC1BUF13	//Pin 7
#define FRONT_DIST_STATE ADC1BUF10	//Pin 17
#define LEFT_DIST_STATE ADC1BUF4    //Pin 6
// TRIS ASSIGNMENTS FOR SETTING PINS AS INPUT
#define RIGHT_DIST_TRIS TRISAbits.TRISA2
#define FRONT_DIST_TRIS TRISBbits.TRISB14
#define LEFT_DIST_TRIS TRISBbits.TRISB2
// ANALOG ASSIGNMENT FOR DIST PINS
#define RIGHT_DIST_ANS ANSAbits.ANSA2
#define FRONT_DIST_ANS ANSBbits.ANSB14
#define LEFT_DIST_ANS ANSBbits.ANSB2


// INFRARED PHOTODIODE PIN ASSIGNMENT
#define ANTENNA_PHOTODIODE_STATE ADC1BUF11	//Pin 16
#define TRIPWIRE_PHOTODIODE_STATE ADC1BUF0  //Pin 2
// TRIS ASSIGNMENTS FOR SETTING PINS AS INPUT
#define ANTENNA_PHOTO_TRIS TRISBbits.TRISB13
#define TRIPWIRE_PHOTO_TRIS TRISAbits.TRISA0
// ANALOG ASSIGNMENT FOR PHOTODIODE PINS
#define ANTENNA_PHOTO_ANS ANSBbits.ANSB13
#define TRIPWIRE_PHOTO_ANS ANSAbits.ANSA0


// LASER PIN ASSIGNMENT
#define LASER_ENABLE _LATB7		//Pin 11



// FINE-TUNED CONSTANTS - Constants that may need to be tuned to improve performance.
//PHOTODIODE SENSITIVITY
#define RIGHT_PHOTO_THRESHOLD 400		//Analog voltage "ON" state for photodiodes.
//SATELLITE TARGETING CONTROL
#define SATELLITE_MIN_ANGLE 40		//Min servo angle for satellite targeting
#define SATELLITE_MAX_ANGLE 80		//Max servo angle for satellite targeting
#define SATELLITE_ANGLE_OFFSET 90    //Angle offset to ensure that the laser does not end up pointing off the board
#define SERVO_START_DUTY_CYCLE 4500
#define SERVO_END_DUTY_CYCLE 2750
//LINE FOLLOW CONTROL
#define SPEED_CHANGE_INTERVAL 0.00075
#define MIN_SPEED_CURVE 0.075
#define MIN_SPEED 0.25
#define MAX_SPEED 0.4375
//CANYON CONTROL
#define CANYON_SPEED 0.375
#define FRONT_DIST_THRESHOLD 3000
#define SIDE_DIST_CANYON_THRESHOLD 1300
#define CANYON_BACKUP 120         //When taking right turns, back up this many steps to ensure the right wall stays in range
#define EXIT_WALL_DETECT_THRESHOLD 1500 //When exiting the canyon, the distance threshold to look for walls is extended
//PERCISE MOTOR CONTROLS
#define STEPS_FOR_90_DEGREE_TURN 280
#define STEPS_FOR_45_DEGREE_TURN 140
#define PERCISE_SPEED_PERCENT 0.3
#define STEPS_FROM_SENSORS_TO_MOTORS 170 //Number of steps to bring the drive motors where the QRD sensors are
//PERCISE MOTOR DIRECTIONS FOR SAMPLE COLLECTION
#define STEPS_TO_SAMPLE_COLLECT 515      //Number of steps to back into/move away from the sample collection tower
#define SAMPLE_COLLECT_ALIGN_OFFSET 45  //Number of steps distance to offset pulling forward for collection, compensates for tripwire being after collection tower.
#define SAMPLE_COLLECT_RETURN_TO_LINE_STEPS 500   //Steps to return to line after sample collection
//PERCISE MOTOR DIRECTIONS FOR SAMPLE DEPOSITION
#define DEPOSITION_SPEED 0.1           //Speed during deposition]
#define STEPS_FOR_DEPOSITION_ALIGNMENT 480   //Steps needed to travel to align for deposition
#define STEPS_FOR_DEPOSITION 430            //Steps needed to travel to deposit sample after alignment
#define DEPOSITION_RETURN_TO_LINE_STEPS 490       //Steps to return to line (slightly higher than steps to deposit possibly due to getting slighly off angle during deposition)
#define RETURN_TO_TILE_EDGE 170     //Reverse to tile edge after deposition in case next tile is a sharp turn
#define DEPOSITION_DIST_THRESHOLD 2000    //Distance sensitivity for detecting deposition bins
//LANDER MOTOR DIRECTIONS
#define RETURN_LANDER_STEPS 1000            //Steps to back into the lander at end of mission
//DISTANCE SENSOR SAMPLING CONTROLS
#define DIST_SAMPLES_PER_READ 5000
#define DIST_FILTER_GAIN .9




