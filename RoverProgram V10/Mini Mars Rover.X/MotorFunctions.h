
//
// MotorFunctions.c
// =========================================================================
// This file handles all actions associated with both stepper motors
// and servo motors.
//


// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef XC_HEADER_TEMPLATE_H
#define XC_HEADER_TEMPLATE_H
#include <xc.h> // include processor files - each processor file is guarded.
#endif  /* XC_HEADER_TEMPLATE_H */


// Only include this file once per compilation
#pragma once


// Import program constants
#ifndef CONSTANTS_AND_ENUMS
#define	CONSTANTS_AND_ENUMS
#include "ConstantsAndEnums.h"
#endif

// Import config functions
#ifndef DEVICE_CONFIG
#define	DEVICE_CONFIG
#include "DeviceConfig.h"
#endif





// PERCISION MOVEMENT INTERRUPT - Current implementation assumes both motors will be moving during percise movement.
unsigned char movementDone = FALSE;
unsigned int stepsTaken = 0;
unsigned int stepsToTake = 0;
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void)
{
  _OC2IF = OFF; //Clear interrupt flag

  //Increment steps then check if movement is complete.
  //If it is, disable the interrupt and mark the movementDone flag for program to continue.
  stepsTaken++;
  if(stepsTaken > stepsToTake) {
    movementDone = TRUE;
    stepsTaken = 0;
    _OC2IE = OFF;
  }
}


//STEPPER MOTOR FUNCTIONS
//SET DIRECTION - Parameters are LEFT or RIGHT motor, and FRONT or BACK direction.
void setMotorDirection(enum direction leftRight, enum direction frontBack) {
  if(leftRight == LEFT) {
    if(frontBack == FORWARD) {
      LEFT_STEPPER_DIRECTION = 1;
    } else {
      LEFT_STEPPER_DIRECTION = 0;
    }
  } else {
    if(frontBack == FORWARD) {
      RIGHT_STEPPER_DIRECTION = 1;
    } else {
      RIGHT_STEPPER_DIRECTION = 0;
    }
  }
}


//SET BOTH MOTOR DIRECTION - Set direction of both left and right motors. 
void setBothMotorDirection(enum direction frontBack) {
  setMotorDirection(RIGHT, frontBack);
  setMotorDirection(LEFT, frontBack);
}


//SET SPEED OF ONE MOTOR - Accepts parameters of motor to adjust and to what percent of max speed to adjust to.
double currentPercentSpeedLeft = 0.3;
double currentPercentSpeedRight = 0.3;
void setStepperMotorPercentSpeedOneSide(enum direction motorSide, double percentSpeed) {
  //Stop movement if percentSpeed is 0.
  //Must be handled here to avoid a divide be zero error.
  if(percentSpeed == 0) {
    if(motorSide == LEFT) {	//Stop left motor
      LEFT_STEPPER_PERIOD = 0;
      LEFT_STEPPER_DUTY_CYCLE = 0;
      currentPercentSpeedLeft = 0;
    } else {		//Stop right motor
      RIGHT_STEPPER_PERIOD = 0;
      RIGHT_STEPPER_DUTY_CYCLE = 0;
      currentPercentSpeedRight = 0;
    }
  }

  //Normal motor speed adjustment (when not stopping motor)
  int periodCountPWM = (int)(1.0/percentSpeed * (double)MAX_STEP_SPEED);
  if(motorSide == LEFT) {	//Change left motor speed
    LEFT_STEPPER_PERIOD = periodCountPWM;
    LEFT_STEPPER_DUTY_CYCLE = LEFT_STEPPER_PERIOD/2;
    currentPercentSpeedLeft = percentSpeed;
  } else {		//Change right motor speed
    RIGHT_STEPPER_PERIOD = periodCountPWM;
    RIGHT_STEPPER_DUTY_CYCLE = RIGHT_STEPPER_PERIOD/2;
    currentPercentSpeedRight = percentSpeed;
  }
}


// SET DRIVE SPEED - Accepts percent of max speed and will enable both motors to drive at that speed.
void setStepperMotorPercentSpeed(double percentSpeed) {
  setStepperMotorPercentSpeedOneSide(LEFT, percentSpeed);
  setStepperMotorPercentSpeedOneSide(RIGHT, percentSpeed);
}


// MOVE STEPS ONE SIDE - Config one motor to drive for a certain number of steps.
void moveDistStepsOneSide(enum direction side, int steps) {
  //Config interrupt
  _OC2IF = 0;
  stepsTaken = 0;
  stepsToTake = steps;
  movementDone = FALSE;
  _OC2IE = ON;

  //Setup drive
  setStepperMotorPercentSpeedOneSide(side, PERCISE_SPEED_PERCENT);
  enableDrivePWM();
}

// MOVE STEPS - Move both drive motors a certain number of steps.
void moveDistSteps(int moveSteps) {
  //Stop motors, then assign movement for each moto
  stopMotors();
  moveDistStepsOneSide(LEFT, moveSteps);
  moveDistStepsOneSide(RIGHT, moveSteps);
  
  //Wait for percision movement to be complete
  while(!movementDone);

  //Stop motors after movement is complete
  stopMotors();
}


// PERCISE TURN - Turn 90 degrees or 45 degrees
// Currently assumes all turns are point turns; both motors move at the same speed but in opposite directions.
void doPerciseTurn(enum direction turnDirection, int degreeSteps) {
  //Get current direction
  unsigned char direction = LEFT_STEPPER_DIRECTION;
  
  //Stop motors long enough to disperse momentum
  stopMotors();
  //for(unsigned int i = 0; i < 60000; i++);
  
  //Reverse the direction of the relevant wheel for the turn
  if(turnDirection == LEFT) {
    LEFT_STEPPER_DIRECTION = !direction;
  } else {
    RIGHT_STEPPER_DIRECTION = !direction;
  }

  //Execute percise turn
  moveDistSteps(degreeSteps);
  
  //Stop motors long enough to disperse momentum
  stopMotors();
  //for(unsigned int i = 0; i < 60000; i++);
  
  //Restore forward movement
  LEFT_STEPPER_DIRECTION = direction;
  RIGHT_STEPPER_DIRECTION = direction;
  enableDrivePWM();
}


// MOVE INCHES ONE SIDE - Take in side and inches to move, then convert to 
// steps to take, then forward that to the relevant function.
void moveDistInchesOneSide(enum direction side, double moveInches) {
  //Convert inches to travel to steps to be taken
  double neededSteps = moveInches / WHEEL_CIRCUMFERENCE * STEPS_PER_ROTATION * STEP_TYPE;

  //Begin taking steps
  moveDistStepsOneSide(side, neededSteps);
}

// MOVE INCHES - Move both motors a given number of inches forward.
void moveDistInches(double moveInches) {
  moveDistInchesOneSide(LEFT, moveInches);
  moveDistInchesOneSide(RIGHT, moveInches);
}



//SERVO FUNCTIONS
//SET SERVO ANGLE - Parameter is desired angle in degrees
void setServoAngle(int moveToAngle) {
  //Change postscalar to avoid integer overflow
  _RCDIV = 1;
  //Calculate new PWM pulse in Hz
  double percentMaxAngle = (double)moveToAngle / SERVO_MAX_ANGLE;
  double newPulse = percentMaxAngle * SERVO_PULSE_RANGE + SERVO_MIN_PULSE;

  //Calculate new PWM pulse in counts and update register
  SERVO_DUTY_CYCLE_PWM = (int)(newPulse/SERVO_PULSE_REFRESH_RATE * SERVO_PERIOD_PWM);
}



// PERCENT SPEED REPORT - Take parameter of direction for which motor is to move and return its speed as percent of max speed.
double getPercentSpeed(enum direction side) {
  if(side == LEFT) {	//If desired motor is the left motor, return left speed.
    return currentPercentSpeedLeft;
  } else {		//If desired motor is the right motor, return right speed.
    return currentPercentSpeedRight;
  }
}


