

//
// ActionFunctions.c
// ======================================================================
// Functions that define the various mission tasks.
//



// This is a guard condition so that contents of this file are not included
// more than once. (Default guard condition provided by MPLAB X)
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

//Import config functions
#ifndef DEVICE_CONFIG
#define	DEVICE_CONFIG
#include "DeviceConfig.h"
#endif

// Import motor functions
#ifndef MOTOR_FUNCTIONS
#define	MOTOR_FUNCTIONS
#include "MotorFunctions.h"
#endif



//DIST FUNCTIONS
// SIDE UPDATE - Update side filters taking into account previous reads (this is an alpha filter)
int rightDistRead = 0;
int leftDistRead = 0;
int frontDistRead = 0;
void updateDistReads() {
    rightDistRead = DIST_FILTER_GAIN*rightDistRead + (1 - DIST_FILTER_GAIN)*RIGHT_DIST_STATE;
    leftDistRead = DIST_FILTER_GAIN*leftDistRead + (1 - DIST_FILTER_GAIN)*LEFT_DIST_STATE;
    frontDistRead = DIST_FILTER_GAIN*frontDistRead + (1 - DIST_FILTER_GAIN)*FRONT_DIST_STATE;
}


// CHECK LEFT DIST - Check distance on the left distance sensor several times, then average the measures together and return average
int sensorBinsDetected() {
    if(!(RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD)) {
      return FALSE;
    }
    for(int i = 0; i < 400; i++) {
        lineFollower();
    }
    return (RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD);
}



// CHECK RIGHT DIST - Check distance on the right distance sensor several times, then average the measures together and return average
unsigned int canyonWallDetected() {
    if(!(RIGHT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD)) {
      return FALSE;
    }
    for(int i = 0; i < 400; i++);
    return (RIGHT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD);
}

unsigned int canyonExitWallDetected() {
    if(!(RIGHT_DIST_STATE > EXIT_WALL_DETECT_THRESHOLD)) {
      return FALSE;
    }
    for(int i = 0; i < 400; i++);
    return (RIGHT_DIST_STATE > EXIT_WALL_DETECT_THRESHOLD);
}


// SATELLITE
// Find the satellite, then turn on the laser.
void satelliteReport() {
  _RCDIV = 1;
  initializeServoPWM();
  int maxDetectAngle = 0;
  int maxDetectVal = 0;

  for(int angle = SERVO_START_DUTY_CYCLE; angle > SERVO_END_DUTY_CYCLE; angle--) {
    SERVO_DUTY_CYCLE_PWM = angle;
    for(int i = 0; i <= 100; i++); //Delay for servo to adjust
    
    //Satellite detected, activate laser and end program.
    if(ANTENNA_PHOTODIODE_STATE > maxDetectVal) {
      maxDetectAngle = angle;
      maxDetectVal = ANTENNA_PHOTODIODE_STATE;
    }
  }
  
  SERVO_DUTY_CYCLE_PWM = maxDetectAngle + SATELLITE_ANGLE_OFFSET;   //Offset compensates for distance between sensing photodiode and laser.
  for(unsigned long i = 0; i <= 50000; i++); //Delay for servo to adjust
  LASER_ENABLE = ON;
  //_RCDIV = 0;
}


// LINE FOLLOW
// Conduct line following by sensors on either side of the line and adjusting
// to keep both sensors on the line.  Runs only one set of checks and adjustments.
void lineFollower() {
  double currentSpeed;
  //Check right detection
  currentSpeed = getPercentSpeed(LEFT);
  if(MIDDLE_RIGHT_QRD_STATE) {		//Line is not detected, slow down opposite wheel
    if(!FAR_LEFT_QRD_STATE) {         //If the far opposite sensor detects the line, this is a sharp turn
      currentSpeed = MIN_SPEED_CURVE;
    } else if(currentSpeed > MIN_SPEED) {                       //If the far opposite sensor does not detect the line, only adjust speed if above MIN_SPEED
        currentSpeed -= SPEED_CHANGE_INTERVAL;
    }
  } else if(!MIDDLE_RIGHT_QRD_STATE && currentSpeed < MAX_SPEED) {	//Line is detected, speed up opposite wheel
    currentSpeed += SPEED_CHANGE_INTERVAL;
  }
  
  setStepperMotorPercentSpeedOneSide(LEFT, currentSpeed);

  
  //Check right detection
  currentSpeed = getPercentSpeed(RIGHT);
  if(MIDDLE_LEFT_QRD_STATE) {                                   //Line is not detected, slow down opposite wheel
    if(!FAR_RIGHT_QRD_STATE) {                                  //If the far opposite sensor detects the line, this is a sharp turn
      currentSpeed = MIN_SPEED_CURVE;
    } else if(currentSpeed > MIN_SPEED) {
      currentSpeed -= SPEED_CHANGE_INTERVAL;                    //If the far opposite sensor does not detect the line, only adjust speed if above MIN_SPEED
    }
  } else if(!MIDDLE_LEFT_QRD_STATE && currentSpeed < MAX_SPEED) {	//Line is detected, speed up opposite wheel
    currentSpeed += SPEED_CHANGE_INTERVAL;
  }

  
  setStepperMotorPercentSpeedOneSide(RIGHT, currentSpeed);
}


// COLLECT SAMPLE - Percision instructions for collecting the sample
void collectSample() {
  //Line up for sample collection
  moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS - SAMPLE_COLLECT_ALIGN_OFFSET);
  doPerciseTurn(LEFT, STEPS_FOR_90_DEGREE_TURN);
  
  //Collect Sample
  setBothMotorDirection(BACKWARD);
  moveDistSteps(STEPS_TO_SAMPLE_COLLECT);
  stopMotors();
  for(long i = 0; i < 200000; i++); //Delay to give time for sample to move

  
  //Return to line
  enableDrivePWM();
  setBothMotorDirection(FORWARD);
  //moveDistSteps(SAMPLE_COLLECT_RETURN_TO_LINE_STEPS);
  setStepperMotorPercentSpeed(PERCISE_SPEED_PERCENT);
  while(MIDDLE_LEFT_QRD_STATE || MIDDLE_RIGHT_QRD_STATE);
  moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS);
  doPerciseTurn(RIGHT, STEPS_FOR_90_DEGREE_TURN);
}


// DEPOSIT SAMPLE - Percision instructions for depositing a collected sample
void depositSample() {
  //Move forward to the middle of the deposition bins
  //moveDistSteps(STEPS_TO_DEPOSIT_BIN_CENTER);
  
  // Delay long enough for sensors to be out of the grey zone of detection
  for(int i = 0; i < 100; i++) {
      lineFollower();
  }
  
  
  
  // Move past bins
  while((RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD)) {
    lineFollower();
  }
  
  setStepperMotorPercentSpeed(PERCISE_SPEED_PERCENT);
  
  // Move to prepare for deposit.
  moveDistSteps(STEPS_FOR_DEPOSITION_ALIGNMENT);
  
  //Determine which bin to deposit into, default assumption is the sample is black
  enum direction turnDirection = RIGHT;
  if(!SAMPLE_COLOR_DETECT) {
    turnDirection = LEFT;
  }
  
  // Deposit sample
  setBothMotorDirection(BACKWARD);
  doPerciseTurn(turnDirection, STEPS_FOR_45_DEGREE_TURN);
  moveDistSteps(STEPS_FOR_DEPOSITION);
  
  // Delay to allow the sample to fall
  for(long i = 0; i < 150000; i++);
  
  //Return to line
  setBothMotorDirection(FORWARD);
  //setStepperMotorPercentSpeed(PERCISE_SPEED_PERCENT);
  enableDrivePWM();
  
  if(turnDirection == RIGHT) {
      setStepperMotorPercentSpeedOneSide(LEFT, MAX_SPEED);
      setStepperMotorPercentSpeedOneSide(RIGHT, MAX_SPEED);
      while(FAR_LEFT_QRD_STATE);
  } else {
      setStepperMotorPercentSpeedOneSide(LEFT, MAX_SPEED);
      setStepperMotorPercentSpeedOneSide(RIGHT, MAX_SPEED);
      while(FAR_RIGHT_QRD_STATE);
  }
  moveDistSteps(250);
  doPerciseTurn(turnDirection, STEPS_FOR_45_DEGREE_TURN);
  setBothMotorDirection(BACKWARD);
  moveDistSteps(300);
  for(long i = 0; i < 50000; i++);
  setBothMotorDirection(FORWARD);
  enableDrivePWM();
//  for(int i = 0; i < 2000; i++) {
//      lineFollower();
//  }
//  moveDistSteps(DEPOSITION_RETURN_TO_LINE_STEPS);
//  doPerciseTurn(turnDirection, STEPS_FOR_45_DEGREE_TURN);
  
  //Back up until line is relocated
//  setBothMotorDirection(BACKWARD);
  //setStepperMotorPercentSpeed(PERCISE_SPEED_PERCENT);
 // enableDrivePWM();
  //while(MIDDLE_LEFT_QRD_STATE && MIDDLE_RIGHT_QRD_STATE);
//  setBothMotorDirection(FORWARD);
  
  //Back up to edge of tile (in case next tile is a curve)
  //setBothMotorDirection(BACKWARD);
  //moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS);
  //setBothMotorDirection(FORWARD);
  
}


//PICK CANYON DIRECTION - Helper function of the canyon, takes multiple 


//CANYON NAVIGATION - Must only be activated AFTER line is no longer detected.
void canyonNavigation() {
  //Begin driving through canyon
  enableDrivePWM();
  setStepperMotorPercentSpeed(CANYON_SPEED);

  //Do canyon navigation until line is refound.
  while(FAR_RIGHT_QRD_STATE || FAR_LEFT_QRD_STATE) {
    //Update side sensor readings
    updateDistReads();
    
    //When a wall if detected ahead, make a turn
    if(frontDistRead >= FRONT_DIST_THRESHOLD) {
      frontDistRead = 0;    //Reset frontDistRead to avoid an immediate false positive after a turn.
      //if(canyonWallDetected()) {	//If wall on right, turn left
      //setBothMotorDirection(BACKWARD);
      //moveDistSteps(CANYON_BACKUP);
      //setBothMotorDirection(FORWARD);
      if(rightDistRead > leftDistRead) {
        doPerciseTurn(LEFT, STEPS_FOR_90_DEGREE_TURN);
      } else {				//If no wall on right, turn right
        //Do alignment to ensure the right wall stays in range after turn.
        
        
        //Do turn
        doPerciseTurn(RIGHT, STEPS_FOR_90_DEGREE_TURN);
      }

      //Resume driving after turn
      enableDrivePWM();
      setStepperMotorPercentSpeed(CANYON_SPEED);
    }
  }

  //When line is refound, move drive wheels over line and turn whichever direction does not have a wall.
  moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS);
  //if(canyonExitWallDetected()) {
  if(rightDistRead > leftDistRead) {
    doPerciseTurn(LEFT, STEPS_FOR_90_DEGREE_TURN);
  } else {
    doPerciseTurn(RIGHT, STEPS_FOR_90_DEGREE_TURN);
  }

  //Re-enable drive at any speed.
  enableDrivePWM();
  setStepperMotorPercentSpeed(CANYON_SPEED);
}


//EXIT LANDER - When a line to the left is detected, conduct movements to follow that line.
void exitLander() {
  moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS);         //Move forward until drive wheels are over line
  doPerciseTurn(LEFT, STEPS_FOR_90_DEGREE_TURN);	//Turn to onto new line
}


// RETURN TO LANDER - When a line is detected to the left, return to rover.
void returnToLander() {
  moveDistSteps(STEPS_FROM_SENSORS_TO_MOTORS);         //Move forward until drive wheels are over line intersection
  doPerciseTurn(RIGHT, STEPS_FOR_90_DEGREE_TURN);	//Turn to back in
  
  //Back into lander
  setMotorDirection(LEFT, BACKWARD);
  setMotorDirection(RIGHT, BACKWARD);
  moveDistSteps(RETURN_LANDER_STEPS);
  
  //setStepperMotorPercentSpeed(MAX_SPEED);
  
  //while(RIGHT_DIST_STATE > RIGHT_DIST_THRESHOLD);   //Wait until in the lander
  stopMotors();
}














