
//Import PIC hardware references
#include "xc.h"

//Enable Debug Mode
#pragma config ICS = PGx3


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

// Import motor functions
#ifndef MOTOR_FUNCTIONS
#define	MOTOR_FUNCTIONS
#include "MotorFunctions.h"
#endif

// Import action functions
#ifndef ACTION_FUNCTIONS
#define	ACTION_FUNCTIONS
#include "ActionFunctions.h"
#endif



void testTurns() {
  while(1) {
    doPerciseTurn(RIGHT, STEPS_FOR_90_DEGREE_TURN);
    for(long i = 0; i < 25000; i++);
    doPerciseTurn(LEFT, STEPS_FOR_90_DEGREE_TURN);
    for(long i = 0; i < 25000; i++);
    //doPerciseTurn(RIGHT, STEPS_FOR_45_DEGREE_TURN);
    //doPerciseTurn(LEFT, STEPS_FOR_45_DEGREE_TURN);
  }
}

void testFrontSensor() {
  setStepperMotorPercentSpeed(0.3);
  while(1) {
    if(FRONT_DIST_STATE >= 1967) {
      stopMotors();
    } else if(LEFT_STEPPER_PERIOD == 0) {
        enableDrivePWM();
        setStepperMotorPercentSpeed(0.3);
    }
  }
}

void testRightSensor() {
  setStepperMotorPercentSpeed(0.3);
  while(1) {
    if(RIGHT_DIST_STATE >= 1000) {
      stopMotors();
    } else if(LEFT_STEPPER_PERIOD == 0) {
        enableDrivePWM();
        setStepperMotorPercentSpeed(0.3);
    }
  }
}

void testPhotodiode() {
  while(1) {
    if(ANTENNA_PHOTODIODE_STATE > 1024) {
        setStepperMotorPercentSpeed(1.0);
    } else {
        setStepperMotorPercentSpeed(0.3);
    }
  }
}

void testPhotodiodeAndLaser() {
  stopMotors();
  while(1) {
    if(ANTENNA_PHOTODIODE_STATE > 100) {
        LASER_ENABLE = ON;
    } else {
        LASER_ENABLE = OFF;
    }
  }
}

void testLineFollower() {
    setMotorDirection(LEFT, FORWARD);
    setMotorDirection(RIGHT, FORWARD);
    while(1) {
        lineFollower();
    }
}

void testServo() {
    _RCDIV = 1;
    initializeServoPWM();
    //SERVO_DUTY_CYCLE_PWM = 2750;
    //while(1);
    for(unsigned int j = 0; j <= 60000; j++);
    SERVO_DUTY_CYCLE_PWM = 1000;
    //while(1) {
        for(int i = 4500; i > 2750; i--) {
            SERVO_DUTY_CYCLE_PWM = i;
            for(int j = 0; j <= 100; j++);
        }
    //}
    while(1);
}


//MILESTONE 10 IMPLEMENTATION
void milestone10Code() {
  //Delay to ensure proper setup
  for(long i = 0; i < 400000; i++);
  
  //Find exit - If the middle right QRD and far left QRD both see the line, then the lander exit has been detected
  while(MIDDLE_RIGHT_QRD_STATE || FAR_LEFT_QRD_STATE) {
    lineFollower();
  }
  
  //Return to Lander
  returnToLander();
  satelliteReport();
  
  //Prevent Program from restarting.
  while(1);
}


void testSatelliteTarget() {
    satelliteReport();
    while(1);
}

void testLaser() {
    LASER_ENABLE = ON;
    while(1);
}

void testCanyonNavigation() {
    canyonNavigation();
    stopMotors();
    while(1);
}

void testDistance(int steps) {
    moveDistSteps(steps);
    while(1);
}

void testDeposit() {
    while(RIGHT_DIST_STATE <= DEPOSITION_DIST_THRESHOLD) {
        lineFollower();
    }
    depositSample();
    while(1);
}

void testPickup() {
  while(!(TRIPWIRE_PHOTODIODE_STATE > RIGHT_PHOTO_THRESHOLD)) {
    lineFollower();
  }
  collectSample();
  stopMotors();
  while(1);
}

void testRightPhotodiode() {
    while(!(TRIPWIRE_PHOTODIODE_STATE > RIGHT_PHOTO_THRESHOLD)) {
        lineFollower();
    }
    stopMotors();
    while(1);
}

void totalCanyonTest() {
    while(!(FAR_LEFT_QRD_STATE && MIDDLE_LEFT_QRD_STATE && MIDDLE_RIGHT_QRD_STATE && FAR_RIGHT_QRD_STATE &&   //Check if off line
            (RIGHT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD) && (LEFT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD) &&            //Check if walls detected
            ((RIGHT_DIST_STATE < DEPOSITION_DIST_THRESHOLD) || (LEFT_DIST_STATE < DEPOSITION_DIST_THRESHOLD)))) {
        lineFollower();
    }
    
    canyonNavigation();
    stopMotors();
    while(1);
    
}

void totalDepositionTest() {
    while(!((RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD))) {
        lineFollower();
    }
    depositSample();
    stopMotors();
    while(1);
}


int main(void) {
  //Setup all devices.
  configShortcut();
  //while(1);
  for(long i = 0; i < 400000; i++); //Delay to ensure proper setup
  //while(1);
  
  //setStepperMotorPercentSpeed(0.5);
  //while(1);
  
  //Tests to verify code segments are working properly
  
  //testTurns();          //Test 45 and 90 degree turns
  //testFrontSensor();    //Test that front sensor triggers properly (moves until sensor triggers)
  //testRightSensor();    //Test that right sensor triggers properly (moves until sensor triggers)
  //testPhotodiode();     //Test that photodiode triggers properly (turns whenever photodiode is triggered))
  //testPhotodiodeAndLaser();    //Tests photodiode and laster together
  //testServo();          //Test servo positioning
  //testLaser();
  //testLineFollower();   //Test line follow program
  //testSatelliteTarget();
  //testCanyonNavigation();
  //testDistance(495);
  //testDeposit();        //Test that the sample deposit function works correctly
  //testPickup();         //Test that the sample collection function works properly
  //testRightPhotodiode();    //Test to ensure the right photodiode is detecting
  //totalCanyonTest();    //Test canyon enter from line following, navigation, and exit.
  //totalDepositionTest();    //Test deposition from line follower to detection to return to line.
  
  //milestone10Code();
  
  
  //for(long i = 0; i < 400000; i++); //Delay to ensure proper setup
  
  // FINAL CODE IMPLEMENTATION
  //Line follow until all sensors see the line, which indicates the line intersection has been reached.
  while(FAR_LEFT_QRD_STATE || MIDDLE_RIGHT_QRD_STATE) {
    lineFollower();
  }
  
  exitLander();
  
  //Main operation loop
  unsigned char canyonComplete = FALSE;
  unsigned char sampleCollected = FALSE;
  unsigned char sampleDeposited = FALSE;
  int depositionVerification = 0;
  while(1) {
    lineFollower();
    
    //Canyon Navigation - When all QRDs lose light of the line, the canyon has been reached
    //if(!canyonComplete && FAR_LEFT_QRD_STATE && MIDDLE_LEFT_QRD_STATE && MIDDLE_RIGHT_QRD_STATE && FAR_RIGHT_QRD_STATE && (FRONT_DIST_STATE > FRONT_DIST_THRESHOLD)) {
    if(!canyonComplete && FAR_LEFT_QRD_STATE && MIDDLE_LEFT_QRD_STATE && MIDDLE_RIGHT_QRD_STATE && FAR_RIGHT_QRD_STATE &&   //Check if off line
            (RIGHT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD) && (LEFT_DIST_STATE > SIDE_DIST_CANYON_THRESHOLD) &&            //Check if walls detected
            ((RIGHT_DIST_STATE < DEPOSITION_DIST_THRESHOLD) || (LEFT_DIST_STATE < DEPOSITION_DIST_THRESHOLD))) {              //Verify walls are not deposition bins (deposition bins will always detect both left and right as above DEPOSITION_DIST_THRESHOLD)
      canyonNavigation();
      canyonComplete = TRUE;
    }
    
    // Sample Pickup
    if(!sampleCollected && (TRIPWIRE_PHOTODIODE_STATE > RIGHT_PHOTO_THRESHOLD)) {
      collectSample();
      sampleCollected = TRUE;
    }
    
    //Sample Deposition - Deposit after sample is collected and the deposition bin is detected
    if(sampleCollected && !sampleDeposited && (RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD)) {
      for(int i = 0; i < 400; i++);
        if(sampleCollected && !sampleDeposited && (RIGHT_DIST_STATE > DEPOSITION_DIST_THRESHOLD) && (LEFT_DIST_STATE > DEPOSITION_DIST_THRESHOLD)) {
          depositSample();
          sampleDeposited = TRUE;
        }

    }
    
    //Return to Lander - If the middle right QRD and far left QRD both see the line, then the lander exit has been detected
    if(canyonComplete && sampleDeposited && !FAR_LEFT_QRD_STATE && !MIDDLE_RIGHT_QRD_STATE && !MIDDLE_LEFT_QRD_STATE) {
      for(int i = 0; i < 100; i++) {
        lineFollower();
      }
      returnToLander();
      satelliteReport();
      break;
    }
  }
  
  //Prevent code from restarting
  //while(1);
  
  //Delay code restart
  for(long i = 0; i < 400000; i++); //Delay to ensure proper setup
}


