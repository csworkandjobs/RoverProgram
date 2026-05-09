

//
// DeviceConfig.c
// =========================================================================
// This file handles all init for the various project components as well as
// any adjustments that need to be applied to config during operation.
//


// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#include <xc.h> // include processor files - each processor file is guarded.
#endif	/* XC_HEADER_TEMPLATE_H */


// Only include this file once per compilation
#pragma once


// Import program constants
#ifndef CONSTANTS_AND_ENUMS
#define	CONSTANTS_AND_ENUMS
#include "ConstantsAndEnums.h"
#endif


//SET POSTSCALAR - Function programs are expecting Postscalar of 1
void setPostscalar() {
    _RCDIV = 0;
}


// CLEAR REGISTERS - Convert all TRIS, ANS, LAT registers to 0 at the start of the program.
void clearRegisters() {
    // Set all registers to 0;
  ANSA = 0;
  ANSB = 0;
  TRISA = 0;
  TRISB = 0;
  LATA = 0;
  LATB = 0;
}


//DRIVE MOTOR CONFIG FUNCTIONS

// INITIALIZE PERCISION DRIVE INTERRUPT
void configPerciseDriveInterrupt() {
  _OC2IE = OFF;		//Disable interrupt
  _OC2IF = FALSE;	//Remove any stored interrupt flag
  _OC2IP = 1;		//Set interrupt priority as 1
}

// INITIALIZE DRIVE MOTORS
void initializeStepperMotors() {
  // Configure left stepper motor for PWM on Pin 4.
  LEFT_STEPPER_SETUP_CON1 = 0x1C06;
  LEFT_STEPPER_SETUP_CON2 = 0x001F;
  LEFT_STEPPER_PERIOD = 0;
  LEFT_STEPPER_DUTY_CYCLE = 0;
  LEFT_STEPPER_DIRECTION = 1;

  // Configure right stepper motor for PWM on Pin 5.
  RIGHT_STEPPER_SETUP_CON1 = 0x1C06;
  RIGHT_STEPPER_SETUP_CON2 = 0x001F;
  RIGHT_STEPPER_PERIOD = 0;
  RIGHT_STEPPER_DUTY_CYCLE = 0;
  RIGHT_STEPPER_DIRECTION = 1;
}

// ENABLE MOTOR PWM - Re-enable PWM if it needs to stop
void enableDrivePWM() {
  // Enable left PWM
  LEFT_STEPPER_SETUP_CON1 = 0x1C06;
  LEFT_STEPPER_SETUP_CON2 = 0x001F;

  // Enable right PWM
  RIGHT_STEPPER_SETUP_CON1 = 0x1C06;
  RIGHT_STEPPER_SETUP_CON2 = 0x001F;
}

//DRIVER MOTOR CLEAR CONFIG - Stops drive motors
void stopMotors() {
  // Reset left PWM config
  LEFT_STEPPER_SETUP_CON1 = 0;
  LEFT_STEPPER_SETUP_CON2 = 0;
  LEFT_STEPPER_PERIOD = 0;
  LEFT_STEPPER_DUTY_CYCLE = 0;

  // Reset right PWM config
  RIGHT_STEPPER_SETUP_CON1 = 0;
  RIGHT_STEPPER_SETUP_CON2 = 0;
  RIGHT_STEPPER_PERIOD = 0;
  RIGHT_STEPPER_DUTY_CYCLE = 0;
}


// SERVO CONFIG FUNCTIONS

// SERVO INIT FUNCTION
void initializeServoPWM() {
  //Slow down clock to talk to servo (is sped back up by clearServoConfig)
  _RCDIV = 1;
  
  // Assign register values to enable PWM on Pin 14.
  SERVO_SETUP_CON1 = 0x1C06;
  SERVO_SETUP_CON2 = 0x001F;

  // Set PWM pulse and default duty cycle.
  //_RCDIV = 1;
  //SERVO_PERIOD_PWM = (OSCILLATOR_FREQUENCY/2/2) / SERVO_PULSE_REFRESH_RATE - 1;
  SERVO_PERIOD_PWM = 39999;
  SERVO_DUTY_CYCLE_PWM = SERVO_START_DUTY_CYCLE;  //180 degrees
  for(unsigned int i = 0; i <= 40000; i++); //Delay for servo to adjust
  //_RCDIV = 0;
}

//SERVO CLEAR CONFIG
void clearServoConfig() {
 // Reset register values for PWM on pin
  SERVO_SETUP_CON1 = 0;
  SERVO_SETUP_CON2 = 0;

  // Reset PWM period and duty cycle.
  //SERVO_PERIOD_PWM = 0;
  //SERVO_DUTY_CYCLE_PWM = 0;
  
  //Speed up clock speed
  _RCDIV = 0;
}



// INITIALIZE SENSORS
void configSensors() {
  // Set sensor pins as input
  MIDDLE_LEFT_QRD_TRIS = 1;
  MIDDLE_RIGHT_QRD_TRIS = 1;
  FAR_LEFT_QRD_TRIS = 1;
  FAR_RIGHT_QRD_TRIS = 1;
  SAMPLE_QRD_TRIS = 1;
  RIGHT_DIST_TRIS = 1;
  FRONT_DIST_TRIS = 1;
  LEFT_DIST_TRIS = 1;
  ANTENNA_PHOTO_TRIS = 1;
  TRIPWIRE_PHOTO_TRIS = 1;

  // ANALOG INPUT ENABLED
  RIGHT_DIST_ANS = 1;
  FRONT_DIST_ANS = 1;
  LEFT_DIST_ANS = 1;
  ANTENNA_PHOTO_ANS = 1;
  TRIPWIRE_PHOTO_ANS = 1;

  // DISABLE LASER
  LASER_ENABLE = OFF;
}


//CONFIG A/D CCONVERSION FOR DISTANCE SENSORS AND PHOTODIODE
void configAd() {
  _ADON = 0;		//Disable A/D during config

  //AD1CON1 Register
  _ADSIDL = 0;		//A/D fontinues in idle mode
  _MODE12 = 1;		//12-bit A/D Operation
  _FORM = 0;		//Unsigned integer output
  _SSRC = 7;		//Auto conversion (internal counter)
  _ASAM = 1;		//Auto sampling

  //AD1CON2 Register
  _PVCFG = 0;		//Use VDD as positive ref voltage
  _NVCFG = 0;		//Use VSS as negative ref voltage
  _BUFREGEN = 1;	//Result appears in buffer location corresponding to channel
  _CSCNA = 1;		//Scans inputs specified in AD1CSSx registers.
  _SMPI = 4;		//Every 3rd conversion sent (number of channels sampled -1) to buffer (5 in use)
  _ALTS = 0;		//Sample MUXA only

  _CN2PDE = 1;

  //AD1CON3 Register
  _ADRC = 0;		//Use system clock
  _SAMC = 0;		//Auto sample every A/D period TAD
  _ADCS = 0b00000010;

  //AD1CSS Registers
  AD1CSSL = 0b0010110000010001;		//Select A/D channels

  _ADON = 1;		//Re-enable A/D
}


// SHORTCUT TO ALL CONFIG FUNCTIONS
void configShortcut() {
  setPostscalar();
  clearRegisters();
  configPerciseDriveInterrupt();
  initializeStepperMotors();
  initializeServoPWM();
  configSensors();
  configAd();
  clearServoConfig();
}




