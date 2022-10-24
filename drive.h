#pragma once

#include <Arduino.h>
#include "ClearCore.h"

// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// Specify which input pin to read from.
// IO-0 through A-12 are all available as digital inputs.
#define upperEndstopPin DI7
#define lowerEndstopPin DI6
#define brakePin IO5

// Define the velocity and acceleration limits to be used for each move
const int velocityLimit = 100;          // mm per sec
const int accelerationLimit = 300;      // mm per sec^2
const int positionUpperLimit = 440;     // mm
const int positionLowerLimit = 0;       // mm

////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                           HELPER FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////

// UNIT CONVERTER (mm to steps and steps to mm)
double unitConverter(double input, String mode);

// CORRECT INPUT VELOCITY
double CorrectVelocityFraction(double vel_frac);

// SAFETY CHECK
bool safetyCheck();

// WAIT FOR END OF MOTION
bool motionWait();

// DRIVE SETUP
bool driveSetup();

// BRAKE POWER CONTROL
void brakeControl(String state, bool manual=false);

// DRIVE POWER CONTROL
bool drivePower(String state, bool ignore_hlfb=false);

// AUTO BRAKE TURN-ON (while the drive is not moving)
void moveCompleted();

// GET CURRENT POSITION
double getPosition();

// GET CURRENT VELOCITY
double getVelocity();

// HOMING SEQUENCE
bool homing(double pos_after_homing);


////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                         MOVEMENT FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////

// ABSOLUTE POSITION MOVEMENT 
bool moveAbsolutePosition(int position, double vel_frac=1.0, bool ignore_limits=false, bool ignore_ishomed=false);

// RELATIVE POSITION MOVEMENT
bool moveRelativePosition(int position, double vel_frac=1.0, bool ignore_limits=false, bool ignore_ishomed=false);

// VELOCITY MOVEMENT
bool moveAtVelocity(double vel_frac=1.0, bool ignore_limits=false, bool ignore_ishomed=false);


////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                        INTERRUPT FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////

// LOWER ENDSTOP INTERRUPT HANDLER
void lowerEndstopInterrupt();

// UPPER ENDSTOP INTERRUPT HANDLER
void upperEndstopInterrupt();
