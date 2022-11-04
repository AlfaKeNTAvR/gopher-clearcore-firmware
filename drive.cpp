#include "drive.h"

// Variables
volatile bool manual_brake_disable = false;
bool isHomed = false;
String mode = "none";
volatile bool brakeOn = false;
uint32_t previousMillisCompleted = 0;
uint32_t currentMillisCompleted = 0;
uint32_t delayCompleted = 1000;


////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                           HELPER FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
// UNIT CONVERTER (mm to steps and steps to mm)
double unitConverter(double input, String mode)
{
  // Divide steps per revolution (drive encoder resolution) by mm per revolution (ballscrew drive pitch)
  double mm_to_steps = 80.0;
  double steps_to_mm = 0.0125;

  if(mode == "mm_to_steps") 
  {
    input = input * mm_to_steps;
  }

  else if(mode == "steps_to_mm") 
  {
    input = input * steps_to_mm;
  }

  return input;  
}


////////////////////////////////////////////////////////////////////////////
// CORRECT INPUT VELOCITY
double CorrectVelocityFraction(double vel_frac)
{
  // Should not exceed 1.0
  if(abs(vel_frac) > 1.0) vel_frac = vel_frac / vel_frac;

  return vel_frac;
}


////////////////////////////////////////////////////////////////////////////
// SAFETY CHECK
bool safetyCheck(bool ignore_ishomed)
{
  // Check 1: the motor is not in fail state
  if(motor.HlfbState() == 0)
  {
    Serial.println("Error: Motor is in a fail state! Reapply power to the motor.");
    return false;
  }

  // Check 2: the motor is enabled
  if(!motor.EnableRequest())
  {
    Serial.println("Error: Motor is not enabled!");
    return false;
  }

  // Check 3: the motor is homed
  if(!isHomed && !ignore_ishomed)
  {
    Serial.println("Error: Not homed!");
    return false;
  }

  // Check 4: the brake is not turned-off manually
  if(manual_brake_disable)
  {
    Serial.println("Error: brake is turned-off manually!");
    return false;
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////
// WAIT FOR END OF MOTION
bool motionWait()
{
  // Wait for the movement to complete
  while(!motor.StepsComplete()) continue;
  return true;
}


////////////////////////////////////////////////////////////////////////////
// DRIVE SETUP
bool driveSetup()
{
  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                        Connector::CPM_MODE_STEP_AND_DIR);

  // Set the motor's HLFB mode
  motor.HlfbMode(MotorDriver::HLFB_MODE_STATIC);
  // Set the HFLB carrier frequency to 482 Hz
  //motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motor.VelMax(unitConverter(velocityLimit, "mm_to_steps"));

  // Set the maximum acceleration for each move
  motor.AccelMax(unitConverter(accelerationLimit, "mm_to_steps"));

  drivePower("on", true);

  // Waits for HLFB to assert (waits for homing to complete if applicable)
  Serial.println("Waiting for HLFB...");

  uint32_t timeout = 10000;
  uint32_t startTime = millis();

  // Wait for the motor to initialize
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) 
  { 
      // Check for timeout
      if(millis() - startTime >= timeout)
      {
        Serial.println("Motor setup failure!");
        return false;
      }
  }

  Serial.println("Motor Ready");
  return true;
}


////////////////////////////////////////////////////////////////////////////
// BRAKE POWER CONTROL
void brakeControl(String state, bool manual)
{
  // Check if the motor was disabled
  if(!motor.EnableRequest()) isHomed = false; // Disabled motor + disabled brake requires homing after enabling

  // Turn on the brake
  if(state == "on") 
  {
    if(manual) manual_brake_disable = false;

    digitalWrite(brakePin, LOW);  
    delay(50); // To ensure the brake is ON
    //Serial.println("Brake Enabled");
  }

  // Turn on the brake
  else if(state == "off") 
  {
    if(manual) manual_brake_disable = true;

    digitalWrite(brakePin, HIGH);
    delay(50);  // To ensure the brake is OFF
    //Serial.println("Brake Disabled"); 
  }
    
}


////////////////////////////////////////////////////////////////////////////
// DRIVE POWER CONTROL
bool drivePower(String state, bool ignore_hlfb)
{
  // Check if brake was turned-off
  if(manual_brake_disable == true) isHomed = false; // Disabled motor + disabled brake requires homing after enabling

  // Enable the motor
  if(state == "on")
  {
    // Enables the motor
    motor.EnableRequest(true);
  
    // Check if the motor was actually enabled
    if(!motor.EnableRequest())
    {
      Serial.println("Error: Motor is NOT enabled");
      return false;
    }
    
    Serial.println("Drive Enabled");
  }

  // Disable the motor
  else if(state =="off")
  {
    // Disables the motor
    motor.EnableRequest(false);
  
    // Check if the motor was actually enabled
    if(motor.EnableRequest())
    {
      Serial.println("Error: Drive is STILL enabled");
      return false;
    }
  
    Serial.println("Drive Disabled");    
  }

  // Check if the motor is not in fail state (ignore on start up - motor take time to energize its coils)
  if(motor.HlfbState() == 0 && !ignore_hlfb)
  {
    Serial.println("Error: Motor is in a fail state! Reapply power to the motor.");
    return false;
  }
  
  return true;
}


////////////////////////////////////////////////////////////////////////////
// AUTO BRAKE TURN-ON (while the drive is not moving)
void moveCompleted()
{
  // If motor is not moving and brake is not turned-off manually
  if(motor.StepsComplete() && manual_brake_disable == false)
  {
    if(mode != "none")
    {
      Serial.println("Movement complete!");
      previousMillisCompleted = millis();
      mode = "none";
    }

    if(millis() - previousMillisCompleted >= delayCompleted) brakeControl("on"); 
  }
}


////////////////////////////////////////////////////////////////////////////
// GET CURRENT POSITION
double getPosition()
{
  return unitConverter(motor.PositionRefCommanded(), "steps_to_mm");
}


////////////////////////////////////////////////////////////////////////////
// GET CURRENT VELOCITY
double getVelocity()
{
  return unitConverter(motor.VelocityRefCommanded(), "steps_to_mm");
}


////////////////////////////////////////////////////////////////////////////
// HOMING SEQUENCE
bool homing(double pos_after_homing)
{
  // Remove flag
  isHomed = false;

  // Checks 1-4
  if(safetyCheck(true) == false) return false;

  // Turn off the brake
  brakeControl("off");

  Serial.println("Rapid homing...");

  // RAPID continuos movement towards lower limit switch
  //moveAtVelocity(0.1, true, true);
  motor.MoveVelocity(6000); //2400);

  // Safety timeout timer
  int rapid_delay = 20 * 1000;    // Milliseconds
  int slow_delay = 3 * 1000;      // Milliseconds
  unsigned long current_millis = 0;
  unsigned long previous_millis = millis();

  // Wait for limit switch signal
  while(digitalRead(upperEndstopPin) != 0)
  {
    current_millis = millis();
    if(current_millis - previous_millis >= rapid_delay)
    {
      motor.MoveVelocity(0);
      Serial.println("Error: Rapid homing timeout!");

      return false;
    }
  }

  // Stop continous movement
  motor.MoveVelocity(0);
 
  // Get below the limit switch
  moveRelativePosition(-5, 0.1, true, true);

  // Wait for the movement to complete
  motionWait();

  Serial.println("Precise homing...");
  
  // SLOW continuos movement towards lower limit switch
  motor.MoveVelocity(240);

  // Reset the timer
  previous_millis = millis();

  // Wait for limit switch signal
  while(digitalRead(upperEndstopPin) != 0)
  {
    current_millis = millis();
    if(current_millis - previous_millis >= slow_delay)
    {
      motor.MoveVelocity(0);
      Serial.println("Error: Slow homing timeout!");
      
      return false;
    }
  }

  // Stop continous movement
 motor.MoveVelocity(0);

  // Get below the limit switch
  moveRelativePosition(-10, 0.1, true, true);

  // Wait for the movement to complete
  motionWait();

  // Set current position to 550
  motor.PositionRefSet(unitConverter(pos_after_homing, "mm_to_steps"));
  
  Serial.println("Homed...");

  // Set flag
  isHomed = true;

  return true;
}



////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                         MOVEMENT FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
// ABSOLUTE POSITION MOVEMENT 
bool moveAbsolutePosition(int position, double vel_frac, bool ignore_limits, bool ignore_ishomed) 
{
  // Checks 1-4
  if(safetyCheck(ignore_ishomed) == false) return false;
  
  // Check 5: velocity fraction is not greater than 1.0 
  vel_frac = CorrectVelocityFraction(vel_frac); 
  
  // Check 6: soft limits will not be exceeded
  if(!ignore_limits)
  {
    if(position > positionUpperLimit || position < positionLowerLimit)
    {
      Serial.println("Error: Out of boundaries!");
      return false;
    }
  }

  // Convert units
  position = unitConverter(position, "mm_to_steps");

  // Set velocity
  motor.VelMax(unitConverter(velocityLimit * vel_frac, "mm_to_steps"));

  // Turn off the brake
  brakeControl("off");

  // Command the move of absolute distance
  motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

  // Change the mode
  mode = "abs";

  return true;
}


////////////////////////////////////////////////////////////////////////////
// RELATIVE POSITION MOVEMENT
bool moveRelativePosition(int position, double vel_frac, bool ignore_limits, bool ignore_ishomed) 
{
  // Checks 1-4
  if(safetyCheck(ignore_ishomed) == false) return false;

  // Check 5: velocity fraction is not greater than 1.0 
  vel_frac = CorrectVelocityFraction(vel_frac); 
  
  // Check 6: soft limits will not be exceeded
  if(!ignore_limits)
  {
    double current_position = unitConverter(motor.PositionRefCommanded(), "steps_to_mm");

    if(current_position + position > positionUpperLimit || current_position + position < positionLowerLimit)
    {
      Serial.println("Error: Out of boundaries!");
      return false;
    }
  }

  // Convert units
  position = unitConverter(position, "mm_to_steps");

  // Set velocity
  motor.VelMax(unitConverter(velocityLimit * vel_frac, "mm_to_steps"));

  // Turn off the brake
  brakeControl("off");

  // Command the move of absolute distance
  motor.Move(position, MotorDriver::MOVE_TARGET_REL_END_POSN);

  // Change the mode
  mode = "rel";

  return true;
}


////////////////////////////////////////////////////////////////////////////
// VELOCITY MOVEMENT
bool moveAtVelocity(double vel_frac, bool ignore_limits, bool ignore_ishomed) 
{
  // Checks 1-4
  if(safetyCheck(ignore_ishomed) == false) return false;

  // Check 5: velocity fraction is not greater than 1.0 + additional reduction to 50%
  //vel_frac = CorrectVelocityFraction(vel_frac) * 0.5; 

  // Serial.print("Current position is ");
  // Serial.print(getPosition());
  // Serial.println(" mm");

  // Check 6: soft limits will not be exceeded
  if(!ignore_limits)
  {
    if((getPosition() - positionLowerLimit <= 5 && vel_frac < 0) || (positionUpperLimit - getPosition() <= 5 && vel_frac > 0))
    {
      Serial.println("Error: Out of boundaries!");
      return false;
    }
  }

  // Setup velocity
  int velocity = unitConverter(velocityLimit * vel_frac, "mm_to_steps");

  // Turn off the brake
  brakeControl("off");

  // Command the move
  motor.MoveVelocity(velocity);

  // Change the mode
  mode = "vel";

  return true; 
}


////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                        INTERRUPT FUNCTIONS                             //
//                                                                        //
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// LOWER ENDSTOP INTERRUPT HANDLER
void lowerEndstopInterrupt() 
{
  // Safely decelerate the motor
  motor.MoveStopDecel(accelerationLimit);

  // Ignore during homing
  if(isHomed)
  { 
    Serial.println("Lower endstop reached!");
    moveAbsolutePosition(positionLowerLimit, 0.5, true, false);
  }
}

////////////////////////////////////////////////////////////////////////////
// UPPER ENDSTOP INTERRUPT HANDLER
void upperEndstopInterrupt() 
{
  // Safely decelerate the motor
  motor.MoveStopDecel(accelerationLimit);

  // Ignore during homing
  if(isHomed)
  { 
    Serial.println("Upper endstop reached!");
    moveAbsolutePosition(positionUpperLimit, 0.5, true, false);
  }
}
////////////////////////////////////////////////////////////////////////////
// DRIVE STATUS DATA
bool driveStatus(void *){

    DynamicJsonDocument doc(1024);
    JsonObject status  = doc.to<JsonObject>();
    JsonObject Brake = status.createNestedObject("Brake");
    JsonObject Motor = status.createNestedObject("Motor");
    JsonObject Limits = status.createNestedObject("Limits");
    Brake["Active"] = brakeOn;
    Brake["ABS"] = !manual_brake_disable;
    Motor["Homed"] = isHomed;
    Motor["CurrentPosition"] = getPosition();
    Motor["CurrentVelocity"] = getVelocity();
    Motor["FailedState"] = !motor.HlfbState();
    Motor["Enabled"] = motor.EnableRequest();
    Limits["UpperLimitReached"] = !digitalRead(upperEndstopPin);
    Limits["LowerLimitReached"] = !digitalRead(lowerEndstopPin);
    String output = "";
    
    // if (command == "bd"){
    //   serializeJsonPretty(Brake,output);
    // }

   
    // else if (command =="mtr"){
    // serializeJsonPretty(Motor,output);
    // }
        
   
    // else if(command == "end"){
    //   serializeJsonPretty(Limits,output);
    // }
        
    serializeJson(status,output);
    Serial.print(output);

    return true;

   
   




}
