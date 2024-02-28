#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();
motor wheelsMotorA = motor(PORT6, false);
motor wheelsMotorB = motor(PORT12, true);
//motor_group wheels = motor_group(wheelsMotorA, wheelsMotorB);




// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

int Brain_precision = 0, Console_precision = 0;

float currentRoll, rollError, integral, derivative, ki, kd, kp, target, lastError, pidsum, currentVelocity, targetVelocity, velocityError;
float currentDirection, lastDirection;

float backlash = 1;

// Used to find the format string for printing numbers with the
// desired number of decimal places
const char* printToBrain_numberFormat() {
  // look at the current precision setting to find the format string
  switch(Brain_precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}

// Used to find the format string for printing numbers with the
// desired number of decimal places
const char* printToConsole_numberFormat() {
  // look at the current precision setting to find the format string
  switch(Console_precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}

// "when started" hat block
int whenStarted1() {
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) { task::sleep(50); }
  derivative = 0.0;
  integral = 0.0;
  kp = 3.114;
  ki = 0.230;
  kd = 1.69;
  
  target = 0.0;
  lastError = 0.0;
  targetVelocity = 0.0;
  //wheelsMotorA.setMaxTorque(100.0, percent);
  wheelsMotorA.setStopping(brake);
  wheelsMotorA.setVelocity(0.0, percent);
  wheelsMotorA.spin(forward);

  //wheelsMotorB.setMaxTorque(100.0, percent);
  wheelsMotorB.setStopping(brake);
  wheelsMotorB.setVelocity(0.0, percent);
  wheelsMotorB.spin(reverse);

  lastDirection = 1;
  
  while (true) {

    // Get the current direction of the motor
  //currentDirection = wheels.velocity(rpm) > 0 ? 1 : -1;

  // Check if the direction has changed
  //if (currentDirection != lastDirection) {
    // Adjust the target based on the direction and the backlash
    //if (currentDirection > 0) {
      //target += backlash;
    //} else {
    //  target -= backlash;
    //}
  //}
    currentRoll = BrainInertial.orientation(roll, degrees);
    rollError = target - currentRoll;
    integral = integral + rollError * (1/20.0);
    //derivative = BrainInertial.gyroRate(xaxis, dps);
    derivative = (rollError - lastError) / 20.0;
    lastError = rollError;
    pidsum = (integral * ki) + (derivative * kd) + (rollError * kp);

    
    

    printf("kp = %.3f , ki = %.3f , kd = %.3f , roll = %.3f , pid = %.3f \n",
           static_cast<float>(kp),
           static_cast<float>(ki),
           static_cast<float>(kd),
           static_cast<float>(currentRoll),
           static_cast<float>(pidsum));
    fflush(stdout);

    wheelsMotorA.setVelocity((pidsum * 5.0), rpm);
    wheelsMotorB.setVelocity((pidsum * 5.0), rpm);

    lastDirection = currentDirection;
    wait(20, msec);
  }
  return 0;
}

// "when Controller ButtonEUp pressed" hat block
void onevent_ControllerButtonEUp_pressed_0() {
  kp = kp + 0.02;
}

// "when Controller ButtonEDown pressed" hat block
void onevent_ControllerButtonEDown_pressed_0() {
  kp = kp - 0.02;
}

// "when Controller ButtonLUp pressed" hat block
void onevent_ControllerButtonLUp_pressed_0() {
  ki = ki + 0.01;
}

// "when Controller ButtonLDown pressed" hat block
void onevent_ControllerButtonLDown_pressed_0() {
  ki = ki - 0.01;
}

// "when Controller ButtonRUp pressed" hat block
void onevent_ControllerButtonRUp_pressed_0() {
  kd = kd + 0.01;
}

// "when Controller ButtonRDown pressed" hat block
void onevent_ControllerButtonRDown_pressed_0() {
  kd = kd - 0.01;
}

// "when Controller ButtonFUp pressed" hat block
void onevent_ControllerButtonFUp_pressed_0() {
  target = target + 0.01;
}

// "when Controller ButtonFDown pressed" hat block
void onevent_ControllerButtonFDown_pressed_0() {
  target = target + 0.01;
}


int main() {
  // register event handlers
  Controller.ButtonEUp.pressed(onevent_ControllerButtonEUp_pressed_0);
  Controller.ButtonEDown.pressed(onevent_ControllerButtonEDown_pressed_0);
  Controller.ButtonLUp.pressed(onevent_ControllerButtonLUp_pressed_0);
  Controller.ButtonLDown.pressed(onevent_ControllerButtonLDown_pressed_0);
  Controller.ButtonRUp.pressed(onevent_ControllerButtonRUp_pressed_0);
  Controller.ButtonRDown.pressed(onevent_ControllerButtonRDown_pressed_0);
  Controller.ButtonFUp.pressed(onevent_ControllerButtonFUp_pressed_0);
  Controller.ButtonFDown.pressed(onevent_ControllerButtonFDown_pressed_0);

  wait(15, msec);
  whenStarted1();
}