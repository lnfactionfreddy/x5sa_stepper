#include <Arduino.h>

/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */

#include "../lib/pins_CHITU3D_common.h"
#include "AccelStepper.h"
#include <MultiStepper.h>


#define LED              PD12
constexpr uint32_t steps_per_mm = 3200;

AccelStepper y_stepper = AccelStepper(y_stepper.DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper x_stepper = AccelStepper(x_stepper.DRIVER, X_STEP_PIN, X_DIR_PIN);
MultiStepper steppers;
void setup() {
  
  y_stepper.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ true);
  x_stepper.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ true);
  y_stepper.setEnablePin(Y_ENABLE_PIN);
  x_stepper.setEnablePin(X_ENABLE_PIN);
  y_stepper.enableOutputs();
  x_stepper.enableOutputs();
  y_stepper.setMaxSpeed(9*steps_per_mm);
  y_stepper.setAcceleration(5*steps_per_mm);
  x_stepper.setMaxSpeed(9*steps_per_mm);
  x_stepper.setAcceleration(5*steps_per_mm);
  // Then give them to MultiStepper to manage
  steppers.addStepper(x_stepper);
  steppers.addStepper(y_stepper);
}

void loop() {
  long positions[2]; // Array of desired stepper positions
  
  positions[0] = 0;
  positions[1] = 20000;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  
  // Move to a different coordinate
  positions[0] = -20000;
  positions[1] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);

}
