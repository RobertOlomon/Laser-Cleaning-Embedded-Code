// #include "AccelStepper.h"
// #include "pin_defs.hpp"

// // Define a stepper and the pins it will use
// AccelStepper stepper(AccelStepper::DRIVER, JAW_ROTATION_STEP_PIN ,JAW_ROTATION_DIR_PIN);

// void setup()
// {  
//   // Change these to suit your stepper if you want
//   stepper.setMaxSpeed(1000);
//   stepper.setAcceleration(20);
//   stepper.moveTo(500);
// }

// void loop()
// {
//     // If at the end of travel go to the other end
//     if (stepper.distanceToGo() == 0)
//       stepper.moveTo(-stepper.currentPosition());
//     stepper.run();
//     // delay(5);
//     // Serial.println("WHAT IS HAPPENING!?!?!?");
// }