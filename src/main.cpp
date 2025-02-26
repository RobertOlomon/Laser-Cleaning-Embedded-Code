#include <Arduino.h>
#include "pin_def_cleaner.hpp"
#include "serial_receiver.hpp"
#include "stepper_command.hpp"

float const maxSpeed = 1000.0;
float const maxAccel = 1000.0;

StepperCommand stepper(stepPin, dirPin);
SerialReceiver receiver;

void setup() {
  Serial.begin(460800);
  stepper.initializeStepper(maxSpeed, maxAccel);
}

void loop() {
  receiver.parse();
  switch (receiver.lastReceivedMessageId()) {
    case SerialReceiver::MessageType::COMMAND: {
      SerialReceiver::CommandMessage msg = receiver.lastReceivedGeneralMessage();
      stepper.moveTo(msg.G0);
      stepper.runStepper();
      break;
    }
    default:
      if (stepper.isRunning()){
        stepper.kill();
      } else{
        digitalWrite(breakPin, HIGH);
      }
      break;
  }
}