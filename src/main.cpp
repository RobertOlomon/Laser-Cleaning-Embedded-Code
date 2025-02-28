#include <Arduino.h>
#include "pin_def_cleaner.hpp"
#include "serial_receiver.hpp"
#include "stepper_command.hpp"

float const defaultSpeed = 1000.0;
float const defaultAccel = 1000.0;

StepperCommand stepper(stepPin, dirPin);
SerialReceiver receiver;

void setup() {
  Serial.begin(460800);
  stepper.initializeStepper(defaultSpeed, defaultAccel);
}

void loop() {
  receiver.parse();
  switch (receiver.lastReceivedMessageId()) {
    case SerialReceiver::MessageType::COMMAND: {
      SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();
      if (msg.is_M_command()){
        if (msg.M80){
          stepper.setMaxSpeed(msg.M80);
        }
        if (msg.M17){
          stepper.setAcceleration(msg.M17);
        }
      }
      if (msg.G0.received){
        stepper.moveTo(msg.G0.value);
      }
      if (msg.G1.received){
        stepper.moveTo(msg.G1.value);
      }
      if (msg.G4.received){
        delay(msg.G4.value);
      }
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