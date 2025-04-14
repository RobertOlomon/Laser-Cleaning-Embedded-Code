#include "pin_def.hpp"
#include "serial_receiver.hpp"
#include "stepper_motor.hpp"
#include "cleaner_system_constants.hpp"
#include <Arduino.h>

float const defaultSpeed = 100.0;
float const defaultAccel = 1000.0;

StepperMotor stepper(CS_PIN);
SerialReceiver receiver;

void setup() {
  Serial.begin(460800);
  SPI.begin();
  stepper.begin();
}

void loop() {
  receiver.parse();
  switch (receiver.lastReceivedMessageId()) {
  case SerialReceiver::MessageType::COMMAND: {
    // Check if the message is a command type
    SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();
    break;
  }
  }
}