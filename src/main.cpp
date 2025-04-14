#include <Arduino.h>

#include "cleaner_system_constants.hpp"
#include "serial_receiver.hpp"
#include "stepper_motor.hpp"

enum CleanerOperatorMode
{
    MANUAL = 0,
    AUTO   = 1,
};

constexpr float DEFAULT_SPEED = 100.0f;
constexpr float DEFAULT_ACCEL = 1000.0f;

constexpr int BAUDERATE = 460800;

SerialReceiver receiver;

CleanerOperatorMode cleaner_operator_mode = CleanerOperatorMode::MANUAL;

void setup()
{
    Serial.begin(BAUDERATE);
    SPI.begin();
}

void loop()
{
    switch (cleaner_operator_mode)
    {
        case CleanerOperatorMode::MANUAL:
            /* code */
            break;
        case CleanerOperatorMode::AUTO:
            /* code */
            break;
        default:
            break;
    }
    // get desired system state
    receiver.parse();  // will either update the message or skip if no message is available
    switch (receiver.lastReceivedMessageId())
    {
        case SerialReceiver::MessageType::COMMAND:
        {
            // Check if the message is a command type
            SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();
            break;
        }
    }
    // attempt to reach the desired state
}