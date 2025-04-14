#include <Arduino.h>

#include "serial_receiver.hpp"
#include "stepper_motor.hpp"

enum CleanerOperatorMode
{
    MANUAL = 0,
    AUTO   = 1,
};

constexpr int JOG_PIN     = 2;  // Pin for jog control
constexpr int JOG_DIR_PIN = 3;  // Pin for jog direction control
constexpr int MODE_PIN    = 4;  // Pin for mode control

constexpr float DEFAULT_SPEED = 100.0f;
constexpr float DEFAULT_ACCEL = 1000.0f;

constexpr int BAUDERATE = 115200;

SerialReceiver receiver;

CleanerOperatorMode cleaner_operator_mode = CleanerOperatorMode::MANUAL;

void setup()
{
    Serial.begin(BAUDERATE);
    Serial.println("Starting Cleaner System...");
    SPI.begin();
}

void loop()
{
    cleaner_operator_mode =
        static_cast<CleanerOperatorMode>(digitalRead(MODE_PIN));  // get system mode
    cleaner_operator_mode = CleanerOperatorMode::AUTO;            // for testing purposes

    switch (cleaner_operator_mode)
    {
        case CleanerOperatorMode::MANUAL:
            // get desired system state
            digitalRead(JOG_PIN);
            break;

        case CleanerOperatorMode::AUTO:

            // get desired system state
            // will either update the message or skip if no message is available
            receiver.parse();
            Serial.println("Parsing message...");
            Serial.println(receiver.lastReceivedMessageId());
            switch (receiver.lastReceivedMessageId())
            {
                case SerialReceiver::MessageType::COMMAND:
                {
                    // Check if the message is a command type
                    SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();

                    if (msg.G0.value != 0)
                    {
                        // Handle G0 command
                        Serial.print("G0 command received with value: ");
                        Serial.println(msg.G0.value);
                    }

                    break;
                }
                break;

                default:
                    break;
            }
            break;

        default:
            break;  // do nothing
    }
}