#include <Arduino.h>
#include "macros.hpp"
#include "cleaner_system.hpp"
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
constexpr int ESTOP_PIN   = 5;  // Pin for emergency stop

constexpr float DEFAULT_SPEED = 100.0f;
constexpr float DEFAULT_ACCEL = 1000.0f;

constexpr int BAUDERATE = 921600;

SerialReceiver receiver;

CleanerOperatorMode cleaner_operator_mode = CleanerOperatorMode::MANUAL;

Cleaner cleaner_system;

void ESTOP_ISR()
{
    cleaner_system.shutdown();
    analogWrite(LED_RED, 0);  // Turn on red LED to indicate emergency stop
    analogWrite(LED_GREEN, 255);   // Turn off green LED
    analogWrite(LED_BLUE, 255);
}

void setup()
{
    pinMode(ESTOP_PIN, INPUT_PULLUP);                // Set ESTOP_PIN as input with pull-up resistor
    delay(100);  // Wait for the pin to stabilize
    attachInterrupt(ESTOP_PIN, ESTOP_ISR, FALLING);  // Attach interrupt to ESTOP_PIN

    Serial.begin(BAUDERATE);
    Serial.println("Starting Cleaner System...");
    SPI.begin();
    pinMode(LED_BLUE, OUTPUT);   // Set LED pins as output
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    analogWrite(LED_BLUE, 255);   // Turn off blue LED
    analogWrite(LED_RED, 255);    // Turn off red LED
    analogWrite(LED_GREEN, 0);  // Turn on green LED to indicate system is starting
}

void loop()
{
    // Get cleaner system state
    Cleaner::State test = cleaner_system.getRealState();

    cleaner_operator_mode =
        static_cast<CleanerOperatorMode>(digitalRead(MODE_PIN));  // get system mode

    cleaner_operator_mode = CleanerOperatorMode::AUTO;            // for testing purposes

    switch (cleaner_operator_mode)
    {
        case CleanerOperatorMode::MANUAL:
        {
            // get desired system state
            digitalRead(JOG_PIN);
        }
        break;

        case CleanerOperatorMode::AUTO:
        {
            // get desired system state
            // will either update the message or skip if no message is available
            receiver.parse();
            Serial.println(receiver.lastReceivedMessageId());

            switch (receiver.lastReceivedMessageId())
            {
                case SerialReceiver::MessageType::COMMAND:
                {
                    // Check if the message is a command type
                    SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();

                    // modify the cleaner system state based on the command message

                    if (msg.G0.received)
                    {
                        // Move command, modify the state to the desired state
                        cleaner_system.des_state_.jaw_rotation = msg.G0.a;  // jaw rotation
                        cleaner_system.des_state_.jaw_pos      = msg.G0.c;  // jaw position
                        cleaner_system.des_state_.clamp_pos    = msg.G0.y;  // clamp position
                        msg.G0.received                        = false;  // reset the received
                    }
                    if (msg.G4.received)
                    {
                        // Dwell command, wait for a certain time
                        delay(msg.G4.val);  // kinda sucks it's blocking but good enough for now
                    }
                    if (msg.G28.received)
                    {
                        // Home command, reset the system state to the default state
                        cleaner_system.reset();
                        cleaner_system.home();
                    }
                }
                break;

                case SerialReceiver::MessageType::STOP:
                {
                    // Check if the message is a stop type
                    SerialReceiver::Stop msg = receiver.lastReceivedStopMessage();
                    cleaner_system.shutdown();
                    Serial.println("Shutting down cleaner system...");
                }
                break;

                default:
                    break;
            }
        }
        break;
        default:
            break;  // do nothing
    }
}