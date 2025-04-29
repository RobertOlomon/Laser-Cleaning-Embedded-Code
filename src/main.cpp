#include <Arduino.h>

#include "cleaner_system.hpp"
#include "macros.hpp"
#include "serial_receiver.hpp"
#include "stepper_motor.hpp"


enum CleanerOperatorMode
{
    MANUAL = 0,
    AUTO   = 1,
};

constexpr int MODE_PIN    = 4;  // Pin for mode control
constexpr int ESTOP_PIN   = 5;  // Pin for emergency stop

constexpr int BAUDERATE = 921600;

SerialReceiver receiver;

CleanerOperatorMode cleaner_operator_mode = CleanerOperatorMode::MANUAL;

Cleaner cleaner_system;

void ESTOP_ISR()
{
    cleaner_system.shutdown();
    analogWrite(LED_RED, 0);      // Turn on red LED to indicate emergency stop
    analogWrite(LED_GREEN, 255);  // Turn off green LED
    analogWrite(LED_BLUE, 255);
}

static bool wasInManualMode = false;

void setup()
{
    pinMode(ESTOP_PIN, INPUT_PULLUP);  // Set ESTOP_PIN as input with pull-up resistor

    noInterrupts();
    attachInterrupt(
        digitalPinToInterrupt(ESTOP_PIN),
        ESTOP_ISR,
        CHANGE);  // Attach interrupt to ESTOP_PIN
    interrupts();

    Serial.begin(BAUDERATE);
    Serial.println("Starting Cleaner System...");
    SPI.begin();

    pinMode(LED_BLUE, OUTPUT);  // Set LED pins as output
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    analogWrite(LED_BLUE, 255);  // Turn off blue LED
    analogWrite(LED_RED, 255);   // Turn off red LED
    analogWrite(LED_GREEN, 0);   // Turn on green LED to indicate system is starting
}

void loop()
{
    cleaner_operator_mode =
        static_cast<CleanerOperatorMode>(digitalRead(MODE_PIN));  // get system mode

    cleaner_operator_mode = CleanerOperatorMode::AUTO;  // for testing purposes

    switch (cleaner_operator_mode)
    {
        case CleanerOperatorMode::MANUAL:
        {

            if (!wasInManualMode)
            {
                // Perform actions needed when switching to MANUAL mode
                cleaner_system.initializeManualMode();
                wasInManualMode = true;
            }
            // get desired system state
            cleaner_system.getDesStateManual();
            // run to the desired state
            cleaner_system.run();
        }
        break;

        case CleanerOperatorMode::AUTO:
        {
            if (wasInManualMode)
            {
                // Perform actions needed when switching to AUTO mode
                cleaner_system.initializeAutoMode();
                wasInManualMode = false;
            }
            // will either update the message or skip if no message is available
            receiver.parse();

            switch (receiver.lastReceivedMessageId())
            {
                case SerialReceiver::MessageType::COMMAND:
                {
                    // If the message is a command type
                    SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();

                    // modify the cleaner system des_state based on the command message
                    cleaner_system.processCommand(msg);

                    // run the cleaner system to the desired state
                    cleaner_system.run();
                }
                break;

                case SerialReceiver::MessageType::STOP:
                {
                    // If the message is a stop type
                    SerialReceiver::Stop msg =
                        receiver.lastReceivedStopMessage();  // read the message just cause?
                    cleaner_system.shutdown();
                    Serial.println("Shutting down cleaner system...");
                }
                break;

                default:
                    break;
            }
        }
        break; // case AUTO

        default:
            break;  // do nothing
    }
}