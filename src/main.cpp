#include <Arduino.h>

#include "cleaner_system.hpp"
#include "macros.hpp"
#include "serial_receiver_transmitter.hpp"
#include "stepper_motor.hpp"

constexpr int BAUDERATE = 921600;

SerialReceiverTransmitter receiver;

Cleaner cleaner_system;

static bool wasInManualMode = false;
static bool wasInDebugMode  = false;

/**
 * @brief Emergency Stop Interrupt Service Routine.
 *
 * This ISR is triggered when an emergency stop condition occurs.
 * It shuts down the cleaner system and updates the LED indicators:
 * - Turns on the red LED to signal an emergency stop.
 * - Turns off the green and blue LEDs.
 */
void ESTOP_ISR()
{
    cleaner_system.shutdown();
    analogWrite(LED_RED, 0);      // Turn on red LED to indicate emergency stop
    analogWrite(LED_GREEN, 255);  // Turn off green LED
    analogWrite(LED_BLUE, 255);
}

void debugLed()
{
    analogWrite(LED_RED, 0);
    analogWrite(LED_GREEN, 0);
    analogWrite(LED_BLUE, 0);
}

Cleaner::CleanerOperatorMode cleaner_operator_mode;

void setup()
{
    pinMode(ESTOP_PIN, INPUT_PULLUP);  // Set ESTOP_PIN as input with pull-up resistor

    noInterrupts();
    attachInterrupt(
        digitalPinToInterrupt(ESTOP_PIN),
        ESTOP_ISR,
        CHANGE);  // Attach interrupt to ESTOP_PIN
    interrupts();

    receiver.begin(BAUDERATE);
    // Serial.println("Starting Cleaner System...");

    pinMode(LED_BLUE, OUTPUT);  // Set LED pins as output
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    analogWrite(LED_BLUE, 128);  // Turn on blue LED
    analogWrite(LED_RED, 255);   // Turn off red LED
    analogWrite(LED_GREEN, 0);   // Turn on green LED to indicate system is starting

    cleaner_operator_mode = Cleaner::CleanerOperatorMode::MANUAL;
    delay(1000);  // Wait for a second so the drivers don't kill themselves
    cleaner_system.begin();
    cleaner_system.updatePCF8575();
}

void loop()
{
    cleaner_operator_mode = static_cast<Cleaner::CleanerOperatorMode>(cleaner_system.isAutoMode());
    // cleaner_operator_mode = Cleaner::CleanerOperatorMode::DEBUG;
    switch (cleaner_operator_mode)
    {
        case Cleaner::CleanerOperatorMode::MANUAL:
        {
            runOnSwitch(wasInManualMode, false, [&]{cleaner_system.initializeManualMode();});
            const auto state = cleaner_system.updateDesStateManual();
            cleaner_system.run();
            DO_EVERY(.1, Serial.println(cleaner_system.getEncoder().getRotationUnwrappedInRadians(), 5));
        }
        break;  // case MANUAL

        case Cleaner::CleanerOperatorMode::AUTO:
        {
            runOnSwitch(wasInManualMode, true, [&]{cleaner_system.initializeAutoMode(receiver);});
            cleaner_system.updateModeAuto();  // Update the pcf to get if we need to switch
            receiver.parse();
            switch (receiver.lastReceivedMessageId())
            {
                case SerialReceiverTransmitter::MessageType::COMMAND:
                {
                    SerialReceiverTransmitter::CommandMessage msg = receiver.lastReceivedCommandMessage();
                    cleaner_system.processCommand(msg);
                    cleaner_system.run();
                }
                break;
                case SerialReceiverTransmitter::MessageType::STOP:
                {
                    // If the message is a stop type, this is not the emergency stop
                    SerialReceiverTransmitter::Stop msg =
                        receiver.lastReceivedStopMessage();  // read the message just cause?
                    cleaner_system.stop();
                }
                break;
                default:
                    break;
            }
        }
        break;  // case AUTO
        case Cleaner::CleanerOperatorMode::DEBUG:
        {
            debugLed();
            // runOnSwitch(wasInManualMode, false, [&]{cleaner_system.initializeManualMode();});
            // const auto state = cleaner_system.updateDesStateManual();
            DO_EVERY(1/10.0f, Serial.println(cleaner_system.getEncoder().getRotationUnwrappedInRadians(), 5));
            // cleaner_system.run();
            // ledcWriteNote(JAW_ROTATION_STEP_PIN, NOTE_C, 4);
            // Serial.println(analogRead(CLAMP_POT_PIN));
            // digitalWrite(ROLL_BRAKE_REAL_PIN, HIGH); 
        }
        default:
            break;  // do nothing
    }
}