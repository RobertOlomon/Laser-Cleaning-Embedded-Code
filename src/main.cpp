#include <Arduino.h>

#include "cleaner_system.hpp"
#include "macros.hpp"
#include "serial_receiver.hpp"
#include "stepper_motor.hpp"

constexpr int BAUDERATE = 921600;

SerialReceiver receiver;

Cleaner cleaner_system;

static bool wasInManualMode = false;
static bool wasInDebugMode = false;

void ESTOP_ISR()
{
    // cleaner_system.shutdown();
    analogWrite(LED_RED, 0);      // Turn on red LED to indicate emergency stop
    analogWrite(LED_GREEN, 255);  // Turn off green LED
    analogWrite(LED_BLUE, 255);
}

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

    pinMode(LED_BLUE, OUTPUT);  // Set LED pins as output
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    analogWrite(LED_BLUE, 128);  // Turn on blue LED
    analogWrite(LED_RED, 255);   // Turn off red LED
    analogWrite(LED_GREEN, 0);   // Turn on green LED to indicate system is starting
}

void loop()
{
    Cleaner::CleanerOperatorMode cleaner_operator_mode =
        static_cast<Cleaner::CleanerOperatorMode>(!cleaner_system.getIOExpander().readButton(MODE_PIN));  // get system mode

    cleaner_operator_mode = Cleaner::CleanerOperatorMode::DEBUG;  // for testing purposes

    switch (cleaner_operator_mode)
    {
        case Cleaner::CleanerOperatorMode::MANUAL:
        {
            runOnSwitch(wasInManualMode, false, cleaner_system, &Cleaner::initializeManualMode);
            cleaner_system.updateDesStateManual();
            cleaner_system.run();
        }
        break; // case MANUAL

        case Cleaner::CleanerOperatorMode::AUTO:
        {
            runOnSwitch(wasInManualMode, true, cleaner_system, &Cleaner::initializeAutoMode);
            // will either update the message or skip if no message is available
            receiver.parse();

            switch (receiver.lastReceivedMessageId())
            {
                case SerialReceiver::MessageType::COMMAND:
                {
                    SerialReceiver::CommandMessage msg = receiver.lastReceivedCommandMessage();
                    cleaner_system.processCommand(msg);
                    cleaner_system.run();
                }
                break;
                case SerialReceiver::MessageType::STOP:
                {
                    // If the message is a stop type, this is not the emergency stop
                    SerialReceiver::Stop msg =
                        receiver.lastReceivedStopMessage();  // read the message just cause?
                    cleaner_system.stop();
                }
                break;

                default:
                    break;
            }
        }
        break; // case AUTO
        case(Cleaner::CleanerOperatorMode::DEBUG):
        {
            runOnSwitch(wasInDebugMode, false, cleaner_system, &Cleaner::initializeManualMode);
            auto State = cleaner_system.updateDesStateManual();
            // cleaner_system.run();
            // uint16_t debug = cleaner_system.getIOExpander().read(ENCODER_JAW_POSITION_BUTTON_PIN);
            // uint16_t debug = cleaner_system.getIOExpander().read16();
            // uint16_t debug2 = cleaner_system.getIOExpander().isConnected();
            // Serial.println(debug2);
            // uint16_t debug = cleaner_system.getIOExpander().read16();
            // Serial.println(debug,BIN);
            PRINT_EVERY(.2,State.print());
            // cleaner_system.getIOExpander().write(1,LOW);
            // cleaner_system.getIOExpander().write(1,255);

            // Serial.print("Mode");
            // Serial.println(cleaner_system.getIOExpander().read(MODE_PIN));
            // Serial.print("Brake");
            // Serial.println(cleaner_system.getIOExpander().read(ROLLER_BRAKE_PIN));

        }
        break; // case DEBUG

        default:
            break;  // do nothing
    }
}