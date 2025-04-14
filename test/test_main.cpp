#include <cstring>
#include <iostream>

#include <unity.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#endif

class SerialReceiver
{
public:
    static constexpr int HEADER_SIZE = 5;
    static constexpr int BUFFER_SIZE = 256;

    enum State
    {
        WAITING_FOR_HEADER = 0,
        READING_HEADER,
        READING_BODY
    };

    enum MessageType
    {
        NONE = 0,
        COMMAND,
        FREE
    };

    struct Gcommand
    {
        bool received = false;
        float value   = 0.0f;
    };

    class CommandMessage
    {
    public:
        Gcommand G0;  // G0 is the move command
        Gcommand G1;  // G1 is the move command
        Gcommand G4;  // G4 is the dwell command
        bool G90;     // G90 is the absolute positioning command
        bool M80;     // M80 is the set max speed command
        bool M17;     // M17 is the set acceleration command

        CommandMessage();
        CommandMessage(Gcommand G0, Gcommand G1, Gcommand G4, bool G90, bool M80, bool M17);
        CommandMessage(char buffer[]);

        bool is_M_command();
    };

    // FreeMessage doesn't have any data associated with it
    class FreeMessage
    {
    public:
        FreeMessage();
        FreeMessage(char buffer[]);
    };

    SerialReceiver();

    void parse();
    CommandMessage lastReceivedCommandMessage() const;
    FreeMessage lastReceivedFreeMessage() const;
    MessageType lastReceivedMessageId() const;

private:
    State state_;
    MessageType currMsgId_;
    MessageType lastReceivedMsgId_;
    uint32_t currMsgLen_;
    char currMsgData_[BUFFER_SIZE];  // buffer size defined by constant
    CommandMessage lastReceivedCommandMessage_;
    FreeMessage lastReceivedFreeMessage_;
};

SerialReceiver::CommandMessage::CommandMessage(char buffer[])
{
    // recieved string from serial, parse to allowed Gcode and Mcode
    char* token = strtok(buffer, " ");
    while (token != NULL)
    {
        if (token[0] == 'G')
        {
            // Extract the command number (e.g., 0 or 4)
            int gCmd = token[1] - '0';
            std::cerr << "G command: " << gCmd << std::endl;
            // Get the parameter (assumes parameter follows the command)
            token = strtok(token + 3, " ");
            std::cerr << "Token: " << token << std::endl;
            if (token == NULL)
            {
                // Serial.print("Expected parameter after G" + static_cast<String>(gCmd) + "\n");
                break;
            }

            switch (gCmd)
            {
                case 0:
                    switch (token[0])
                    {
                        case 'X':
                            G0.received = true;
                            std::cerr << "Token: " << token + 1 << std::endl;
                            G0.value = atof(token + 1);
                            break;
                        case 'Y':
                            G1.received = true;
                            G1.value    = atof(token + 1);
                            break;
                        default:
                            // Serial.print("Unhandled G-code: G" + static_cast<String>(gCmd) +
                            // "\n");
                            break;
                    }
                    break;
                case 1:
                    std::cerr << "G1 command hit token: " << token[0] << std::endl;

                    switch (token[0])
                    {
                        case 'X':
                            G1.received = true;
                            G1.value    = atof(token + 1);
                            break;
                        case 'Y':
                            G1.received = true;
                            G1.value    = atof(token + 1);
                            break;
                        default:
                            // Serial.print("Unhandled G-code: G" + static_cast<String>(gCmd) +
                            // "\n");
                            break;
                    }
                    break;
                case 4:
                    G4.received = true;
                    G4.value    = atof(token + 1);
                    break;
                default:
                    // Serial.print("Unhandled G-code: G" + static_cast<String>(gCmd) + "\n");
                    break;
            }
        }
        else if (token[0] == 'M')
        {
            // For M commands, if they have no additional parameter you can process
            // directly.
            int mCmd = atoi(token + 1);
            switch (mCmd)
            {
                case 80:
                    M80 = true;
                    break;
                case 17:
                    M17 = true;
                    break;
                default:
                    // Serial.print("Unhandled M-code: M" + static_cast<String>(mCmd) + "\n");
                    break;
            }
        }
        // Advance to the next token
        token = strtok(NULL, " ");
    }
}

void setUp(void)
{
    // This is run before EACH test
}

void tearDown(void)
{
    // This is run after EACH test
}

void test_G0_received()
{
    char command[] = "G0 X10.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G0.received);
}

void test_G0_value()
{
    char command[] = "G0 X10.0";
    SerialReceiver::CommandMessage msg(const_cast<char*>(command));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, msg.G0.value);
}

void test_G1_received()
{
    char command[] = "G0 X10.0";
    SerialReceiver::CommandMessage msg(const_cast<char*>(command));
    TEST_ASSERT_FALSE(msg.G1.received);
    char command2[] = "G1 X10.0";
    SerialReceiver::CommandMessage msg2(command2);
    TEST_ASSERT_TRUE(msg2.G1.received);
}
#ifdef ARDUINO
void loop() {}
void setup()
#else
int main(int argc, char** argv)
#endif
{
    UNITY_BEGIN();
    RUN_TEST(test_G0_received);
    RUN_TEST(test_G0_value);
    RUN_TEST(test_G1_received);

    UNITY_END();
}