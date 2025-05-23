
#include "serial_receiver.hpp"

#include <cstring>

#ifdef ARDUINO
#include <Arduino.h>
#else
#endif

SerialReceiver::CommandMessage::CommandMessage()
    : G0(),
      G4(),
      G28(),
      G90(),
      M80(),
      M17(),
      M906()  // Initialize all command messages to default values
{
}

SerialReceiver::CommandMessage::CommandMessage(
    gCommand G0,
    gCommand G4,
    gCommand G28,
    gCommand G90,
    mCommand M80,
    mCommand M17,
    mCommand M906)  // Define message with parameters
    : G0(G0),
      G4(G4),
      G28(G28),
      G90(G90),
      M80(M80),
      M17(M17),
      M906(M906)
{
}

/**
 * @brief Constructs a CommandMessage object by parsing a buffer containing G-code or M-code commands.
 *
 * This constructor takes a character buffer as input, which represents a G-code or M-code command string.
 * It parses the buffer to identify the command type (G or M) and its associated parameters. The parsed
 * data is stored in the respective command fields (e.g., G0, G4, M80, etc.) of the CommandMessage object.
 *
 * The parsing logic handles:
 * - G-code commands (e.g., G0, G4, G28, G90) and their parameters (e.g., Y, A, C).
 * - M-code commands (e.g., M80, M17, M906) and their parameters.
 *
 * @param buffer A null-terminated character array containing the G-code or M-code command string.
 *
 * @note The constructor uses `strtok` to tokenize the input buffer and processes each token to extract
 *       the command type and parameters. Unhandled commands or parameters are logged using `Serial.print`.
 */
SerialReceiver::CommandMessage::CommandMessage(char buffer[])
{
    // received string from serial, parse to allowed Gcode and Mcode
    char POS_STRTOK_F_YOU[strlen(buffer)+1];
    std::memcpy(POS_STRTOK_F_YOU, buffer, strlen(buffer)+1);
    char *token = strtok(POS_STRTOK_F_YOU, " ");

    switch (token[0])
    {
        case 'G':
        {
            // Extract the command number (e.g., 0 or 4) by skipping G and seaching for int
            int gCmd = atoi(token + 1);

            switch (gCmd)
            {
                // For the Gcode, both G0 and G1 are used in the same way here, so they are
                // combined.
                case 0:
                case 1:
                    G0.received = true;
                    ProcessCommand(&buffer[strlen(token) + 1], &G0);
                    break;
                case 4:
                    G4.received = true;
                    G4.val      = atof(&buffer[strlen(token) + 2]); // + 2 to skip the letter
                    break;
                case 28:
                    G28.received = true;
                    ProcessHomeCommand(&buffer[strlen(token) + 1], &G28);
                    break;
                case 90:
                    G90.received = true;
                    Serial.println("G90 received, I ain't implementing that\n");
                    break;
                default:
                    Serial.print("Unhandled Gcode type: G");
                    Serial.print(std::to_string(token[0]).c_str());
                    Serial.print("\n");
                    break;
            }
        }
        break;
        case 'M':
        {
            // For M commands, if they have no additional parameter you can process
            // directly
            int mCmd = atoi(token + 1);
            switch (mCmd)
            {
                case 80:
                    M80.received = true;
                    ProcessCommand(&buffer[strlen(token) + 1], &M80);
                    break;
                case 17:
                    M17.received = true;
                    ProcessCommand(&buffer[strlen(token) + 1], &M17);
                    break;
                case 906:
                    M906.received = true;
                    ProcessCommand(&buffer[strlen(token) + 1], &M906);
                    break;
                default:
                    Serial.print("Unhandled M-code: M");
                    Serial.println(mCmd);
                    break;
            }
        }
        break;
    }
}

/** Param is the rest of the gCode command in the form of Y10.0 A10.0 C10.0 */
template <typename commandType>
void SerialReceiver::CommandMessage::ProcessCommand(char *param, commandType *command)
{
    char *token = strtok(param, " ");
    // Process the command and extract parameters
    while (token != NULL)
    {
        // Check for Y, A, C parameters
        switch (token[0])
        {
            case 'Y':
                command->y = atof(token + 1);
                break;
            case 'A':
                command->a = atof(token + 1);
                break;
            case 'C':
                command->c = atof(token + 1);
                break;
            default:
                Serial.print("Unhandled Gcode parameter: ");
                Serial.print(std::to_string(token[0]).c_str());
                Serial.print("\n");
                break;
        }
        token = strtok(NULL, " ");
    }
}

// Special code for the home command because of it's strangeness, it's a hack but I don't wanna refactor stuff
void SerialReceiver::CommandMessage::ProcessHomeCommand(char *param, gCommand *command)
{
    char *token = strtok(param, " ");
    // Process the command and extract parameters
    while (token != NULL)
    {
        // Check for Y, A, C parameters
        switch (token[0])
        {
            case 'Y':
                command->y = 1.0f;
                break;
            case 'A':
                command->a = 1.0f;
                break;
            case 'C':
                command->c = 1.0f;
                break;
            default:
                // Should be when no axis are specified, means home all
                command->y = 1.0f;
                command->a = 1.0f;
                command->c = 1.0f;
                break;
        }
        token = strtok(NULL, " ");
    }
}

SerialReceiver::Stop::Stop() {}

SerialReceiver::Stop::Stop(char buffer[])
{
    // Empty constructor
}

// Constructor for SerialReceiver
SerialReceiver::SerialReceiver()
    : state_(State::WAITING_FOR_HEADER),
      currMsgId_(MessageType::NONE),
      lastReceivedMsgId_(MessageType::NONE),
      currMsgLen_(0),
      lastReceivedCommandMessage_(),
      lastReceivedStopMessage_()
{
}

/**
 * @brief Parses incoming serial data and processes messages based on their
 * type.
 *
 * This function reads data from the serial buffer and processes it according to
 * the current state. It handles three states: WAITING_FOR_HEADER,
 * READING_HEADER, and READING_BODY.
 *
 * - WAITING_FOR_HEADER: Waits for the header byte (0xA5) to start reading a new
 * message.
 * - READING_HEADER: Reads the message type and length from the header.
 * - READING_BODY: Reads the message body and processes it based on the message
 * type.
 *
 * The message is expected to be in the following format:
 * 0xA5 - Header
 * 1 byte - Message type
 * 4 bytes - Message length
 * n bytes - Message data
 *
 * It modifies the lastReceivedCommandMessage_ and lastReceivedStopMessage_
 * based on the message type. and updates the lastReceivedMsgId_ with the
 * message type. The function updates the state machine and processes messages
 * of type COMMAND.
 */

void SerialReceiver::parse()
{
    switch (state_)
    {
        case State::WAITING_FOR_HEADER:
            if (Serial.available() > 0 && Serial.read() == 0xA5)
            {
                state_ = State::READING_HEADER;
            }
            break;
        case State::READING_HEADER:
            if (Serial.available() >= HEADER_SIZE)
            {
                currMsgId_ = static_cast<MessageType>(
                    Serial.read());  // Read the message type and set to current message id
                // Define a union to convert 4 bytes to an int32_t
                union
                {
                    uint8_t bytes[4];
                    int32_t value;
                } HeaderLength;
                // Read the message length max size of 4 bytes
                for (int i = 0; i < 4; i++)
                {
                    HeaderLength.bytes[i] = Serial.read();
                }
                currMsgLen_ = HeaderLength.value;
                state_      = State::READING_BODY;
            }
            break;
        case State::READING_BODY:
            if (Serial.available() >= currMsgLen_)
            {
                Serial.readBytes(currMsgData_, currMsgLen_);
                switch (currMsgId_)
                {
                    case MessageType::COMMAND:
                        lastReceivedCommandMessage_ = CommandMessage(currMsgData_);
                        break;
                    case MessageType::STOP:
                        lastReceivedStopMessage_ =
                            Stop(currMsgData_);  // Kinda useless but here for completeness
                        break;
                    case MessageType::NONE:
                        break;
                }
                lastReceivedMsgId_ = currMsgId_;
                state_             = State::WAITING_FOR_HEADER;
            }
            break;
    };
}

SerialReceiver::CommandMessage SerialReceiver::lastReceivedCommandMessage() const
{
    return lastReceivedCommandMessage_;
}

SerialReceiver::Stop SerialReceiver::lastReceivedStopMessage() const
{
    return lastReceivedStopMessage_;
}

SerialReceiver::MessageType SerialReceiver::lastReceivedMessageId() const
{
    return lastReceivedMsgId_;
}