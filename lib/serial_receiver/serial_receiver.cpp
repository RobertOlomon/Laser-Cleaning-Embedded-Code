#include "serial_receiver.hpp"
#include <Arduino.h>
#include <cstring>

SerialReceiver::CommandMessage::CommandMessage() // Define default message
    : G0(G0), G1(G1), G4(G4), G90(false), M80(false), M17(false) {}

SerialReceiver::CommandMessage::CommandMessage(Gcommand G0, Gcommand G1,
                                               Gcommand G4, bool G90, bool M80,
                                               bool M17)
    : G0(G0), G1(G1), G4(G4), G90(G90), M80(M80), M17(M17) {}

SerialReceiver::CommandMessage::CommandMessage(char buffer[]) {
  // recieved string from serial, parse to allowed Gcode and Mcode
  char *token = strtok(buffer, " ");
  while (token != NULL) {
    if (token[0] == 'G') {
      // Extract the command number (e.g., 0 or 4)
      int gCmd = token[1] - '0';
      // Get the parameter (assumes parameter follows the command)
      token = strtok(token + 3, " ");
      if (token == NULL) {
        Serial.print("Expected parameter after G" + static_cast<String>(gCmd) +
                     "\n");
        break;
      }

      float param = atof(token);
      switch (gCmd) {
      case 0:
        G0.received = true;
        G0.value = param;
        break;
      case 4:
        G4.received = true;
        G4.value = param;
        break;
      default:
        Serial.print("Unhandled G-code: G" + static_cast<String>(gCmd) + "\n");
        break;
      }
    } else if (token[0] == 'M') {
      // For M commands, if they have no additional parameter you can process
      // directly.
      int mCmd = atoi(token + 1);
      switch (mCmd) {
      case 80:
        M80 = true;
        break;
      case 17:
        M17 = true;
        break;
      default:
        Serial.print("Unhandled M-code: M" + static_cast<String>(mCmd) + "\n");
        break;
      }
    }
    // Advance to the next token
    token = strtok(NULL, " ");
  }
}

SerialReceiver::FreeMessage::FreeMessage() {}

SerialReceiver::FreeMessage::FreeMessage(char buffer[]) {
  // Empty constructor
}

// Constructor for SerialReceiver
SerialReceiver::SerialReceiver()
    : state_(State::WAITING_FOR_HEADER), currMsgId_(MessageType::NONE),
      lastReceivedMsgId_(MessageType::NONE), currMsgLen_(0),
      lastReceivedCommandMessage_(), lastReceivedFreeMessage_() {}

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
 * It modyfies the lastReceivedCommandMessage_ and lastReceivedFreeMessage_
 * based on the message type. and updates the lastReceivedMsgId_ with the
 * message type. The function updates the state machine and processes messages
 * of type COMMAND.
 */

void SerialReceiver::parse() {
  switch (state_) {
  case State::WAITING_FOR_HEADER:
    if (Serial.available() > 0 && Serial.read() == 0xA5) {
      state_ = State::READING_HEADER;
    }
    break;
  case State::READING_HEADER:
    if (Serial.available() >= HEADER_SIZE) {
      currMsgId_ = static_cast<MessageType>(
          Serial.read()); // Read the message type and set to current message id
      // Define a union to convert 4 bytes to an int32_t
      union {
        uint8_t bytes[4];
        int32_t value;
      } HeaderLength;
      // Read the message length max size of 4 bytes
      for (int i = 0; i < 4; i++) {
        HeaderLength.bytes[i] = Serial.read();
      }
      currMsgLen_ = HeaderLength.value;
      state_ = State::READING_BODY;
    }
    break;
  case State::READING_BODY:
    if (Serial.available() >= currMsgLen_) {
      Serial.readBytes(currMsgData_, currMsgLen_);
      switch (currMsgId_) {
      case MessageType::COMMAND:
        lastReceivedCommandMessage_ = CommandMessage(currMsgData_);
        break;
      case MessageType::FREE:
        lastReceivedFreeMessage_ = FreeMessage(
            currMsgData_); // Kinda useless but here for completeness
        break;
      case MessageType::NONE:
        break;
      }
      lastReceivedMsgId_ = currMsgId_;
      state_ = State::WAITING_FOR_HEADER;
    }
    break;
  };
}

bool SerialReceiver::CommandMessage::is_M_command() { return M80 || M17; }

SerialReceiver::CommandMessage
SerialReceiver::lastReceivedCommandMessage() const {
  return lastReceivedCommandMessage_;
}

SerialReceiver::FreeMessage SerialReceiver::lastReceivedFreeMessage() const {
  return lastReceivedFreeMessage_;
}

SerialReceiver::MessageType SerialReceiver::lastReceivedMessageId() const {
  return lastReceivedMsgId_;
}