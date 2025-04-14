#pragma once

#include <Arduino.h>
#include <cstdint>

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

  struct Gcommand {
    bool received = false;
    float value = 0.0f;
  };

  class CommandMessage
  {
  public:
      Gcommand G0;  // G0 is the move command
      Gcommand G1;  // G1 is the move command
      Gcommand G4;  // G4 is the dwell command
      bool G90;  // G90 is the absolute positioning command
      bool M80;  // M80 is the set max speed command
      bool M17;  // M17 is the set acceleration command

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
    char currMsgData_[BUFFER_SIZE]; // buffer size defined by constant
    CommandMessage lastReceivedCommandMessage_;
    FreeMessage lastReceivedFreeMessage_;
    
};