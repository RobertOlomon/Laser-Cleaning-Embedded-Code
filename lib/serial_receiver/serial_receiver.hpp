#ifndef SERIAL_RECEIVER_H
#define SERIAL_RECEIVER_H

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

  class CommandMessage
  {
  public:
      float G0;
      float G1;
      float G4;
      bool G90;
      bool M80;
      bool M17;

      CommandMessage();
      CommandMessage(float G0, float G1, float G4, bool G90, bool M80, bool M17);
      CommandMessage(char buffer[]);
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
  CommandMessage lastReceivedGeneralMessage() const;
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

#endif // SERIAL_RECEIVER_H