#pragma once

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
        STOP
    };

    struct gCommand
    {
        bool received = false;
        float y       = 0.0f;  // slide position mm
        float a       = 0.0f;  // jaw rotation rads
        float c       = 0.0f;  // jaw position mm
        float val     = 0.0f;  // value for G4 and other commands
    };

    class CommandMessage
    {
    public:
        gCommand G0;   // G0 is the move command
        gCommand G4;   // G4 is the dwell command
        gCommand G28;  // G28 is the home command
        bool G90;      // G90 is the absolute positioning command
        bool M80;      // M80 is the set max speed command
        bool M17;      // M17 is the set acceleration command

        CommandMessage();
        CommandMessage(gCommand G0, gCommand G4, gCommand G28, bool G90, bool M80, bool M17);
        CommandMessage(char buffer[]);

        bool is_M_command();

    private:
        void ProcessGCommand(char* param, gCommand* command);
    };

    // Stop doesn't have any data associated with it
    class Stop
    {
    public:
        Stop();
        Stop(char buffer[]);
    };

    SerialReceiver();

    void parse();
    CommandMessage lastReceivedCommandMessage() const;
    Stop lastReceivedStopMessage() const;
    MessageType lastReceivedMessageId() const;

private:
    State state_;
    MessageType currMsgId_;
    MessageType lastReceivedMsgId_;
    uint32_t currMsgLen_;
    char currMsgData_[BUFFER_SIZE];  // buffer size defined by constant
    CommandMessage lastReceivedCommandMessage_;
    Stop lastReceivedStopMessage_;
};