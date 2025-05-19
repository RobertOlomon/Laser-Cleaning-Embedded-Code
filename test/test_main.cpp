// #include <cstring>
// #include <iostream>

// #include <unity.h>

// #include "serial_receiver.hpp"

// // #ifdef ARDUINO
// // #include <Arduino.h>
// // #else
// // #endif

// // #ifdef NATIVE
// struct SerialMock
// {
//     template <typename T>
//     void println(const T &value)
//     {
//         std::cerr << value << std::endl;
//     }
//     template <typename T>
//     void print(const T &value)
//     {
//         std::cerr << value;
//     }
// };

// #define Serial SerialMock()
// // #endif

// SerialReceiver::CommandMessage::CommandMessage()
//     : G0(),
//       G4(),
//       G28(),
//       G90(),
//       M80(),
//       M17(),
//       M906()  // Initialize all command messages to default values
// {
// }

// SerialReceiver::CommandMessage::CommandMessage(
//     gCommand G0,
//     gCommand G4,
//     gCommand G28,
//     gCommand G90,
//     mCommand M80,
//     mCommand M17,
//     mCommand M906)  // Define message with parameters
//     : G0(G0),
//       G4(G4),
//       G28(G28),
//       G90(G90),
//       M80(M80),
//       M17(M17),
//       M906(M906)
// {
// }

// SerialReceiver::CommandMessage::CommandMessage(char buffer[])
// {
//     // received string from serial, parse to allowed Gcode and Mcode
//     char POS_STRTOK_F_YOU[strlen(buffer)];
//     std::memcpy(POS_STRTOK_F_YOU, buffer, strlen(buffer));
//     char *token = strtok(POS_STRTOK_F_YOU, " ");

//     switch (token[0])
//     {
//         case 'G':
//         {
//             // Extract the command number (e.g., 0 or 4) by skipping G and seaching for int
//             int gCmd = atoi(token + 1);

//             switch (gCmd)
//             {
//                 // For the Gcode, both G0 and G1 are used in the same way here, so they are
//                 // combined.
//                 case 0:
//                 case 1:
//                     G0.received = true;
//                     ProcessCommand(&buffer[strlen(token) + 1], &G0);
//                     break;
//                 case 4:
//                     G4.received = true;
//                     G4.val      = atof(&buffer[strlen(token) + 1]);
//                     break;
//                 case 28:
//                     G28.received = true;
//                     ProcessCommand(&buffer[strlen(token) + 1], &G28);
//                     break;
//                 case 90:
//                     G90.received = true;
//                     Serial.println("G90 received, I ain't implementing that\n");
//                     break;
//                 default:
//                     Serial.print("Unhandled Gcode type: G");
//                     Serial.print(std::to_string(token[0]).c_str());
//                     Serial.print("\n");
//                     break;
//             }
//         }
//         case 'M':
//         {
//             // For M commands, if they have no additional parameter you can process
//             // directly.
//             int mCmd = atoi(token + 1);
//             switch (mCmd)
//             {
//                 case 80:
//                     M80.received = true;
//                     break;
//                 case 17:
//                     M17.received = true;
//                     break;
//                 case 906:
//                     M906.received = true;
//                     ProcessCommand(&buffer[strlen(token) + 1], &M906);
//                     break;
//                 default:    
//                     Serial.print("Unhandled M-code: M");
//                     Serial.println(mCmd);
//                     break;
//             }
//         }
//     }
// }

// /** Param is the rest of the gCode command in the form of Y10.0 A10.0 C10.0 */
// template <typename commandType>
// void SerialReceiver::CommandMessage::ProcessCommand(char *param, commandType *command)
// {
//     char *token = strtok(param, " ");
//     // Process the G0 command and extract parameters
//     while (token != NULL)
//     {
//         // Check for Y, A, C parameters
//         switch (token[0])
//         {
//             case 'Y':
//                 command->y = atof(token + 1);
//                 break;
//             case 'A':
//                 command->a = atof(token + 1);
//                 break;
//             case 'C':
//                 command->c = atof(token + 1);
//                 break;
//             default:
//                 Serial.print("Unhandled Gcode parameter: ");
//                 Serial.print(std::to_string(token[0]).c_str());
//                 Serial.print("\n");
//                 break;
//         }
//         token = strtok(NULL, " ");
//     }
// }

// SerialReceiver::Stop::Stop() {}

// SerialReceiver::Stop::Stop(char buffer[])
// {
//     // Empty constructor
// }

// // Constructor for SerialReceiver
// SerialReceiver::SerialReceiver()
//     : state_(State::WAITING_FOR_HEADER),
//       currMsgId_(MessageType::NONE),
//       lastReceivedMsgId_(MessageType::NONE),
//       currMsgLen_(0),
//       lastReceivedCommandMessage_(),
//       lastReceivedStopMessage_()
// {
// }

// /**
//  * @brief Parses incoming serial data and processes messages based on their
//  * type.
//  *
//  * This function reads data from the serial buffer and processes it according to
//  * the current state. It handles three states: WAITING_FOR_HEADER,
//  * READING_HEADER, and READING_BODY.
//  *
//  * - WAITING_FOR_HEADER: Waits for the header byte (0xA5) to start reading a new
//  * message.
//  * - READING_HEADER: Reads the message type and length from the header.
//  * - READING_BODY: Reads the message body and processes it based on the message
//  * type.
//  *
//  * The message is expected to be in the following format:
//  * 0xA5 - Header
//  * 1 byte - Message type
//  * 4 bytes - Message length
//  * n bytes - Message data
//  *
//  * It modyfies the lastReceivedCommandMessage_ and lastReceivedStopMessage_
//  * based on the message type. and updates the lastReceivedMsgId_ with the
//  * message type. The function updates the state machine and processes messages
//  * of type COMMAND.
//  */

// // void SerialReceiver::parse()
// // {
// //     switch (state_)
// //     {
// //         case State::WAITING_FOR_HEADER:
// //             if (Serial.available() > 0 && Serial.read() == 0xA5)
// //             {
// //                 state_ = State::READING_HEADER;
// //             }
// //             break;
// //         case State::READING_HEADER:
// //             if (Serial.available() >= HEADER_SIZE)
// //             {
// //                 currMsgId_ = static_cast<MessageType>(
// //                     Serial.read());  // Read the message type and set to current message id
// //                 // Define a union to convert 4 bytes to an int32_t
// //                 union
// //                 {
// //                     uint8_t bytes[4];
// //                     int32_t value;
// //                 } HeaderLength;
// //                 // Read the message length max size of 4 bytes
// //                 for (int i = 0; i < 4; i++)
// //                 {
// //                     HeaderLength.bytes[i] = Serial.read();
// //                 }
// //                 currMsgLen_ = HeaderLength.value;
// //                 state_      = State::READING_BODY;
// //             }
// //             break;
// //         case State::READING_BODY:
// //             if (Serial.available() >= currMsgLen_)
// //             {
// //                 Serial.readBytes(currMsgData_, currMsgLen_);
// //                 switch (currMsgId_)
// //                 {
// //                     case MessageType::COMMAND:
// //                         lastReceivedCommandMessage_ = CommandMessage(currMsgData_);
// //                         break;
// //                     case MessageType::STOP:
// //                         lastReceivedStopMessage_ =
// //                             Stop(currMsgData_);  // Kinda useless but here for completeness
// //                         break;
// //                     case MessageType::NONE:
// //                         break;
// //                 }
// //                 lastReceivedMsgId_ = currMsgId_;
// //                 state_             = State::WAITING_FOR_HEADER;
// //             }
// //             break;
// //     };
// // }

// SerialReceiver::CommandMessage SerialReceiver::lastReceivedCommandMessage() const
// {
//     return lastReceivedCommandMessage_;
// }

// SerialReceiver::Stop SerialReceiver::lastReceivedStopMessage() const
// {
//     return lastReceivedStopMessage_;
// }

// SerialReceiver::MessageType SerialReceiver::lastReceivedMessageId() const
// {
//     return lastReceivedMsgId_;
// }

// void setUp(void)
// {
//    ; // This is run before EACH test
// }

// void tearDown(void)
// {
//     ;// This is run after EACH test
// }

// void test_G0_received()
// {
//     char command[] = "G0 Y10.0";
//     SerialReceiver::CommandMessage msg(command);
//     TEST_ASSERT_TRUE(msg.G0.received);
// }

// void test_G0_value()
// {
//     char command[] = "G0 Y10.0";
//     SerialReceiver::CommandMessage msg(command);
//     TEST_ASSERT_EQUAL_FLOAT(10.0f, msg.G0.y);
// }

// void test2_G0_value()
// {
//     char command[] = "G0 Y40.0 A20.0 C30.0";
//     SerialReceiver::CommandMessage msg(command);
//     TEST_ASSERT_EQUAL_FLOAT(40.0f, msg.G0.y);
//     TEST_ASSERT_EQUAL_FLOAT(20.0f, msg.G0.a);
//     TEST_ASSERT_EQUAL_FLOAT(30.0f, msg.G0.c);
// }

// void test_G1_received()
// {
//     char command[] = "G0 Y10.0";
//     SerialReceiver::CommandMessage msg(command);
//     TEST_ASSERT_TRUE(msg.G0.received);
//     char command2[] = "G1 Y10.0";
//     SerialReceiver::CommandMessage msg2(command2);
//     TEST_ASSERT_TRUE(msg2.G0.received);
// }
// #ifdef ARDUINO
// void loop() {}
// void setup()
// #else
// int main(int argc, char **argv)
// #endif
// {
//     UNITY_BEGIN();
//     RUN_TEST(test_G0_received);
//     RUN_TEST(test_G0_value);
//     RUN_TEST(test2_G0_value);
//     RUN_TEST(test_G1_received);

//     UNITY_END();
// }