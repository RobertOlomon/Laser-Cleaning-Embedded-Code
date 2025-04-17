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

// SerialReceiver::CommandMessage::CommandMessage()  // Define default message
//     : G0(G0),
//       G4(G4),
//       G90(false),
//       M80(false),
//       M17(false)
// {
// }

// SerialReceiver::CommandMessage::CommandMessage(
//     gCommand G0,
//     gCommand G4,
//     bool G90,
//     bool M80,
//     bool M17)
//     : G0(G0),
//       G4(G4),
//       G90(G90),
//       M80(M80),
//       M17(M17)
// {
// }

// SerialReceiver::CommandMessage::CommandMessage(char buffer[])
// {
//     // received string from serial, parse to allowed Gcode and Mcode
//     char POS_STRTOK_FUCK_YOU[strlen(buffer)];
//     std::memcpy(POS_STRTOK_FUCK_YOU, buffer, strlen(buffer));
//     char *token = strtok(POS_STRTOK_FUCK_YOU, " ");

//     if (token[0] == 'G')
//     {
//         // Extract the command number (e.g., 0 or 4)
//         int gCmd = atoi(token + 1);

//         switch (gCmd)
//         {
//             // For the Gcode, both G0 and G1 are used in the same way here, so they are
//             // combined.
//             case 0:
//             case 1:
//                 G0.received = true;
//                 ProcessG0Command(&buffer[strlen(token) + 1], &G0);
//                 break;
//             case 4:
//                 G4.received = true;
//                 G4.val      = atof(&buffer[strlen(token) + 1]);
//                 break;
//             default:
//                 Serial.print("Unhandled Gcode type: G");
//                 Serial.print(std::to_string(token[0]).c_str());
//                 Serial.print("\n");
//                 break;
//         }
//     }
//     else if (token[0] == 'M')
//     {
//         // For M commands, if they have no additional parameter you can process
//         // directly.
//         int mCmd = atoi(token + 1);
//         switch (mCmd)
//         {
//             case 80:
//                 M80 = true;
//                 break;
//             case 17:
//                 M17 = true;
//                 break;
//             default:
//                 Serial.print("Unhandled M-code: M");
//                 Serial.println(mCmd);
//                 break;
//         }
//     }
// }

// /** Param is the rest of the gCode command in the form of Y10.0 A10.0 C10.0 */

// void SerialReceiver::CommandMessage::ProcessGCommand(char *param, gCommand *command)
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

// void setUp(void)
// {
//     // This is run before EACH test
// }

// void tearDown(void)
// {
//     // This is run after EACH test
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