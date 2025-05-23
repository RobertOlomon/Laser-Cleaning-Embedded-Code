#include <cstring>
#include <iostream>

#include <unity.h>

#include "serial_receiver.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#else
struct SerialMock
{
    template <typename T>
    void println(const T &value)
    {
        std::cerr << value << std::endl;
    }
    template <typename T>
    void print(const T &value)
    {
        std::cerr << value;
    }
    int read() { return 1; }
    int available() { return 1; }
    int readBytes(char *bonk, int boink) { return 1; }
};

#define Serial SerialMock()

#endif

// clang-format off

#include "serial_receiver.cpp"

// clang-format on

void setUp(void)
{
    ;  // This is run before EACH test
}

void tearDown(void)
{
    ;  // This is run after EACH test
}

void test_G0_received()
{
    char command[] = "G0 Y10.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G0.received);
}

void test_G0_value()
{
    char command[] = "G0 Y10.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, msg.G0.y);
}

void test2_G0_value()
{
    char command[] = "G0 Y40.0 A20.0 C30.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, msg.G0.y);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, msg.G0.a);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, msg.G0.c);
}

void test_G1_received()
{
    char command[] = "G0 Y10.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G0.received);
    char command2[] = "G1 Y10.0";
    SerialReceiver::CommandMessage msg2(command2);
    TEST_ASSERT_TRUE(msg2.G0.received);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, msg2.G0.y);
}

void test_G4_Received()
{
    char command[] = "G4 S100.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G4.received);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, msg.G4.val);
}

void test_G28_Received()
{
    char command[] = "G28";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G28.received);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, msg.G28.c);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, msg.G28.y);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, msg.G28.a);
}

void test_G28_Partial() 
{
    char command[] = "G28 C";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.G28.received);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, msg.G28.c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, msg.G28.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, msg.G28.a);
}

void test_M80_Command()
{
    char command[] = "M80 Y40.0 A20.0 C30.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.M80.received);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, msg.M80.y);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, msg.M80.a);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, msg.M80.c);
}

void test_M17_Command()
{
    char command[] = "M17 Y40.0 A20.0 C30.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.M17.received);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, msg.M17.y);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, msg.M17.a);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, msg.M17.c);
}

void test_M906_Command()
{
    char command[] = "M906 Y40.0 A20.0 C30.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_TRUE(msg.M906.received);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, msg.M906.y);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, msg.M906.a);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, msg.M906.c);
}

void test_M906_Command_Trigger_When_Correct()
{
    char command[] = "M80 Y40.0 A20.0 C30.0";
    SerialReceiver::CommandMessage msg(command);
    TEST_ASSERT_FALSE(msg.M906.received);
}

#ifdef ARDUINO
void loop() {}
void setup()
#else
int main(int argc, char **argv)
#endif
{
    UNITY_BEGIN();

    RUN_TEST(test_G0_received);
    RUN_TEST(test_G0_value);
    RUN_TEST(test2_G0_value);
    RUN_TEST(test_G1_received);
    RUN_TEST(test_G4_Received);
    RUN_TEST(test_M80_Command);
    RUN_TEST(test_M17_Command);
    RUN_TEST(test_M906_Command);
    RUN_TEST(test_M906_Command_Trigger_When_Correct);
    RUN_TEST(test_G28_Received);
    RUN_TEST(test_G28_Partial);

    UNITY_END();
}