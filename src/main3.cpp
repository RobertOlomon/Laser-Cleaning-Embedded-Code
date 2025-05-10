// // //
// // //    FILE: PCF8575_interrupt_advanced.ino
// // //  AUTHOR: Rob Tillaart
// // // PURPOSE: test PCF8575 library
// // //     URL: https://github.com/RobTillaart/PCF8575
// // //
// // //  TEST SETUP
// // //   Connect INT pin of the PCF8575 to UNO pin 2
// // //
// // //   (from figure 4 datasheet
// // //   Place a pull up resistor 4K7 between pin and 5V
// // //   Place a capacitor 10-400 pF between pin and GND


// #include "PCF8575.h"
// #include "RotaryEncoder.h"
// #include "BindArg.h"
// // #include "FunctionalInterrupt.h"
// class TestClass {
//   public:
//     TestClass(uint8_t pin) : _pin(pin), count(0) {}
  
//     void begin() {
//       pinMode(_pin, INPUT_PULLUP);
//       // now the ISR service is up—this will succeed
//       attachInterrupt(_pin,
//         bindArgGateThisAllocate(&TestClass::classIsr, this),
//                       FALLING);
//     }
  
//     volatile uint32_t count;
  
//   private:
//     uint8_t _pin;
//     void classIsr() { count++; }
//   };

// PCF8575 PCF(0x20);


// ////////////////////////////////////
// //
// //  INTERRUPT ROUTINE + FLAG
// //
// const int IRQPIN = 6;


// RotaryEncoder encoder(13, 12, PCF);
// TestClass testObject(IRQPIN);

// ////////////////////////////////////
// //
// //  MAIN CODE
// //
// void setup()
// {
//   //  while(!Serial);  //  uncomment when needed
//   Serial.begin(921600);
//   Serial.println(__FILE__);
//   Serial.print("PCF8575_LIB_VERSION:\t");
//   Serial.println(PCF8575_LIB_VERSION);
//   Serial.println();

//   Wire.begin();
//   PCF.begin();

//   testObject.begin();
// }


// void loop()
// {
//   uint32_t now = millis();

//   //  make a local copy of the counter.
//   // noInterrupts();
//   // uint8_t irq_count = testObject.count;
//   // testObject.count = 0;
//   // interrupts();

//   Serial.print("IRQ count: ");
//   Serial.println(testObject.count);
//   delay(100);

//   // if (irq_count > 0)
//   // {
//   //   if (irq_count > 1)
//   //   {
//   //     Serial.print("IRQ missed: ");
//   //     Serial.println(irq_count - 1);  //  as last will be handled
//   //   }
//   //   uint16_t x = PCF.read16();
//   //   encoder.tick();
//   //   // auto value = PCF.read(13);
//   //   // auto value2 = PCF.read(12);
//   //   // Serial.print("READ:\t");
//   //   // Serial.print('\t');
//   //   // Serial.println(x, BIN);
//   //   // Serial.print('\t');
//   //   // Serial.println(encoder.getPosition());
//   //   Serial.println(testObject.count);
//   //   // Serial.print(value);
//   //   // Serial.print('\t');
//   //   // Serial.println(value2);
//   // }

//   //  simulate doing other things here.
//   //  uses to a large delay to miss IRQ's on purpose.
// //   delay(1000);
// }


// //  -- END OF FILE --
