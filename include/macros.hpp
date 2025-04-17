#define PrintHzRateDebug() do { \
    static unsigned long last_time = micros(); \
    unsigned long current_time = micros(); \
    unsigned long elapsed_time = current_time - last_time; \
    last_time = current_time; \
    if (elapsed_time > 0) { \
        Serial.println((float)1e6 / elapsed_time); \
    } else { \
        Serial.println("inf"); \
    } \
} while (0)
