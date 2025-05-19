#define PrintHzRateDebug() do { \
    static unsigned long last_time = micros(); \
    static unsigned int loop_count = 0; \
    \
    loop_count++; \
    if (loop_count >= 1000) { \
        unsigned long current_time = micros(); \
        unsigned long elapsed = current_time - last_time; \
        last_time = current_time; \
        loop_count = 0; \
        \
        if (elapsed > 0) { \
            /* elapsed is microseconds for 1000 loops → Hz = 1000 / (elapsed/1e6) */ \
            float hz = (1000.0f * 1e6f) / (float)elapsed; \
            Serial.println(hz); \
        } \
    } \
} while (0)


#define PRINT_EVERY(seconds, block)                  \
    do {                                             \
        static unsigned long _lastPrintTime = 0;     \
        unsigned long _now = millis();               \
        if (_now - _lastPrintTime >= (seconds * 1000)) { \
            _lastPrintTime = _now;                   \
            block;                                   \
        }                                            \
    } while (0)

inline void runOnSwitch(bool& flag, bool trigger_when, Cleaner& system, void (Cleaner::*func)()) {
    if (flag == trigger_when) {
        (system.*func)();
        flag = !flag;
    }
}