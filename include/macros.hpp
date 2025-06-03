#define PrintHzRateDebug()                                                                 \
    do                                                                                     \
    {                                                                                      \
        static unsigned long last_time = micros();                                         \
        static unsigned int loop_count = 0;                                                \
                                                                                           \
        loop_count++;                                                                      \
        if (loop_count >= 1000)                                                            \
        {                                                                                  \
            unsigned long current_time = micros();                                         \
            unsigned long elapsed      = current_time - last_time;                         \
            last_time                  = current_time;                                     \
            loop_count                 = 0;                                                \
                                                                                           \
            if (elapsed > 0)                                                               \
            {                                                                              \
                /* elapsed is microseconds for 1000 loops → Hz = 1000 / (elapsed/1e6) */ \
                float hz = (1000.0f * 1e6f) / (float)elapsed;                              \
                Serial.println(hz);                                                        \
            }                                                                              \
        }                                                                                  \
    } while (0)

#define DO_EVERY(seconds, block)                      \
    do                                                \
    {                                                 \
        static unsigned long _lastRunTime = 0;        \
        unsigned long _now                = millis(); \
        if (_now - _lastRunTime >= (seconds * 1e3))   \
        {                                             \
            _lastRunTime = _now;                      \
            block;                                    \
        }                                             \
    } while (0)

inline void runOnSwitch(bool& flag, bool trigger_when, Cleaner& system, void (Cleaner::*func)())
{
    if (flag == trigger_when)
    {
        (system.*func)();
        flag = !flag;
    }
}

#define DO_ONCE_AFTER(seconds, block)                    \
    do                                                   \
    {                                                    \
        static unsigned long _startTime = 0;             \
        static bool _done               = false;         \
        if (!_done)                                      \
        {                                                \
            if (_startTime == 0)                         \
            {                                            \
                _startTime = millis();                   \
            }                                            \
            if (millis() - _startTime >= (seconds)*1000) \
            {                                            \
                _done = true;                            \
                block;                                   \
            }                                            \
        }                                                \
    } while (0)

template <typename T>
inline T limit_val(const T& val, const T& min_val, const T& max_val)
{
    if (val < min_val)
    {
        return min_val;
    }
    if (val > max_val)
    {
        return max_val;
    }
    return val;
}