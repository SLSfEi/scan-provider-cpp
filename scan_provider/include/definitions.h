#ifndef DEFINITIONS
#define DEFINITIONS

#define SERVER_ADDR "127.0.0.1"
#define SERVER_PORT 15245

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#define SERIAL_PORT "COM5"
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#define SERIAL_PORT "/dev/ttyUSB0"
static inline void delay(sl_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}
#endif

#endif // !DEFINITIONS