#ifndef DEFINITIONS
#define DEFINITIONS

#ifndef M_PI
#define M_PI   3.14159265358979323846264338327950288
#endif

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef EPSILON
#define EPSILON 0.0001
#endif

#ifdef _WIN32
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(ssize_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}
#endif

#endif // !DEFINITIONS
