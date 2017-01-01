#ifndef TIMER_H
#define TIMER_H
#include <cstdint>

class Timer
{
public:
    using ticks_t = uint32_t;
    Timer();

    void sleep(ticks_t delay);
    void start();
};

#endif // TIMER_H
