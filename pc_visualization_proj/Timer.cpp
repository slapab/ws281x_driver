#include <thread>
#include <chrono>
#include "Timer.h"

Timer::Timer()
{
 ;
}

void Timer::start() {
    ;
}

void Timer::sleep(Timer::ticks_t delay) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
