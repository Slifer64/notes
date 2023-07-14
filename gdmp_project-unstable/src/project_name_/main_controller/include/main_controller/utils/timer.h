#ifndef $_PROJECT_384$_TIMER_H
#define $_PROJECT_384$_TIMER_H

#include <chrono>

class Timer
{
public:
    Timer()
    {
      clock_gettime(CLOCK_REALTIME, &beg_);
    }

    long int elapsedNanoSec()
    {
        clock_gettime(CLOCK_REALTIME, &end_);
        return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
    }

    double elapsedMicroSec()
    {
        return elapsedNanoSec()/1000.0;
    }

    double elapsedMilliSec()
    {
        return elapsedNanoSec()/1000000.0;
    }

    double elapsedSec()
    {
        return elapsedNanoSec()/1000000000.0;
    }

    void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};


#endif // $_PROJECT_384$_TIMER_H
