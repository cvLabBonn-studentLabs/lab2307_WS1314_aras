/*
 * timer.h
 * For banchmarking purposes only
 *
 *  Created on: Mar 25, 2014
 *      Author: neek
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <cstdlib>
#include <sys/time.h>

namespace timer {

class Timer
{
    timeval timer[2];

  public:
    timeval start();
    timeval stop();
    int duration() const;
};

}

#endif /* TIMER_H_ */
