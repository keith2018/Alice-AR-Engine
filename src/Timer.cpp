/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "Timer.h"

namespace Alice {
    void Timer::start() {
        tm.start();
    }

    void Timer::stop() {
        tm.stop();
    }

    double Timer::elapseSeconds() const {
        return tm.getTimeSec();
    }

    double Timer::elapseMillis() const {
        return tm.getTimeMilli();
    }
}