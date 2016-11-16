/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_TIMER_H
#define ALICE_TIMER_H


#include <opencv2/opencv.hpp>
#include <string>

namespace Alice {
    class Timer {
    public:
        Timer() {}

        ~Timer() {}

        void start();

        void stop();

        double elapseSeconds() const;

        double elapseMillis() const;

    private:
        cv::TickMeter tm;
    };


    class ScopedTimer {
    public:

        ScopedTimer(const char *str)
                : mStr(str) {
            mTimer.start();
        }

        ~ScopedTimer() {
            mTimer.stop();
            printf("%s: %f ms\n", mStr.c_str(), mTimer.elapseMillis());
        }

        operator bool() {
            return true;
        }

    private:
        Timer mTimer;
        std::string mStr;
    };
}

#ifndef NDEBUG
#   define TIMED(X) if(Alice::ScopedTimer _ScopedTimer = X)
#else
#   define TIMED(X)
#endif


#endif //ALICE_TIMER_H
