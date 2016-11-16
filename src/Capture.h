/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_CAPTURE_H
#define ALICE_CAPTURE_H


#include <opencv2/opencv.hpp>

namespace Alice {
    class Capture {
    public:
        Capture() : cap(NULL) {
        }

        ~Capture() {
            close();
        }

        bool open(int w, int h, int idx = 0);

        void close();

        void nextFrame(cv::Mat &frame);

    private:
        cv::VideoCapture *cap;
    };
}


#endif //ALICE_CAPTURE_H
