/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "Capture.h"

#define VIDEO_CAP_FPS 30

namespace Alice {

    bool Capture::open(int w, int h, int idx) {
        cap = new cv::VideoCapture(idx);

        cap->set(CV_CAP_PROP_FPS, VIDEO_CAP_FPS);
        cap->set(CV_CAP_PROP_FRAME_WIDTH, w);
        cap->set(CV_CAP_PROP_FRAME_HEIGHT, h);

        return cap->isOpened();
    }

    void Capture::close() {
        if (cap != NULL) {
            cap->release();

            delete cap;
            cap = NULL;
        }
    }

    void Capture::nextFrame(cv::Mat &frame) {
        if (cap != NULL) {
            *cap >> frame;
        }
    }
}