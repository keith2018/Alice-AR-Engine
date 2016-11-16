/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_PIPELINE_H
#define ALICE_PIPELINE_H


#include <iostream>

#include "Camera.h"
#include "Marker.h"
#include "Detector.h"


namespace Alice {
    class Pipeline {
    public:
        Pipeline(int camIdx, int w, int h) :
                camera(w, h),
                detector(camera, handler),
                markerInited(false),
                quitApp(false),
                camIndex(camIdx),
                width(w),
                height(h) {}

        void addMarker(std::string markerPath);

        void start();

    private:
        void videoCapThreadFunc();

    private:
        Camera camera;
        MarkerHandler handler;
        Detector detector;

    private:
        int camIndex;
        int width;
        int height;

        bool quitApp;
        bool markerInited;
    };
}


#endif //ALICE_PIPELINE_H
