/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include <thread>

#include "Pipeline.h"
#include "Capture.h"
#include "Timer.h"
#include "Config.h"


namespace Alice {
    void Pipeline::videoCapThreadFunc() {
        for (int i = 0; i < handler.size(); i++) {
            ArViewer::addDrawable(&(handler.getFrameMarkerInfo(i).drawable));
        }

        Capture capture;
        capture.open(width, height, camIndex);

        bool markerFound = false;
        cv::Mat frame;

        Timer timer;
        char fpsStr[10];

        while (!quitApp) {
            capture.nextFrame(frame);

            timer.start();
            markerFound = detector.detectFrame(frame);
            timer.stop();

            // show marker quad
            if (Config::showMarkerQuad) {
                if (markerFound) {
                    for (int i = 0; i < handler.size(); i++) {
                        if (handler.getFrameMarkerInfo(i).visible) {
                            Detector::draw2dContour(frame, handler.getFrameMarkerInfo(i).quadPts2d, CV_RGB(200, 0, 0));
                        }
                    }
                }
            }

            // show fps
            if (Config::showFPS) {
                sprintf(fpsStr, "FPS %02d", int(1000.0f / timer.elapseMillis()));
                cv::putText(frame, fpsStr, cv::Point(5, 15), CV_FONT_HERSHEY_PLAIN, 1.0f, CV_RGB(255, 0, 0));
            }

            // draw video frame
            ArViewer::setVideoFrame(frame);
        }

        capture.close();

        std::cout << "quit capture thread.\n";
        exit(0);
    }

    void Pipeline::addMarker(std::string markerPath) {
        detector.addMarker(markerPath);
    }

    void Pipeline::start() {
        if (!markerInited) {
            TIMED("detector.trainMarkers") {
                detector.trainMarkers();
            }
            markerInited = true;
        }

        std::thread capThread(&Pipeline::videoCapThreadFunc, this);

        double projectMatrix[16];
        camera.getGlProjMatrix(projectMatrix);
        ArViewer::setGlProjMatrix(projectMatrix);

        ArViewer::onQuitCmd(&quitApp);
        ArViewer::start(0, NULL, width, height);
    }

}