/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_DETECTOR_H
#define ALICE_DETECTOR_H


#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "Marker.h"
#include "Camera.h"


namespace Alice {
    class BaseDetector {
    public:
        virtual ~BaseDetector() {};

        // marker
        virtual bool addMarker(std::string markerPath) = 0;

        virtual void trainMarkers() = 0;

        // frame
        virtual bool detectFrame(cv::Mat &frame) = 0;

        // for debug
        static void draw2dContour(cv::Mat &image, std::vector<cv::Point2f> pts, cv::Scalar color) {
            for (size_t i = 0; i < pts.size(); i++) {
                cv::line(image, pts[i], pts[(i + 1) % pts.size()], color, 2, CV_AA);
            }
        }
    };


    class Detector : public BaseDetector {
    public:
        Detector(Camera &cam, MarkerHandler &handler) :
                camera(cam),
                markerHandler(handler),
                detector(new cv::SURF(500, 4)),
                extractor(new cv::SURF(500, 4)),
                matcher(new cv::FlannBasedMatcher()),
                trackWnd(20),
                minMatchCnt(10),
                minKnnRatio(0.75f) {}

        bool addMarker(std::string markerPath);

        void trainMarkers();

        bool detectFrame(cv::Mat &frame);

    protected:
        bool extractFeatures(const cv::Mat &image,
                             std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                             cv::Mat mask = cv::Mat());

        void knnMatch(cv::Mat &desQuery, std::vector<cv::DMatch> &matches);

        bool evaluateHomography(const std::vector<cv::KeyPoint> &queryKeyPts,
                                const std::vector<cv::KeyPoint> &trainKeyPts,
                                std::vector<cv::DMatch> &matches,
                                cv::Mat &homography, FrameMarkerInfo &marker);

        bool checkValid(FrameMarkerInfo &marker);

        void resetAllMarkerStates();

    protected:
        static void toGray(const cv::Mat &image, cv::Mat &gray);

        static bool checkRectShape(std::vector<cv::Point2f> &rect_pts);

        static bool isPointInQuad(cv::Point2f &pt, std::vector<cv::Point2f> &quadPts);

    private:
        Camera &camera;
        MarkerHandler &markerHandler;

        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> extractor;
        cv::Ptr<cv::DescriptorMatcher> matcher;

        int trackWnd;
        int minMatchCnt;
        float minKnnRatio;
    };
}


#endif //ALICE_DETECTOR_H
