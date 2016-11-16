/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_MARKER_H
#define ALICE_MARKER_H


#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "ArViewer.h"

namespace Alice {
    class Marker {
    public:
        Marker() : id(-1), width(0), height(0) {}

    public:
        int id;
        std::string imgPath;
        cv::Mat grayImg;

        int width;
        int height;
        std::vector<cv::Point2f> quadPts2d;

        std::vector<cv::KeyPoint> keyPts;
        cv::Mat descriptors;
    };


    class FrameMarkerInfo {
    public:
        FrameMarkerInfo(Marker &m) :
                marker(m),
                frameWidth(0), frameHeight(0),
                visible(false) {}

        void generateTrackingMask(int expand) {
            cv::Rect rect = cv::boundingRect(quadPts2d);
            if (expand > 0) {
                rect.x -= expand;
                rect.y -= expand;
                rect.width += 2 * expand;
                rect.height += 2 * expand;
            }

            if (rect.x < 0) rect.x = 0;
            if (rect.y < 0) rect.y = 0;
            if (rect.width + rect.x > frameWidth) rect.width = frameWidth - rect.x - 1;
            if (rect.height + rect.y > frameHeight) rect.height = frameHeight - rect.y - 1;

            trackingMask = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
            trackingMask(rect).setTo(255);
        }

        void getGlMatrix(Camera &camera, double m[16]) {
            std::vector<cv::Point3f> objPts;
            for (int i = 0; i < 4; i++) {
                cv::Point3f pt(marker.quadPts2d[i].x, marker.quadPts2d[i].y, 0);

                pt.x -= marker.width / 2;
                pt.y -= marker.height / 2;

                pt.x /= DEFAULT_FOCAL_LEN;
                pt.y /= DEFAULT_FOCAL_LEN;

                objPts.push_back(pt);
            }

            cv::Mat r, R, t;
            cv::solvePnP(objPts, quadPts2d, camera.getCamK(), camera.getCamD(), r, t);
            cv::Rodrigues(r, R);

            // revert y & z
            cv::Mat mirror;
            mirror = cv::Mat::eye(3, 3, CV_64F);
            mirror.at<double>(1, 1) = -1;
            mirror.at<double>(2, 2) = -1;

            R = mirror * R;
            t = mirror * t;

            R = R.t();

            memset(m, 0, sizeof(double) * 16);
            for (int i = 0; i < 3; i++) {
                memcpy(&m[i * 4], R.ptr(i), sizeof(double) * 3);
            }
            memcpy(&m[12], t.data, sizeof(double) * 3);
            m[15] = 1;

            drawable.setSize(marker.width / (float) DEFAULT_FOCAL_LEN,
                             marker.height / (float) DEFAULT_FOCAL_LEN);
        }

    public:
        Marker marker;

        cv::Mat homography;

        int frameWidth;
        int frameHeight;
        cv::Mat trackingMask;
        std::vector<cv::Point2f> quadPts2d;

        bool visible;

        Drawable drawable;
    };


    class MarkerHandler {
    public:
        MarkerHandler() : markerIndex(0) {}

        inline size_t size() {
            return markers.size();
        }

        void addMarker(Marker &marker) {
            marker.id = markerIndex;

            FrameMarkerInfo info(marker);
            markers.push_back(info);

            markerIndex++;
        }

        inline Marker &getMarker(int id) {
            return markers[id].marker;
        }

        inline FrameMarkerInfo &getFrameMarkerInfo(int idx) {
            return markers[idx];
        }

        bool isAllMarkerVisible() {
            bool visible = true;
            for (int i = 0; i < size(); i++) {
                if (!markers[i].visible) {
                    visible = false;
                    break;
                }
            }

            return visible;
        }

    private:
        int markerIndex;
        std::vector<FrameMarkerInfo> markers;
    };
}


#endif //ALICE_MARKER_H
