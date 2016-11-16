/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#ifndef ALICE_CAMERA_H
#define ALICE_CAMERA_H


#include <opencv2/opencv.hpp>

#define DEFAULT_FOCAL_LEN 500

namespace Alice {
    class Camera {
    public:
        Camera(double w, double h) :
                width(w),
                height(h),
                farClip(1000.0f),
                nearClip(0.1f) {
            // default calibration data
            camK = cv::Mat::eye(3, 3, CV_64F);
            camK.at<double>(0, 0) = DEFAULT_FOCAL_LEN;
            camK.at<double>(1, 1) = DEFAULT_FOCAL_LEN;
            camK.at<double>(0, 2) = width / 2;
            camK.at<double>(1, 2) = height / 2;

            camD = cv::Mat::zeros(1, 5, CV_64F);
        }

        void setFrameSize(double w, double h);

        void getGlProjMatrix(double m[16]);

        inline cv::Mat &getCamK() {
            return camK;
        }

        inline cv::Mat &getCamD() {
            return camD;
        }

    private:
        double width;
        double height;
        double farClip;
        double nearClip;

        cv::Mat camK, camD;
    };
}

#endif //ALICE_CAMERA_H
