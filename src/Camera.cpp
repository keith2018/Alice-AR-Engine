/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "Camera.h"

namespace Alice {

    void Camera::getGlProjMatrix(double m[16]) {
        // http://www.codinglabs.net/article_world_view_projection_matrix.aspx
        m[0] = 2.0f * camK.at<double>(0, 0) / width;
        m[1] = 0;
        m[2] = 0;
        m[3] = 0;
        m[4] = 2.0f * camK.at<double>(0, 1) / width;
        m[5] = 2.0f * camK.at<double>(1, 1) / height;
        m[6] = 0;
        m[7] = 0;
        m[8] = -(2.0f * camK.at<double>(0, 2) / width) + 1.0f;
        m[9] = (2.0f * camK.at<double>(1, 2) / height) - 1.0f;
        m[10] = -(farClip + nearClip) / (farClip - nearClip);
        m[11] = -1.0f;
        m[12] = 0;
        m[13] = 0;
        m[14] = -2.0f * farClip * nearClip / (farClip - nearClip);
        m[15] = 0;
    }

    void Camera::setFrameSize(double w, double h) {
        camK.at<double>(0, 0) *= w / width;
        camK.at<double>(0, 2) *= w / width;
        camK.at<double>(1, 1) *= h / height;
        camK.at<double>(1, 2) *= h / height;

        width = w;
        height = h;
    }
}