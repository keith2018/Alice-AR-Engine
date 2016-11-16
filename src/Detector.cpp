/*
 *
 * Alice AR Engine
 *
 * @author 	: keith@robot9.me
 * @date	: 2016/9/20
 *
 */

#include "Detector.h"

#define REFINE_MATCH
#define ENABLE_TRACK_MASK

namespace Alice {

    bool Detector::addMarker(std::string markerPath) {
        cv::Mat markerImg = cv::imread(markerPath, CV_LOAD_IMAGE_GRAYSCALE);

        // create marker
        Marker marker;
        marker.imgPath = markerPath;
        marker.grayImg = markerImg;

        marker.width = markerImg.cols;
        marker.height = markerImg.rows;

        // quad points
        marker.quadPts2d.resize(4);

        marker.quadPts2d[0] = cv::Point2f(0, 0);
        marker.quadPts2d[1] = cv::Point2f(marker.width, 0);
        marker.quadPts2d[2] = cv::Point2f(marker.width, marker.height);
        marker.quadPts2d[3] = cv::Point2f(0, marker.height);

        // features
        bool isOk = extractFeatures(markerImg, marker.keyPts, marker.descriptors);
        if (!isOk) {
            return false;
        }

        markerHandler.addMarker(marker);
        return true;
    }

    void Detector::trainMarkers() {
        size_t markerCnt = markerHandler.size();

        std::vector<cv::Mat> descriptors;
        descriptors.resize(markerCnt);

        for (int i = 0; i < markerCnt; i++) {
            descriptors[i] = markerHandler.getMarker(i).descriptors.clone();
        }

        matcher->clear();
        matcher->add(descriptors);
        matcher->train();
    }

    bool Detector::extractFeatures(const cv::Mat &image,
                                   std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors,
                                   cv::Mat mask) {
        detector->detect(image, keyPoints, mask);
        if (keyPoints.empty() || keyPoints.size() < minMatchCnt) {
            return false;
        }

        extractor->compute(image, keyPoints, descriptors);

        return !keyPoints.empty();
    }

    void Detector::knnMatch(cv::Mat &desQuery, std::vector<cv::DMatch> &matches) {
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(desQuery, knnMatches, 2);

        for (size_t i = 0; i < knnMatches.size(); i++) {
            const cv::DMatch &bestMatch = knnMatches[i][0];
            const cv::DMatch &betterMatch = knnMatches[i][1];

            float distanceRatio = bestMatch.distance / betterMatch.distance;
            if (distanceRatio < minKnnRatio) {
                matches.push_back(bestMatch);
            }
        }
    }

    bool Detector::evaluateHomography(const std::vector<cv::KeyPoint> &queryKeyPts,
                                      const std::vector<cv::KeyPoint> &trainKeyPts,
                                      std::vector<cv::DMatch> &matches,
                                      cv::Mat &homography, FrameMarkerInfo &marker) {

        if (matches.size() < minMatchCnt) {
            return false;
        }

        std::vector<cv::Point2f> srcPoints, dstPoints;
        srcPoints.resize(matches.size());
        dstPoints.resize(matches.size());

        for (size_t i = 0; i < matches.size(); i++) {
            srcPoints[i] = trainKeyPts[matches[i].trainIdx].pt;
            dstPoints[i] = queryKeyPts[matches[i].queryIdx].pt;
        }

        int th_dist = (int) (sqrt(0.005 * marker.frameWidth * marker.frameHeight / M_PI) + 0.5);
        std::vector<unsigned char> inliersMask(srcPoints.size());

        homography = cv::findHomography(srcPoints, dstPoints, CV_FM_RANSAC, th_dist, inliersMask);

        std::vector<cv::DMatch> inliers;
        for (size_t i = 0; i < inliersMask.size(); i++) {
            if (inliersMask[i]) {
                inliers.push_back(matches[i]);
            }
        }

        matches.swap(inliers);
        if (matches.size() < minMatchCnt) {
            return false;
        }

#ifdef REFINE_MATCH
        std::vector<cv::Point2f> markerPos;
        cv::perspectiveTransform(marker.marker.quadPts2d, markerPos, homography);

        std::vector<cv::Point2f> inlierSrcPts;
        std::vector<cv::Point2f> inlierDstPts;
        for (int i = 0; i < inliersMask.size(); i++) {
            if (inliersMask[i] && isPointInQuad(dstPoints[i], markerPos)) {
                inlierSrcPts.push_back(srcPoints[i]);
                inlierDstPts.push_back(dstPoints[i]);
            }
        }

        if (inlierDstPts.size() < minMatchCnt) {
            return false;
        }

        th_dist = (int) (sqrt(0.005 * marker.marker.width * marker.marker.height / M_PI) + 0.5);
        homography = cv::findHomography(inlierSrcPts, inlierDstPts, CV_FM_RANSAC, th_dist, inliersMask);

        inliers.clear();
        for (size_t i = 0; i < inliersMask.size(); i++) {
            if (inliersMask[i]) {
                inliers.push_back(matches[i]);
            }
        }

        matches.swap(inliers);
        if (matches.size() < minMatchCnt) {
            return false;
        }
#endif

        cv::perspectiveTransform(marker.marker.quadPts2d, marker.quadPts2d, marker.homography);
        return checkValid(marker);
    }

    bool Detector::checkValid(FrameMarkerInfo &marker) {
        if (!checkRectShape(marker.quadPts2d)) {
            return false;
        }

        return true;
    }

    bool Detector::detectFrame(cv::Mat &frame) {
        // extract
        cv::Mat grayFrame;
        toGray(frame, grayFrame);

        int frameWidth = grayFrame.cols;
        int frameHeight = grayFrame.rows;

        // merge tracking mask
        cv::Mat trackMask;

#ifdef ENABLE_TRACK_MASK
        if (markerHandler.isAllMarkerVisible()) {
            for (int i = 0; i < markerHandler.size(); i++) {
                FrameMarkerInfo &info = markerHandler.getFrameMarkerInfo(i);
                if (info.visible) {
                    if (trackMask.empty()) {
                        trackMask = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
                    }
                    trackMask += info.trackingMask;
                }
            }
        }
#endif

        std::vector<cv::KeyPoint> frameKeyPoints;
        cv::Mat frameDescriptors;
        bool isOk = extractFeatures(grayFrame, frameKeyPoints, frameDescriptors, trackMask);
        if (!isOk) {
            resetAllMarkerStates();
            return false;
        }

        // match
        std::vector<cv::DMatch> matches;
        knnMatch(frameDescriptors, matches);

        if (matches.size() < minMatchCnt) {
            resetAllMarkerStates();
            return false;
        }

        std::vector<std::vector<cv::DMatch>> markerMatches;
        markerMatches.resize(markerHandler.size());
        for (int i = 0; i < matches.size(); i++) {
            cv::DMatch &m = matches[i];
            markerMatches[m.imgIdx].push_back(m);
        }

        // if any marker visible
        bool markerFound = false;

        // find homography
        for (int i = 0; i < markerHandler.size(); i++) {
            FrameMarkerInfo &currMarker = markerHandler.getFrameMarkerInfo(i);

            // reset state
            currMarker.frameWidth = frameWidth;
            currMarker.frameHeight = frameHeight;

            bool found = evaluateHomography(frameKeyPoints, currMarker.marker.keyPts,
                                            markerMatches[i], currMarker.homography, currMarker);
            if (!found) {
                currMarker.visible = false;
                currMarker.drawable.setVisible(false);
                continue;
            }

            currMarker.generateTrackingMask(trackWnd);
            currMarker.visible = true;

            currMarker.getGlMatrix(camera, currMarker.drawable.glMat);
            currMarker.drawable.setVisible(true);

            markerFound = true;
        }

        return markerFound;
    }

    void Detector::resetAllMarkerStates() {
        for (int i = 0; i < markerHandler.size(); i++) {
            FrameMarkerInfo &currMarker = markerHandler.getFrameMarkerInfo(i);
            currMarker.visible = false;
            currMarker.drawable.setVisible(false);
        }
    }

    void Detector::toGray(const cv::Mat &image, cv::Mat &gray) {
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, CV_BGR2GRAY);
        } else if (image.channels() == 4) {
            cv::cvtColor(image, gray, CV_BGRA2GRAY);
        } else if (image.channels() == 1) {
            gray = image;
        }
    }

    bool Detector::checkRectShape(std::vector<cv::Point2f> &rect_pts) {
        float vec[4][2];
        vec[0][0] = rect_pts[1].x - rect_pts[0].x;
        vec[0][1] = rect_pts[1].y - rect_pts[0].y;
        vec[1][0] = rect_pts[2].x - rect_pts[1].x;
        vec[1][1] = rect_pts[2].y - rect_pts[1].y;
        vec[2][0] = rect_pts[3].x - rect_pts[2].x;
        vec[2][1] = rect_pts[3].y - rect_pts[2].y;
        vec[3][0] = rect_pts[0].x - rect_pts[3].x;
        vec[3][1] = rect_pts[0].y - rect_pts[3].y;

        float val = vec[3][0] * vec[0][1] - vec[3][1] * vec[0][0];
        int s = val > 0 ? 1 : -1;

        bool ret = true;
        for (int i = 0; i < 3; i++) {
            val = vec[i][0] * vec[i + 1][1] - vec[i][1] * vec[i + 1][0];
            if (val * s <= 0) {
                ret = false;
                break;
            }
        }

        return ret;
    }

    bool Detector::isPointInQuad(cv::Point2f &pt, std::vector<cv::Point2f> &quadPts) {
        int nCount = 4;
        int nCross = 0;

        for (int i = 0; i < nCount; i++) {
            cv::Point2f &pStart = quadPts[i];
            cv::Point2f &pEnd = quadPts[(i + 1) % nCount];

            if (pStart.y == pEnd.y) {
                continue;
            }
            if (pt.y < std::min(pStart.y, pEnd.y) || pt.y > std::max(pStart.y, pEnd.y)) {
                continue;
            }
            double x = (pt.y - pStart.y) * (pEnd.x - pStart.x) / (pEnd.y - pStart.y) + pStart.x;
            if (x > pt.x) {
                nCross++;
            }
        }

        return (nCross % 2 == 1);
    }
}