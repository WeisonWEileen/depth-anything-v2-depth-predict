#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H

#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

class ORBMatcher
{
public:
    ORBMatcher();
    void match(
        const cv::Mat &img_1,
        const cv::Mat &img_2);
    void show_keypoints(
        const cv::Mat &img,
        const std::vector<cv::KeyPoint> &keypoints,
        std::string window_name);
    void show_matches(
        const cv::Mat &img_1,
        const cv::Mat &img_2,
        const std::vector<cv::KeyPoint> &keypoints_1,
        const std::vector<cv::KeyPoint> &keypoints_2,
        const std::vector<cv::DMatch> &matches,
        std::string window_name);

private:
    cv::Ptr<cv::ORB> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
};


#endif
