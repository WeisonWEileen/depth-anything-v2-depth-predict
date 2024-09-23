#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H


#include <initializer_list>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/highgui/highgui.hpp>

class ORBMatcher
{
public:
    ORBMatcher();
    std::pair<std::vector<cv::Point>, std::vector<cv::Point>> match(
        const cv::Mat &img_1,
        const cv::Mat &img_2);
    void show_keypoints(
        const cv::Mat &img,
        const std::vector<cv::KeyPoint> &keypoints,
        std::string& window_name);
    void show_matches(
        const cv::Mat &img_1,
        const cv::Mat &img_2,
        const std::vector<cv::KeyPoint> &keypoints_1,
        const std::vector<cv::KeyPoint> &keypoints_2,
        const std::vector<cv::DMatch> &matches,
        std::string& window_name);
    void visualize_matches(
        const cv::Mat &img_1,
        const cv::Mat &img_2,
        const std::vector<cv::Point> &pixel_cords_1,
        const std::vector<cv::Point> &pixel_cords_2,
        const std::string &window_name);

    void visualize_matches(
        const cv::Mat& img_1,
        const cv::Mat& img_2,
        const std::vector<cv::Point>& pixel_cords_1,
        const std::vector<cv::Point>& pixel_cords_2,
        const std::string& window_name,
        const std::initializer_list<int> & valid_indexes);

private:
    // cv::Ptr<cv::ORB> detector_;
    cv::Ptr<cv::ORB> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
};

#endif
