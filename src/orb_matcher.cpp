#include "orb_matcher.h"
#include <initializer_list>
#include <opencv2/imgproc.hpp>

const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
const double fontScale = 0.5;
const int thickness = 1;

ORBMatcher::ORBMatcher()
{
    detector_ = cv::ORB::create();
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void ORBMatcher::show_keypoints(
    const cv::Mat& img,
    const std::vector<cv::KeyPoint>& keypoints,
    std::string& window_name)
{
    cv::Mat outimg;
    cv::drawKeypoints(img, keypoints, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow(window_name, outimg);
}

void ORBMatcher::show_matches(
    const cv::Mat& img_1,
    const cv::Mat& img_2,
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector<cv::DMatch>& matches,
    std::string& window_name)
{
    cv::Mat img_match;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    cv::imshow(window_name, img_match);
}

void ORBMatcher::visualize_matches(
    const cv::Mat& img_1,
    const cv::Mat& img_2,
    const std::vector<cv::Point>& pixel_cords_1,
    const std::vector<cv::Point>& pixel_cords_2,
    const std::string& window_name) 
{
    cv::Mat img_combined;
    cv::hconcat(img_1, img_2, img_combined);

    // std::initializer_list<int> valid_indexes{ 11,12,13,14,79, 33, 30, 56,99,100,101,102,103,104,105,106,107,108};
    for (int i = 0; i < pixel_cords_1.size(); i++) {
            cv::Point pt1 = pixel_cords_1[i];
            cv::Point pt2 = pixel_cords_2[i] + cv::Point(img_1.cols, 0);

            cv::line(img_combined, pt1, pt2, cv::Scalar(0, 255, 0), 1);
            cv::circle(img_combined, pt1, 5, cv::Scalar(0, 0, 255), 0);
            cv::circle(img_combined, pt2, 5, cv::Scalar(0, 0, 255), 0);

            // 将数字 i 绘制在第一个圆圈的中心
            std::string text = std::to_string(i);
            cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, 0);
            cv::Point textOrg(pt1.x - textSize.width / 2, pt1.y + textSize.height / 2);
            cv::putText(img_combined, text, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
    }
    cv::imshow(window_name, img_combined);
}

void ORBMatcher::visualize_matches(
    const cv::Mat& img_1,
    const cv::Mat& img_2,
    const std::vector<cv::Point>& pixel_cords_1,
    const std::vector<cv::Point>& pixel_cords_2,
    const std::string& window_name, const std::initializer_list<int>& valid_indexes)
{
    cv::Mat img_combined;
    cv::hconcat(img_1, img_2, img_combined);

    // std::initializer_list<int> valid_indexes{ 11,12,13,14,79, 33, 30, 56,99,100,101,102,103,104,105,106,107,108};
    for (int i = 0; i < pixel_cords_1.size(); i++) {
        if (std::find(valid_indexes.begin(), valid_indexes.end(), i) != valid_indexes.end()) {
            cv::Point pt1 = pixel_cords_1[i];
            cv::Point pt2 = pixel_cords_2[i] + cv::Point(img_1.cols, 0);

            cv::line(img_combined, pt1, pt2, cv::Scalar(0, 255, 0), 1);
            cv::circle(img_combined, pt1, 5, cv::Scalar(0, 0, 255), 0);
            cv::circle(img_combined, pt2, 5, cv::Scalar(0, 0, 255), 0);

            // 将数字 i 绘制在第一个圆圈的中心
            std::string text = std::to_string(i);
            cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, 0);
            cv::Point textOrg(pt1.x - textSize.width / 2, pt1.y + textSize.height / 2);
            cv::putText(img_combined, text, textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
        }
    }

    cv::imshow(window_name, img_combined);
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> ORBMatcher::match(
    const cv::Mat& img_1,
    const cv::Mat& img_2)
{
    auto start = std::chrono::high_resolution_clock::now();

    /* Initialize. */
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    /* Detect oriented FAST keypoints. */
    detector_->detect(img_1, keypoints_1);
    detector_->detect(img_2, keypoints_2);

    /* Calculate BRIEF descriptors with the detected keypoints. */
    detector_->compute(img_1, keypoints_1, descriptors_1);
    detector_->compute(img_2, keypoints_2, descriptors_2);

    // show_keypoints(img_1, keypoints_1, "ORB Keypoints");

    /* Match BRIEF descriptors in two images using the Hamming distance. */
    std::vector<cv::DMatch> matches;
    matcher_->match(descriptors_1, descriptors_2, matches);

    // Find the minimum and maximum distance between all matches.
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    // std::cout << "Max dist: " << max_dist << std::endl;
    // std::cout << "Min dist: " << min_dist << std::endl;

    // Filter Match Pairs.
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point> pixel_cords_1, pixel_cords_2;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= std::max(2 * min_dist, 20.0)) {
            good_matches.push_back(matches[i]);
            pixel_cords_1.push_back(cv::Point(static_cast<int>(keypoints_1[matches[i].queryIdx].pt.x),
                static_cast<int>(keypoints_1[matches[i].queryIdx].pt.y)));
            pixel_cords_2.push_back(cv::Point(static_cast<int>(keypoints_2[matches[i].trainIdx].pt.x),
                static_cast<int>(keypoints_2[matches[i].trainIdx].pt.y)));
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;

    return std::make_pair(pixel_cords_1, pixel_cords_2);
}