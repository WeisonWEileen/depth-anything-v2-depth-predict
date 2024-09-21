#include "orb_matcher.h"

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

    for (int i = 0; i < pixel_cords_1.size(); i++) {
        cv::Point pt1 = pixel_cords_1[i];
        cv::Point pt2 = pixel_cords_2[i] + cv::Point(img_1.cols, 0);

        cv::line(img_combined, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        cv::circle(img_combined, pt1, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(img_combined, pt2, 5, cv::Scalar(0, 0, 255), -1);
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
    // std::cout << "Calculate time: " << duration.count() << "ms" << std::endl;

    // show_matches(img_1, img_2, keypoints_1, keypoints_2, matches, "All Matches");
    // show_matches(img_1, img_2, keypoints_1, keypoints_2, good_matches, "Good Matches");
    visualize_matches(img_1, img_2, pixel_cords_1, pixel_cords_2, "Good Pairs");

    cv::waitKey(0);

    return std::make_pair(pixel_cords_1, pixel_cords_2);
}