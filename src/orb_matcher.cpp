#include "orb_matcher.h"

ORBMatcher::ORBMatcher()
{
    detector_ = cv::ORB::create();
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void ORBMatcher::show_keypoints(
    const cv::Mat &img,
    const std::vector<cv::KeyPoint> &keypoints,
    std::string window_name)
{
    cv::Mat outimg;
    cv::drawKeypoints(img, keypoints, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow(window_name, outimg);

    // cv::waitKey(0);
}

void ORBMatcher::show_matches(
    const cv::Mat &img_1,
    const cv::Mat &img_2,
    const std::vector<cv::KeyPoint> &keypoints_1,
    const std::vector<cv::KeyPoint> &keypoints_2,
    const std::vector<cv::DMatch> &matches,
    std::string window_name)
{
    cv::Mat img_match;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    cv::imshow(window_name, img_match);

    cv::waitKey(0);
}

void ORBMatcher::match(
    const cv::Mat &img_1,
    const cv::Mat &img_2)
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
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    std::cout << "Max dist: " << max_dist << std::endl;
    std::cout << "Min dist: " << min_dist << std::endl;

    // Filter Match Pairs.
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Calculate time: " << duration.count() << "ms" << std::endl;

    // show_matches(img_1, img_2, keypoints_1, keypoints_2, matches, "All Matches");
    show_matches(img_1, img_2, keypoints_1, keypoints_2, good_matches, "Good Matches");
}