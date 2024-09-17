#include "utils.h"
#include "orb_matcher.h"
#include "depth_aligner.h"

int main(int argc, char **argv)
{
    if (argc != 7)
    {
        std::cout << "Usage: depth_calculate img1 img2 depth_pred1 depth_pred2 calibs poses." << std::endl;
        return -1;
    }

    /* Read image. */
    cv::Mat img_1 = imread(argv[1], cv::IMREAD_COLOR);
    if (img_1.empty())
    {
        std::cerr << "Error: Could not open or find the img1." << std::endl;
        return -1;
    }

    cv::Mat img_2 = imread(argv[2], cv::IMREAD_COLOR);
    if (img_2.empty())
    {
        std::cerr << "Error: Could not open or find the img2." << std::endl;
        return -1;
    }

    // std::vector<std::vector<float>> depth_gt;
    // readDepthImage(argv[3], depth_gt);
    // if(depth_gt.empty())
    // {
    //     std::cerr << "Error: Could not open or find the image 'depth_gt.png'" << std::endl;
    //     return -1;
    // }

    std::vector<std::vector<float>> depth_pred_1;
    if (!readTiffImage(argv[3], depth_pred_1))
    {
        std::cerr << "Error: Could not open or find the depth_pred1" << std::endl;
        return -1;
    }

    std::vector<std::vector<float>> depth_pred_2;
    if (!readTiffImage(argv[4], depth_pred_2))
    {
        std::cerr << "Error: Could not open or find the depth_pred2" << std::endl;
        return -1;
    }

    Eigen::Matrix<double, 3, 3> camera_intrinsics;
    if (!readIntrinsics(argv[5], camera_intrinsics))
    {
        std::cerr << "Error: Could not open or find the calib." << std::endl;
        return -1;
    }

    // std::cerr << "Camera intrinsics: " << camera_intrinsics << std::endl;

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    int frame_1 = extractNumber(argv[1]);
    int frame_2 = extractNumber(argv[2]);
    // std::cerr << "Frame 1: " << frame_1 << " Frame 2: " << frame_2 << std::endl;
    if (!readPoses(argv[6], frame_1+1, frame_2+1, rotation, translation))
    {
        std::cerr << "Error: Could not open or find the poses." << std::endl;
        return -1;
    }

    // std::cerr << "Rotation: " << rotation << std::endl;
    // std::cerr << "Translation: " << translation << std::endl;
    // std::cerr << "Test: " << rotation*rotation.transpose() << std::endl;

    ORBMatcher orb_matcher;
    std::pair<std::vector<cv::Point>, std::vector<cv::Point>> pixel_cords = orb_matcher.match(img_1, img_2);
    // orb_matcher.visualize_matches(img_1, img_2, pixel_cords.first, pixel_cords.second, "Matches");
    // cv::waitKey(0);

    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> depth_preds = std::make_pair(depth_pred_1, depth_pred_2);
    DepthAligner depth_aligner;
    std::pair<float, float> scales;
    std::pair<float, float> shifts;
    depth_aligner.align(depth_preds, pixel_cords, camera_intrinsics, rotation, translation, scales, shifts);

    std::cerr << "Scales: " << scales.first << " " << scales.second << std::endl;
    std::cerr << "Shifts: " << shifts.first << " " << shifts.second << std::endl;

    return 0;
}