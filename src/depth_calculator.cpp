#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "depth_aligner.h"
#include "orb_matcher.h"
#include "utils.h"

void getPathConfigs(cv::FileStorage& fsSettings, std::vector<std::string>& file_paths)
{
    file_paths.push_back(fsSettings["rgb_path_1st"]);
    file_paths.push_back(fsSettings["rgb_path_2st"]);
    file_paths.push_back(fsSettings["tif_path_1st"]);
    file_paths.push_back(fsSettings["tif_path_2st"]);
    file_paths.push_back(fsSettings["cam2can_calib_path"]);
    file_paths.push_back(fsSettings["poses_path"]);
}

int main(int argc, char* argv[])
{
    std::string config_file = "./config/filePath.yaml";
    cv::FileStorage fsSettings(config_file.c_str(), cv::FileStorage::READ);

    std::vector<std::string> file_paths;
    getPathConfigs(fsSettings, file_paths);

    /* Read image. */
    cv::Mat img_1 = cv::imread(file_paths[0], cv::IMREAD_COLOR);
    if (img_1.empty()) {
        std::cerr << "Error: Could not open or find the img1." << std::endl;
        return -1;
    }

    cv::Mat img_2 = imread(file_paths[1], cv::IMREAD_COLOR);
    if (img_2.empty()) {
        std::cerr << "Error: Could not open or find the img2." << std::endl;
        return -1;
    }

    std::vector<std::vector<float>> depth_pred_1;
    if (!readTiffImage(file_paths[2], depth_pred_1)) {
        std::cerr << "Error: Could not open or find the depth_pred1" << std::endl;
        return -1;
    }
    disparity2depth(depth_pred_1, 40.0f);

    std::vector<std::vector<float>> depth_pred_2;
    if (!readTiffImage(file_paths[3], depth_pred_2)) {
        std::cerr << "Error: Could not open or find the depth_pred2" << std::endl;
        return -1;
    }
    disparity2depth(depth_pred_2, 40.0f);

    Eigen::Matrix<double, 3, 3> camera_intrinsics;
    if (!readIntrinsics(file_paths[4], camera_intrinsics)) {
        std::cerr << "Error: Could not open or find the calib." << std::endl;
        return -1;
    }

    std::vector<Eigen::Matrix3d> rotation;
    std::vector<Eigen::Vector3d> translation;
    int frame_1 = extractNumber(file_paths[0]);
    int frame_2 = extractNumber(file_paths[1]);
    if (!readPoses(file_paths[5], frame_1 + 1, frame_2 + 1, rotation, translation)) {
        std::cerr << "Error: Could not open or find the poses." << std::endl;
        return -1;
    }
    std::initializer_list<int> valid_indexes{79,33,30,56,100 ,102,108,305,393};
    // using ORB to match the keypoints
    ORBMatcher orb_matcher;
    // std::pair<std::vector<cv::Point>, std::vector<cv::Point>> pixel_cords = orb_matcher.match(img_1, img_2);

    // to load the precomputed orb matches
    auto pixel_cords = load_npy_points("/home/weison/LightGlue/m_kpts0.npy", "/home/weison/LightGlue/m_kpts1.npy");
    std::cout << "Number of orb pairs: " << pixel_cords.first.size() << std::endl; 

    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> depth_preds = std::make_pair(depth_pred_1, depth_pred_2);
    DepthAligner depth_aligner;
    depth_aligner.align(depth_preds, pixel_cords, camera_intrinsics, rotation[0], translation[0],valid_indexes);

    // depth_aligner.align(depth_preds, pixel_cords, camera_intrinsics, rotation[1], translation[1]);
    // depth_aligner.align(depth_preds, pixel_cords, camera_intrinsics, rotation[2], translation[2]);
    // depth_aligner.align(depth_preds, pixel_cords, camera_intrinsics, rotation[3], translation[3]);

    // orb_matcher.visualize_matches(img_1, img_2, pixel_cords.first, pixel_cords.second, "Matches",valid_indexes);
    orb_matcher.visualize_matches(img_1, img_2, pixel_cords.first, pixel_cords.second, "Matches",valid_indexes);
    cv::waitKey(0);

    return 0;
}