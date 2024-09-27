#ifndef DEPTH_ALIGNER_H
#define DEPTH_ALIGNER_H

#include <Eigen/Dense>
#include <array>
#include <initializer_list>
#include <opencv2/opencv.hpp>

// grouth truth value for scale and shift 
const std::array<float,2> gt_ss_71{2158.7249,0.5174077};
const std::array<float,2> gt_ss_75{2134.702,1.1729892};

class DepthAligner
{
public:
    DepthAligner();
    void align(
        const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> &depth_preds,
        const std::pair<std::vector<cv::Point>, std::vector<cv::Point>> &pixel_cords,
        const Eigen::Matrix<double, 3, 3> &camera_intrinsics,
        const Eigen::Matrix3d &rotation,
        const Eigen::Vector3d &translation);
    void align(
        const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>>& depth_preds,
        const std::pair<std::vector<cv::Point>, std::vector<cv::Point>>& pixel_cords,
        const Eigen::Matrix<double, 3, 3>& camera_intrinsics,
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& translation,
        const std::initializer_list<int> &valid_indexes);
    Eigen::VectorXd linearLeastSquares(
        const Eigen::VectorXd &y,
        const Eigen::MatrixXd &H);
};

#endif