#ifndef DEPTH_ALIGNER_H
#define DEPTH_ALIGNER_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class DepthAligner
{
public:
    DepthAligner();
    void align(
        const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> &depth_preds,
        const std::pair<std::vector<cv::Point>, std::vector<cv::Point>> &pixel_cords,
        const Eigen::Matrix<double, 3, 3> &camera_intrinsics,
        const Eigen::Matrix3d &rotation,
        const Eigen::Vector3d &translation,
        std::pair<float, float> &scales,
        std::pair<float, float> &shift);
    Eigen::VectorXd linearLeastSquares(
        const Eigen::VectorXd &y,
        const Eigen::MatrixXd &H);
};

#endif