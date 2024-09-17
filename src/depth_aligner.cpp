#include "depth_aligner.h"

DepthAligner::DepthAligner()
{
}

Eigen::VectorXd DepthAligner::linearLeastSquares(
    const Eigen::VectorXd &y,
    const Eigen::MatrixXd &H)
{
    Eigen::VectorXd beta = (H.transpose() * H).inverse() * H.transpose() * y;

    return beta;
}

void DepthAligner::align(
    const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> &depth_preds,
    const std::pair<std::vector<cv::Point>, std::vector<cv::Point>> &pixel_cords,
    const Eigen::Matrix<double, 3, 3> &camera_intrinsics,
    const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation,
    std::pair<float, float> &scales,
    std::pair<float, float> &shifts)
{

    Eigen::VectorXd y(int(3 * pixel_cords.first.size()));
    Eigen::MatrixXd H(int(3 * pixel_cords.first.size()), 4);

    for (int i = 0; i < pixel_cords.first.size(); i++)
    {
        y.segment<3>(3 * i) = translation;

        cv::Point pixel_1 = pixel_cords.first[i];
        cv::Point pixel_2 = pixel_cords.second[i];

        Eigen::Vector3d point_1 = Eigen::Vector3d(pixel_1.x, pixel_1.y, 1);
        Eigen::Vector3d point_2 = Eigen::Vector3d(pixel_2.x, pixel_2.y, 1);

        float depth_1 = depth_preds.first[pixel_1.y][pixel_1.x];
        float depth_2 = depth_preds.second[pixel_2.y][pixel_2.x];

        Eigen::Vector3d col_1 = depth_1 * camera_intrinsics.inverse() * point_1;
        Eigen::Vector3d col_2 = camera_intrinsics.inverse() * point_1;
        Eigen::Vector3d col_3 = -depth_2 * rotation * camera_intrinsics.inverse() * point_2;
        Eigen::Vector3d col_4 = -rotation * camera_intrinsics.inverse() * point_2;

        Eigen::Matrix<double, 3, 4> H_i;
        H_i.col(0) = col_1;
        H_i.col(1) = col_2;
        H_i.col(2) = col_3;
        H_i.col(3) = col_4;

        H.block<3, 4>(i * 3, 0) = H_i;
    }

    Eigen::VectorXd beta = linearLeastSquares(y, H);
    scales = std::make_pair(beta(0), beta(2));
    shifts = std::make_pair(beta(1), beta(3));
}