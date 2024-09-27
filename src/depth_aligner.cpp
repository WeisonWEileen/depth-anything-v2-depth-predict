#include "depth_aligner.h"
#include "utils.h"
#include <initializer_list>

DepthAligner::DepthAligner()
{
}

Eigen::VectorXd DepthAligner::linearLeastSquares(
    const Eigen::VectorXd& y,
    const Eigen::MatrixXd& H)
{
    Eigen::VectorXd beta = (H.transpose() * H).inverse() * H.transpose() * y;
    return beta;
}

/**
@brief
 *
 * @param depth_preds the pair of depth predictions using depth-anythiny vit
model
 * @param pixel_cords the matched pair of the original rgb images
 * @param camera_intrinsics
 * @param rotation 3×3 rotation Matrix
 * @param translation 3×1 translation vector
 * @param scales 2 output scales
 * @param shifts 2 output shifts
 */
void DepthAligner::align(
    const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>>& depth_preds,
    const std::pair<std::vector<cv::Point>, std::vector<cv::Point>>& pixel_cords,
    const Eigen::Matrix<double, 3, 3>& camera_intrinsics,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation)
{
    Eigen::VectorXd y(static_cast<int>(3 * pixel_cords.first.size()));
    Eigen::MatrixXd H(static_cast<int>(3 * pixel_cords.first.size()), 4);

    for (int i = 0; i < pixel_cords.first.size(); i++) {
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

        H.block<3, 1>(i * 3, 0) = col_1;
        H.block<3, 1>(i * 3, 1) = col_2;
        H.block<3, 1>(i * 3, 2) = col_3;
        H.block<3, 1>(i * 3, 3) = col_4;
    }

    Eigen::VectorXd beta = linearLeastSquares(y, H);

    std::cout << "scale_1 and shift_1  " << beta(0) << " " << beta(1) << std::endl;
    std::cout << "scale_2 and shift_2  " << beta(2) << " " << beta(3) << std::endl;
}

/**
 * @brief using prior knowledge of relative camera pose to calculate the scale and shift 
 * 
 * @param depth_preds depth anything vit model output
 * @param pixel_cords detected keypoints
 * @param camera_intrinsics 
 * @param rotation from i+1 frame to i frame
 * @param translation 
 * @param valid_indexes 
 */
void DepthAligner::align(
    const std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>>& depth_preds,
    const std::pair<std::vector<cv::Point>, std::vector<cv::Point>>& pixel_cords,
    const Eigen::Matrix<double, 3, 3>& camera_intrinsics,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation,
    const std::initializer_list<int>& valid_indexes)
{
    Eigen::VectorXd y(static_cast<int>(3 * valid_indexes.size()));
    Eigen::MatrixXd H(static_cast<int>(3 * valid_indexes.size()), 4);

    int j = 0;
    for (int i = 0; i < pixel_cords.first.size(); i++) {
        if (std::find(valid_indexes.begin(), valid_indexes.end(), i) != valid_indexes.end()) {
            y.segment<3>(3 * j) = translation;

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

            H.block<3, 1>(j * 3, 0) = col_1;
            H.block<3, 1>(j * 3, 1) = col_2;
            H.block<3, 1>(j * 3, 2) = col_3;
            H.block<3, 1>(j * 3, 3) = col_4;
            j++;
        }
    }

    Eigen::VectorXd beta = linearLeastSquares(y, H);
    std::array<float, 2> results_scsi_71 { static_cast<float>(beta(0)), static_cast<float>(beta(1)) };
    std::array<float, 2> results_scsi_75 { static_cast<float>(beta(2)), static_cast<float>(beta(3)) };
    std::cout << "scale_1 and shift_1  " << results_scsi_71[0] << " " << results_scsi_71[1] << std::endl;
    std::cout << "scale_2 and shift_2  " << results_scsi_75[0] << " " << results_scsi_75[1] << std::endl;
    auto depth_gt_71 = rel2absDepth(depth_preds.first, gt_ss_71);
    auto depth_gt_75 = rel2absDepth(depth_preds.second, gt_ss_75);
    auto depth_71 = rel2absDepth(depth_preds.first, results_scsi_71);
    auto depth_75 = rel2absDepth(depth_preds.second, results_scsi_75);

    depth_error_measure(pixel_cords.first,depth_gt_71, depth_71, valid_indexes);
    depth_error_measure(pixel_cords.second,depth_gt_75, depth_75, valid_indexes);
}
