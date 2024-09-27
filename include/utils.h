#ifndef _UTILS_H
#define _UTILS_H

#include <string>

#include <opencv2/opencv.hpp>
#include <png++/png.hpp>
#include <tiffio.h>
#include <Eigen/Dense>
#include <vector>

void readDepthImage(
    std::string & file_name,
    std::vector<std::vector<float>> &data);

bool readTiffImage(
    std::string &file_name,
    std::vector<std::vector<float>> &data);

bool readIntrinsics(
    std::string & file_name,
    Eigen::Matrix<double, 3, 3> &camera_intrinsics);

int extractNumber(const std::string &path);

bool readPoses(
    std::string& file_name,
    int frame_1,
    int frame_2,
    std::vector<Eigen::Matrix3d> &rotation,
    std::vector<Eigen::Vector3d> &translation);

Eigen::Matrix4d readPoseFromLine(const std::string &line);

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> load_npy_points(const std::string& file1, const std::string& file2); 

void disparity2depth(std::vector<std::vector<float>>& img,float down_limit);

void depth_error_measure(const std::vector<cv::Point> &keyPoints,const std::vector<std::vector<float>>&  gt, const std::vector<std::vector<float>>& pred,const std::initializer_list<int>& valid_indexes);

/**
 * @brief based on scale and shift, convert relative depth to absolute depth
 *
 * @tparam Container
 * @param pred
 * @param scale_shift
 */
template <typename Container>
std::vector<std::vector<float>> rel2absDepth(const std::vector<std::vector<float>>& pred, const Container& scale_shift)
{
    std::vector<std::vector<float>> depth = pred;
    float scale = scale_shift[0];
    float shift = scale_shift[1];
    for (auto& row : depth) {
        std::transform(row.begin(), row.end(), row.begin(), [scale, shift](float val) {
            return val * scale + shift;
        });
    }
    return depth;
}
#endif