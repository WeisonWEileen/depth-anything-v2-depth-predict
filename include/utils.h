#ifndef _UTILS_H
#define _UTILS_H

#include <string>

#include <png++/png.hpp>
#include <tiffio.h>
#include <Eigen/Dense>

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
    Eigen::Matrix3d &rotation,
    Eigen::Vector3d &translation);

Eigen::Matrix4d readPoseFromLine(const std::string &line);

#endif