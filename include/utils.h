#ifndef _UTILS_H
#define _UTILS_H

#include <iostream>
#include <string>

#include <png++/png.hpp>
#include <tiffio.h>

#include <Eigen/Dense>

extern void readDepthImage(
    std::string file_name,
    std::vector<std::vector<float>> &data);

extern bool readTiffImage(
    std::string file_name,
    std::vector<std::vector<float>> &data);

extern bool readIntrinsics(
    std::string file_name,
    Eigen::Matrix<double, 3, 3> &camera_intrinsics);

extern int extractNumber(const std::string &path);

extern bool readPoses(
    std::string file_name,
    int frame_1,
    int frame_2,
    Eigen::Matrix3d &rotation,
    Eigen::Vector3d &translation);

extern Eigen::Matrix4d readPoseFromLine(const std::string &line);

#endif