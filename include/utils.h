#ifndef _UTILS_H
#define _UTILS_H

#include <iostream>
#include <png++/png.hpp>
#include <tiffio.h>

extern void readDepthImage(
  const std::string file_name,
  std::vector<std::vector<float>> &data);


extern bool readTiffImage(
  const std::string file_name,
  std::vector<std::vector<float>> &data);

#endif