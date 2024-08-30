#include "utils.h"

void readDepthImage(
    const std::string file_name,
    std::vector<std::vector<float>> &data)
{
  png::image<png::gray_pixel_16> image(file_name);
  int32_t width = image.get_width();
  int32_t height = image.get_height();

  data.resize(height, std::vector<float>(width));
  for (int32_t v = 0; v < height; v++)
  {
    for (int32_t u = 0; u < width; u++)
    {
      uint16_t val = image.get_pixel(u, v);
      if (val == 0)
      {
        data[v][u] = -1.0;
      }
      else
      {
        data[v][u] = ((float)val) / 256.0;
      }
    }
  }

  // std::cout << "Depth image: " << std::endl;

  // for (int32_t v = 0; v < height; v++)
  // {
  //   for (int32_t u = 0; u < width; u++)
  //   {
  //     std::cout << data[v][u] << " ";
  //   }
  //   std::cout << std::endl;
  // }
}

bool readTiffImage(
    const std::string file_name,
    std::vector<std::vector<float>> &data)
{
  TIFF *tif = TIFFOpen(file_name.c_str(), "r");
  if (!tif)
  {
    std::cerr << "Failed to open the TIFF file." << std::endl;
    return false;
  }

  int32_t width, height;
  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);

  for (int32_t v = 0; v < height; v++)
  {
    std::vector<float> row;
    row.resize(width);
    TIFFReadScanline(tif, &row[0], v);
    data.push_back(row);
  }

  TIFFClose(tif);

  // std::cout << "Predicted Depth image: " << std::endl;

  // for (int32_t v = 0; v < height; v++)
  // {
  //   for (int32_t u = 0; u < width; u++)
  //   {
  //     std::cout << data[v][u] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  return true;
}
