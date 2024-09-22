#include "cnpy.h"
#include "utils.h"

void readDepthImage(
    std::string & file_name,
    std::vector<std::vector<float>> &data)
{
  png::image<png::gray_pixel_16> image(file_name);
  size_t width = image.get_width();
  size_t height = image.get_height();

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
        data[v][u] = (static_cast<float>(val)) / 256.0f;
      }
    }
  }
}

bool readTiffImage(
    std::string& file_name,
    std::vector<std::vector<float>> &data)
{
  TIFF *tif = TIFFOpen(file_name.c_str(), "r");
  if (!tif)
  {
    std::cerr << "Failed to open the TIFF file." << std::endl;
    return false;
  }

  int32_t width{0}, height{0};
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

  return true;
}

bool readIntrinsics(
    std::string & file_name,
    Eigen::Matrix<double, 3, 3> &camera_intrinsics)
{
  std::ifstream infile(file_name);
  if (!infile.is_open())
  {
    std::cerr << "Failed to open the calibration file." << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(infile, line))
  {
    if (line.rfind("P_rect_02:", 0) == 0)
    {
      std::stringstream ss(line);
      std::string temp;
      ss >> temp;

      double value{0.0};

      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          ss >> value;
          if (j < 3)
          {
            camera_intrinsics(i, j) = value;
          }
        }
      }
      break;
    }
  }

  infile.close();
  return true;
}

int extractNumber(const std::string &path)
{
  size_t lastSlash = path.find_last_of('/');
  size_t lastDot = path.find_last_of('.');

  std::string fileName = path.substr(lastSlash + 1, lastDot - lastSlash - 1);

  int number = std::stoi(fileName);
  return number;
}

Eigen::Matrix4d readPoseFromLine(const std::string &line)
{
  std::stringstream ss(line);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      ss >> pose(i, j);
    }
  }
  return pose;
}

bool readPoses(
    std::string & file_name,
    int frame_1,
    int frame_2,
    Eigen::Matrix3d &rotation,
    Eigen::Vector3d &translation)
{
  std::ifstream infile(file_name);
  if (!infile.is_open())
  {
    std::cerr << "Failed to open the poses file." << std::endl;
    return false;
  }

  std::string line;
  int lineNumber = 0;
  Eigen::Matrix4d pose_1, pose_2;
  while (std::getline(infile, line))
  {
    lineNumber++;
    if (lineNumber == frame_1)
    {
      pose_1 = readPoseFromLine(line);
       
      std::cout << "Pose 1: " << std::endl << pose_1 << std::endl;
    }
    if (lineNumber == frame_2)
    {
      pose_2 = readPoseFromLine(line);
      std::cerr << "Pose 2: " << std::endl << pose_2 << std::endl;
    }
    if (lineNumber > frame_2)
    {
      break;
    }
  }

//   Eigen::Matrix4d transform = pose_2 * pose_1.inverse();
  Eigen::Matrix4d transform = pose_1 * pose_2.inverse();
//   rotation = transform.block<3, 3>(0, 0);
//   translation = transform.block<3, 1>(0, 3);
  rotation = pose_1.block<3, 3>(0, 0);
  translation = pose_1.block<3, 1>(0,3) ;

  infile.close();
  return true;
}

std::pair<std::vector<cv::Point>, std::vector<cv::Point>> load_npy_points(const std::string& file1, const std::string& file2) {
    // 读取 .npy 文件
    cnpy::NpyArray arr1 = cnpy::npy_load(file1);
    cnpy::NpyArray arr2 = cnpy::npy_load(file2);

    // 获取数据指针
    int* data1 = arr1.data<int>();
    int* data2 = arr2.data<int>();

    // 获取数组形状
    auto shape1 = arr1.shape;
    auto shape2 = arr2.shape;

    

    // 确保形状匹配
    if (shape1 != shape2 || shape1.size() != 2 || shape1[1] != 2) {
        throw std::runtime_error("Shape mismatch or invalid shape");
    }

    // 转换为 std::vector<cv::Point>
    std::vector<cv::Point> points1, points2;
    for (unsigned int i = 0; i < shape1[0]; ++i) {
        points1.emplace_back(data1[i * 2], data1[i * 2 + 1]);
        points2.emplace_back(data2[i * 2], data2[i * 2 + 1]);
    }

    return std::make_pair(points1, points2);
}
