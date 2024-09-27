cmake_minimum_required(VERSION 2.8)
project(depth_calculate)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
# set on compile commands json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(PNG REQUIRED)
find_package(TIFF REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(CNPY_LIB cnpy)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
# message(STATUS "Include directories: ${CMAKE_SOURCE_DIR}/include")


# add_executable(feature_extraction feature_extration.cpp)
# target_link_libraries(feature_extraction ${OpenCV_LIBS})

add_executable(depth_calculator src/main.cpp src/utils.cpp src/orb_matcher.cpp src/depth_aligner.cpp)
target_link_libraries(depth_calculator ${OpenCV_LIBS} PNG::PNG TIFF::TIFF Eigen3::Eigen ${CNPY_LIB})

