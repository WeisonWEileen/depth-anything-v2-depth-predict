cmake_minimum_required(VERSION 2.8)
project(depth_calculate)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(OpenCV 4.2.0 REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
# message(STATUS "Include directories: ${CMAKE_SOURCE_DIR}/include")


# add_executable(feature_extraction feature_extration.cpp)
# target_link_libraries(feature_extraction ${OpenCV_LIBS})

add_executable(depth_calculator src/depth_calculator.cpp src/orb_matcher.cpp)
target_link_libraries(depth_calculator ${OpenCV_LIBS})

