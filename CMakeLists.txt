cmake_minimum_required(VERSION 3.10)
project(slam)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Correct path for pybind11_DIR
set(pybind11_DIR "~/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11")
find_package(pybind11 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/ext/matplotlibcpp
)

add_executable(slam
  src/main.cpp
  src/process_sensor_data.cpp
  src/single_robot_mapping.cpp
)

target_link_libraries(slam
  ${OpenCV_LIBS}
  Python3::Python
  Python3::NumPy
  pybind11::module
)

