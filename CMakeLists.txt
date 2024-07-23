cmake_minimum_required(VERSION 3.10)
project(slam)

set(CMAKE_CXX_STANDARD 20)

# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
set(pybind11_DIR "~/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11")
find_package(pybind11 REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)

# Add Eigen to the include directories
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/ext/eigen-3.4.0  # Adjust this path to the location of Eigen in your project
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/ext/matplotlibcpp
        ${pybind11_INCLUDE_DIRS}
        ${fmt_INCLUDE_DIRS}
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
        fmt::fmt  # Link the fmt library
        ${CMAKE_THREAD_LIBS_INIT}  # Link the pthread library
)

# Enable debug info in executable
set_target_properties(slam PROPERTIES COMPILE_FLAGS "-g")
