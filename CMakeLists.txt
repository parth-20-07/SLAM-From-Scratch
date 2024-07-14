cmake_minimum_required(VERSION 3.10)
project(slam)

set(CMAKE_CXX_STANDARD 17)

# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

find_package(OpenCV REQUIRED)


include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(slam
        src/main.cpp
        src/process_sensor_data.cpp
        src/single_robot_mapping.cpp
)

target_link_libraries(slam
        ${OpenCV_LIBS}
)

# Enable debug info in executable
set_target_properties(slam PROPERTIES COMPILE_FLAGS "-g")
