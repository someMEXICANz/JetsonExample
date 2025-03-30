cmake_minimum_required(VERSION 3.19)
project(CameraTest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # This is important for VSCode
add_definitions(-Wno-deprecated-declarations)

# Find CUDA
find_package(CUDAToolkit REQUIRED)
include_directories(${CUDAToolkit_INCLUDE_DIRS})
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)


find_package(Boost REQUIRED COMPONENTS thread system)

# Find the i2c library
find_library(I2C_LIBRARY i2c PATHS /usr/lib/aarch64-linux-gnu)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Open3D
find_package(Open3D REQUIRED)

# Find RealSense
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}) # Add Eigen Include Path
add_definitions(-DEIGEN_USE_THREADS)

# Find I2C library (for IMU)
find_library(I2C_LIBRARY NAMES i2c)
if(NOT I2C_LIBRARY)
    message(FATAL_ERROR "libi2c not found, please install 'libi2c-dev' package")
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/include 
    ${OpenCV_INCLUDE_DIRS}
    ${REALSENSE2_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}  
)

# Link directories
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${REALSENSE2_LIBRARY_DIRS})

# Add executable (Including GPS files)
add_executable(CameraTest
    src/main.cpp
    src/Camera.cpp
    src/Model.cpp
    src/ObjectDetection.cpp
    src/GPS.cpp
    src/Position.cpp
    src/RobotPosition.cpp
    src/BrainCommStats.cpp
    src/BrainComm.cpp
    src/IMU.cpp
    src/UPS.cpp
)



target_link_libraries(CameraTest
    ${OpenCV_LIBS}
    ${REALSENSE2_LIBRARIES}
    ${LIBSERIAL_LIBRARIES}
    ${Boost_LIBRARIES}  
    ${I2C_LIBRARY}
    Open3D::Open3D
    cudart
    nvinfer
    nvonnxparser
    rt
    pthread
    
)