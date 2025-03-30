#ifndef CAMERA_H
#define CAMERA_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ObjectDetection.h>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <iostream>
#include "open3d/Open3D.h"




class Camera {


public:
    explicit Camera();                                                   // Constructor
    ~Camera();                                                  // Destructor

    // Delete copy constructor and assignment operator
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    rs2::frame color_frame, depth_frame;                        // Raw Frames
    cv::Mat color_mat, depth_mat ;                              // Frames in OpenCV format 
    float depth_scale;                                          // Depth scale for converting depth values
    
    void updateStreams();                                      
    void preprocessFrames(std::vector<float> &output); // Prepare & Process frame for Tensorrt engine
    // void displayStreams();                                      // Open 2 windows and display raw depth and color streams 
    // void displayDetections(const std::vector<DetectedObject>& detections);

    // Core Operations
    bool start();
    void stop();
    bool restart();

    // Status checks
    bool isConnected() const {return connected;}
    bool isRunning() const {return running;}


private:

    rs2::pipeline pipe;              // RealSense pipeline
    rs2::config config;              // Configuration for the pipeline
    rs2::align align_to;             // Align depth to color
    
    // Internal methods
    void updateLoop();
    bool initialize();
    bool reconnect();

    bool running;
    bool connected;

    // open3d::t::geometry::PointCloud current_PointCloud;
    std::unique_ptr<std::thread> update_thread;
    std::mutex stream_mutex;
    // FPS tracking
    std::chrono::steady_clock::time_point lastFrameTime;
    float fps;
    const float fpsAlpha = 0.1f;  // Smoothing factor for FPS moving average
    
    cv::Mat convertFrameToMat(const rs2::frame& frame); // Convert RealSense frame to OpenCV Mat
    void updateFPS();              // Update FPS calculation
    
    const std::chrono::milliseconds RECONNECT_DELAY{2500};



   
















};

#endif // CAMERA_H
