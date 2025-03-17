#ifndef CAMERA_H
#define CAMERA_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <stdexcept>
#include <iostream>
#include <ObjectDetection.h>


class Camera {
private:
    rs2::pipeline pipe;              // RealSense pipeline
    rs2::config config;              // Configuration for the pipeline
    rs2::align align_to;             // Align depth to color
                 
    // FPS tracking
    std::chrono::steady_clock::time_point lastFrameTime;
    float fps;
    const float fpsAlpha = 0.1f;  // Smoothing factor for FPS moving average
    
    cv::Mat convertFrameToMat(const rs2::frame& frame); // Convert RealSense frame to OpenCV Mat
    void updateFPS();              // Update FPS calculation

public:
    Camera();                                                   // Constructor
    ~Camera();                                                  // Destructor
    rs2::frame color_frame, depth_frame;                        // Raw Frames
    cv::Mat color_mat, depth_mat ;                              // Frames in OpenCV format 
    float depth_scale;                                          // Depth scale for converting depth values
    
    void updateFrames();                                      
    void preprocessFrames(std::vector<float> &output);          // Prepare & Process frame for Tensorrt engine
    void displayStreams();                                      // Open 2 windows and display raw depth and color streams 
    void displayDetections(const std::vector<DetectedObject>& detections);
   
};

#endif // CAMERA_H
