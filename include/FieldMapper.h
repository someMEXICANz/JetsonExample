#ifndef FIELD_MAPPER_H
#define FIELD_MAPPER_H

#include <thread>
#include <mutex>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/Geometry.h>


#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "RobotPosition.h"


class FieldMapper {
    

    public:

    // Delete copy constructor and assignment operator
    FieldMapper(const FieldMapper&) = delete;
    FieldMapper& operator=(const FieldMapper&) = delete;\

    void start();
    void stop();

    cv::Mat getOccumpancyMap() const;



    private:


    void updateLoop();
    std::unique_ptr<std::thread> write_thread;
        mutable std::mutex map_mutex;
    bool running;
    



};

#endif FIELD_MAPPER_H
