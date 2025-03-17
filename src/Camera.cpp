#include <Camera.h>


Camera::Camera() : align_to(RS2_STREAM_COLOR), fps(0.0f){
    // Configure the pipeline for color and depth streams
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Retrieve the depth scale from a temporary pipeline start
    rs2::pipeline_profile profile = pipe.start(config);
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    depth_scale = sensor.get_depth_scale();


}


Camera::~Camera() 
{
    pipe.stop();
}


cv::Mat Camera::convertFrameToMat(const rs2::frame& frame) 
{
    // Get frame dimensions
    int width = frame.as<rs2::video_frame>().get_width();
    int height = frame.as<rs2::video_frame>().get_height();

    // Convert to OpenCV Mat
    if (frame.get_profile().format() == RS2_FORMAT_BGR8) 
        return cv::Mat(cv::Size(width, height), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    else if (frame.get_profile().format() == RS2_FORMAT_Z16) 
        return cv::Mat(cv::Size(width, height), CV_16U, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    else 
        throw std::runtime_error("Unsupported frame format");
}


void Camera::updateFPS() {
    auto currentTime = std::chrono::steady_clock::now();
    float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
    lastFrameTime = currentTime;
    
    float currentFPS = 1.0f / deltaTime;
    fps = (fps * (1.0f - fpsAlpha)) + (currentFPS * fpsAlpha);
}


void Camera::updateFrames()
{
     // Wait for a synchronized frameset
    rs2::frameset frameset = pipe.wait_for_frames();

    // Align depth to color
    frameset = align_to.process(frameset);

    // Get color and depth frames
    color_frame = frameset.get_color_frame();
    depth_frame = frameset.get_depth_frame();
    
    color_mat = convertFrameToMat(color_frame);
    depth_mat = convertFrameToMat(depth_frame);

}


void Camera::preprocessFrames(std::vector<float> &output)
{
    cv::Mat resized;
    cv::resize(color_mat, resized, cv::Size(320, 320), 0, 0, cv::INTER_CUBIC);

    // Convert to NCHW format, normalize to [0,1]
    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0, cv::Size(320, 320), cv::Scalar(), true, false);

    // Ensure blob is in the correct shape
    if (blob.dims != 4 || blob.size[0] != 1 || blob.size[1] != 3 || blob.size[2] != 320 || blob.size[3] != 320)
    {
        std::cerr << "ERROR: blobFromImage() produced incorrect shape!" << std::endl;
        return;
    }
    std::memcpy(output.data(), blob.ptr<float>(0), 320 * 320 * 3 * sizeof(float));
}



void Camera::displayStreams()
{
    cv::Mat depth_display;
    depth_mat.convertTo(depth_display, CV_8UC1, 255.0 / 1000.0);
    
    cv::applyColorMap(depth_display,depth_display,cv::COLORMAP_JET);
    cv::imshow("Color Frame", color_mat);
    cv::imshow("Depth Frame", depth_display);
}


void Camera::displayDetections(const std::vector<DetectedObject>& detections)
{      
    
    updateFPS();
    cv::Mat display_image = color_mat.clone();
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
    cv::putText(display_image, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    
    for (const auto& det : detections) 
    {
        // Rest of your drawing code, but use scaled_bbox instead of det.bbox
        cv::Scalar color;
        std::string label;
        
        switch(det.classId) 
        {
            case 0:
                color = cv::Scalar(0, 255, 0);
                label = "MobileGoal";
                break;
            case 1:
                color = cv::Scalar(0, 0, 255);
                label = "RedRing";
                break;
            case 2:
                color = cv::Scalar(255, 0, 0);
                label = "BlueRing";
                break;
        }

        cv::rectangle(display_image, det.bbox, color, 2);
        label += " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

        cv::Point text_origin(det.bbox.x, det.bbox.y - 5);
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, nullptr);
        cv::rectangle(display_image, 
                     cv::Point(text_origin.x, text_origin.y - text_size.height),
                     cv::Point(text_origin.x + text_size.width, text_origin.y + 5),
                     color, -1);

        cv::putText(display_image, label, text_origin,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("Color Frame", display_image);
 
}
