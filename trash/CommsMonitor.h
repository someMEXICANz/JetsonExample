#ifndef BRAIN_COMM_MONITOR_H
#define BRAIN_COMM_MONITOR_H

#include "BrainComm.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

namespace Brain {

class CommsMonitor {
public:
    // Constructor with window title
    explicit CommsMonitor(const std::string& title = "BrainComm Monitor");
    ~CommsMonitor();

    // No copy or move
    CommsMonitor(const CommsMonitor&) = delete;
    CommsMonitor& operator=(const CommsMonitor&) = delete;

    // Start/stop the monitor
    void start();
    void stop();

    // Update monitor with latest data from BrainComm
    void updateData(const BrainComm& comm);

private:
    // UI update thread function
    void updateLoop();
    void copyTransmitStats(TransmitStats& dest, const TransmitStats& src);
    void copyReceiveStats(ReceiveStats& dest, const ReceiveStats& src);
    
    // Rendering functions
    void renderDisplay();
    void drawSection(const std::string& title, int y_pos);
    void drawGPSData(const std::string& label, const Position2D& pos, int y_pos);
    void drawStats(const std::string& label, size_t value, int y_pos);
    void drawStats(const std::string& label, long long value, const std::string& unit, int y_pos);
    void drawConnectionVisualization(int x_pos, int y_pos);
    
    // Helper methods
    std::string errorTypeToString(CommError error);

    // Window properties
    std::string window_title;
    cv::Mat display_buffer;
    
    // Thread management
    std::thread update_thread;
    std::atomic<bool> is_running;
    std::mutex data_mutex;
    std::atomic<bool> dirty;
    
    // Data from BrainComm
    Position2D left_gps;
    Position2D right_gps;
    Position2D sister_position;
    
    TransmitStats tx_stats;
    ReceiveStats rx_stats;
    
    uint16_t current_request;
    uint16_t current_response;
    bool connected;
    bool transmitting;
    
    std::vector<ErrorLog> recent_errors;
};

} // namespace Brain

#endif // BRAIN_COMM_MONITOR_H