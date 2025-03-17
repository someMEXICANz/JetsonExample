#include "CommsMonitor.h"
#include <sstream>
#include <iomanip>

namespace Brain {

CommsMonitor::CommsMonitor(const std::string& title)
    : window_title(title)
    , display_buffer(cv::Mat(800, 1000, CV_8UC3, cv::Scalar(30, 30, 30)))
    , is_running(false)
    , dirty(true)
{
}

CommsMonitor::~CommsMonitor() {
    stop();
}

void CommsMonitor::start() {
    if (is_running) return;
    
    is_running = true;
    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, 1000, 800);
    
    // Start update thread
    update_thread = std::thread(&CommsMonitor::updateLoop, this);
}

void CommsMonitor::stop() {
    is_running = false;
    
    if (update_thread.joinable()) {
        update_thread.join();
    }
    
    cv::destroyWindow(window_title);
}

// In BrainCommMonitor.cpp

void CommsMonitor::updateData(const BrainComm& comm) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Update GPS positions
    left_gps = comm.getLeftGPSData();
    right_gps = comm.getRightGPSData();
    sister_position = comm.getSisterPosition();
    
    // Copy stats manually (fix for the atomic member issue)
    copyTransmitStats(tx_stats, comm.getStats().getTransmitStats());
    copyReceiveStats(rx_stats, comm.getStats().getReceiveStats());
    
    // Get flags and connection status
    current_request = comm.getCurrentRequests();
    current_response = comm.getCurrentResponse();
    connected = comm.isConnected();
    transmitting = comm.isTransmitting();
    
    // Get recent errors
    recent_errors = comm.getStats().getRecentErrors(5); // Get last 5 errors
    
    // Mark display as dirty
    dirty = true;
}

// Add these helper methods to BrainCommMonitor class
void CommsMonitor::copyTransmitStats(TransmitStats& dest, const TransmitStats& src) {
    dest.total_transmissions = src.total_transmissions.load();
    dest.failed_transmissions = src.failed_transmissions.load();
    dest.missed_deadlines = src.missed_deadlines.load();
    dest.write_buffer_overruns = src.write_buffer_overruns.load();
    dest.max_transmission_delay = src.max_transmission_delay;
    dest.avg_transmission_delay = src.avg_transmission_delay;
}

void CommsMonitor::copyReceiveStats(ReceiveStats& dest, const ReceiveStats& src) {
    dest.total_reads = src.total_reads.load();
    dest.failed_reads = src.failed_reads.load();
    dest.invalid_requests = src.invalid_requests.load();
    dest.lost_connections = src.lost_connections.load();
    dest.reestablished_connections = src.reestablished_connections.load();
}

void CommsMonitor::updateLoop() {
    while (is_running) {
        // Only redraw if data has changed
        if (dirty) {
            renderDisplay();
            dirty = false;
        }
        
        // Show the display buffer
        cv::imshow(window_title, display_buffer);
        
        // Process UI events and wait for a short time
        int key = cv::waitKey(50);
        if (key == 27) { // ESC key
            is_running = false;
        }
    }
}

void CommsMonitor::renderDisplay() {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Clear the display
    display_buffer = cv::Scalar(30, 30, 30);
    
    // Draw title
    cv::putText(display_buffer, "BrainComm Monitoring Dashboard", 
                cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                cv::Scalar(200, 200, 200), 2);
    
    // Draw connection status
    std::string status = connected ? "CONNECTED" : "DISCONNECTED";
    cv::Scalar status_color = connected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(display_buffer, "Status: " + status, 
                cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                status_color, 2);
    
    // Draw request/response flags
    std::stringstream flags_ss;
    flags_ss << "Request: 0x" << std::hex << std::setw(4) << std::setfill('0') << current_request 
             << " Response: 0x" << std::hex << std::setw(4) << std::setfill('0') << current_response;
    cv::putText(display_buffer, flags_ss.str(), 
                cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(200, 200, 200), 1);
    
    // Draw GPS section
    drawSection("GPS Positions", 160);
    drawGPSData("Left GPS", left_gps, 200);
    drawGPSData("Right GPS", right_gps, 240);
    drawGPSData("Sister", sister_position, 280);
    
    // Draw statistics section
    drawSection("Communication Statistics", 340);
    
    // Transmit stats
    drawStats("Transmit Total:", tx_stats.total_transmissions, 380);
    drawStats("Transmit Errors:", tx_stats.failed_transmissions, 410);
    drawStats("Avg Delay:", tx_stats.avg_transmission_delay.count(), "µs", 440);
    
    // Receive stats
    drawStats("Receive Total:", rx_stats.total_reads, 480);
    drawStats("Receive Errors:", rx_stats.failed_reads, 510);
    drawStats("Lost Connections:", rx_stats.lost_connections, 540);
    
    // Draw recent errors section
    drawSection("Recent Errors", 600);
    int y_pos = 640;
    for (const auto& error : recent_errors) {
        std::string error_string = errorTypeToString(error.error_type) + ": " + error.description;
        // Truncate if too long
        if (error_string.length() > 60) {
            error_string = error_string.substr(0, 57) + "...";
        }
        cv::putText(display_buffer, error_string, 
                    cv::Point(30, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                    cv::Scalar(255, 150, 150), 1);
        y_pos += 30;
    }
    
    // Draw visualization of connection state
    drawConnectionVisualization(800, 300);
}

void CommsMonitor::drawSection(const std::string& title, int y_pos) {
    // Draw section title
    cv::putText(display_buffer, title, 
                cv::Point(20, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                cv::Scalar(200, 200, 120), 1);
    
    // Draw underline
    cv::line(display_buffer, cv::Point(20, y_pos + 5), cv::Point(500, y_pos + 5), 
             cv::Scalar(200, 200, 120), 1);
}

void CommsMonitor::drawGPSData(const std::string& label, const Position2D& pos, int y_pos) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2)
       << label << ": X=" << pos.x << " Y=" << pos.y << " Heading=" << pos.heading;
    
    cv::putText(display_buffer, ss.str(), 
                cv::Point(30, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(180, 180, 220), 1);
}

void CommsMonitor::drawStats(const std::string& label, size_t value, int y_pos) {
    std::stringstream ss;
    ss << label << " " << value;
    
    cv::putText(display_buffer, ss.str(), 
                cv::Point(30, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(180, 220, 180), 1);
}

void CommsMonitor::drawStats(const std::string& label, long long value, const std::string& unit, int y_pos) {
    std::stringstream ss;
    ss << label << " " << value << " " << unit;
    
    cv::putText(display_buffer, ss.str(), 
                cv::Point(30, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(180, 220, 180), 1);
}

void CommsMonitor::drawConnectionVisualization(int x_pos, int y_pos) {
    // Draw connection visualization rectangle
    cv::Scalar color = connected ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 200);
    if (connected && transmitting) {
        // Blinking effect when transmitting
        static int blink_counter = 0;
        blink_counter = (blink_counter + 1) % 10;
        if (blink_counter < 5) {
            color = cv::Scalar(0, 255, 255);  // Yellow-green when actively transmitting
        }
    }
    
    cv::rectangle(display_buffer, cv::Point(x_pos, y_pos), cv::Point(x_pos + 150, y_pos + 80), 
                  color, -1);  // Filled rectangle
    
    // Draw connection text
    std::string conn_text = connected ? (transmitting ? "ACTIVE" : "IDLE") : "DISCONNECTED";
    cv::putText(display_buffer, conn_text, 
                cv::Point(x_pos + 10, y_pos + 45), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(0, 0, 0), 2);
}

std::string CommsMonitor::errorTypeToString(CommError error) {
    switch (error) {
        case CommError::None: return "None";
        case CommError::ConnectionLost: return "Connection Lost";
        case CommError::WriteTimeout: return "Write Timeout";
        case CommError::ReadTimeout: return "Read Timeout";
        case CommError::InvalidRequest: return "Invalid Request";
        case CommError::InvalidPacket: return "Invalid Packet";
        case CommError::BufferOverrun: return "Buffer Overrun";
        case CommError::TransmissionFailed: return "Transmission Failed";
        case CommError::AcknowledgmentFailed: return "Acknowledgment Failed";
        case CommError::DeadlineMissed: return "Deadline Missed";
        case CommError::RequestTimeout: return "Request Timeout";
        default: return "Unknown";
    }
}

} // namespace Brain