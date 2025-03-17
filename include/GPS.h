#ifndef GPS_H
#define GPS_H

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/circular_buffer.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <optional>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <Position.h>





class GPS 
{
public:
    explicit GPS(boost::asio::io_service& io_service, 
                const std::string& port = "",
                size_t history_size = 100);
    ~GPS();

    // Delete copy constructor and assignment operator
    GPS(const GPS&) = delete;
    GPS& operator=(const GPS&) = delete;

    // Core operations
    bool start();
    void stop();
    bool restart();

    // Status checks
    bool isConnected() const { return connected_; }
    bool isRunning() const { return running_; }
    

    // Position retrieval
    Position getPosition();
    Position getLastValidPosition();
    std::vector<Position> getPositionHistory();

    // Configuration methods
    void setHistorySize(size_t size) { position_history_.set_capacity(size); }
    void setPort(const std::string& port) { port_ = port; }
    const std::string& getPort() const { return port_; }

private:
    // Internal methods
    void readLoop();
    bool initializePort();
    bool validatePosition(const Position& pos);
    void processBuffer(const std::vector<unsigned char>& buffer);
    bool reconnect();

    std::string port_;
    boost::asio::io_service& ioService;
    std::unique_ptr<boost::asio::serial_port> serialPort;
    std::mutex position_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::unique_ptr<std::thread> read_thread_;  

    uint32_t status_;

    Position position_;
    boost::circular_buffer<Position> position_history_;
    
    // Statistics
    size_t total_reads_{0};
    size_t failed_reads_{0};
    size_t invalid_positions_{0};

    const std::chrono::milliseconds READ_TIMEOUT{500};
    const std::chrono::milliseconds RECONNECT_DELAY{2500} ;
    const float MAX_POSITION_JUMP = 0.25f;
    const float MAX_ANGLE_JUMP = 30.0f;


};




#endif