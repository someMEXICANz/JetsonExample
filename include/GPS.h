#ifndef GPS_H
#define GPS_H

#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <iostream>
#include "Position.h"

class GPS 
{
public:
    // Status bit definitions
    static constexpr uint32_t STATUS_CONNECTED    = 0x00000001;
    static constexpr uint32_t STATUS_NODOTS       = 0x00000002;
    static constexpr uint32_t STATUS_NORAWBITS    = 0x00000004;
    static constexpr uint32_t STATUS_NOGROUPS     = 0x00000008;
    static constexpr uint32_t STATUS_NOBITS       = 0x00000010;
    static constexpr uint32_t STATUS_PIXELERROR   = 0x00000020;
    static constexpr uint32_t STATUS_SOLVER       = 0x00000040;
    static constexpr uint32_t STATUS_ANGLEJUMP    = 0x00000080;
    static constexpr uint32_t STATUS_POSJUMP      = 0x00000100;
    static constexpr uint32_t STATUS_NOSOLUTION   = 0x00000200;
    static constexpr uint32_t STATUS_KALMAN_EST   = 0x00100000;

    explicit GPS(boost::asio::io_service& io_service, 
                const std::string& port = "");
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
    
    // Raw position retrieval - no filtering or validation
    Position getRawPosition() const;
    uint32_t getStatus() const { return status_; }

    // Configuration methods
    void setPort(const std::string& port) { port_ = port; }
    const std::string& getPort() const { return port_; }

private:
    // Internal methods
    void readLoop();
    bool initializePort();
    void processBuffer(const std::vector<unsigned char>& buffer);
    bool reconnect();

    // Hardware connection
    std::string port_;
    boost::asio::io_service& ioService;
    std::unique_ptr<boost::asio::serial_port> serialPort;
    
    // Thread safety and state
    mutable std::mutex position_mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::unique_ptr<std::thread> read_thread_;  

    // Position and status data
    uint32_t status_;
    Position position_;
    
    // Configuration constants
    const std::chrono::milliseconds READ_TIMEOUT{500};
    const std::chrono::milliseconds RECONNECT_DELAY{2500};
};

#endif // GPS_H