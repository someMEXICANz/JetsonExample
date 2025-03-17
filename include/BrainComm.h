#ifndef BRAIN_COMM_H
#define BRAIN_COMM_H

#include "BrainCommTypes.h"
#include "BrainCommStats.h"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <queue>


namespace Brain {

class BrainComm {
public:
    explicit BrainComm(boost::asio::io_service& service, 
                      const std::string& usb_port = "");
    ~BrainComm();

    // Delete copy constructor and assignment operator
    BrainComm(const BrainComm&) = delete;
    BrainComm& operator=(const BrainComm&) = delete;

    // Core operations
    bool start();
    void stop();
    bool restart();

    // Request management
    bool updateRequests(uint16_t flags);

    // Data retrieval
    Position2D getLeftGPSData() const;
    Position2D getRightGPSData() const;
    Position2D getSisterPosition() const;
    Position2D getLeftGPSOffset() const;
    Position2D getRightGPSOffset() const;
    uint32_t getBrainBattery() const;

    // Motor control
    void setMotorVoltages(float left, float right);
    void setMacroBits(uint32_t macro_bits);
    void setJetsonBattery(uint32_t level);

    // Statistics and error handling
    const StatsManager& getStats() const { return stats; }
    uint16_t getCurrentResponse() const { return response_flags; }
    uint16_t getCurrentRequests() const { return request_flags; }
    bool isTransmitting() const { return sendData; }

    // Status checks
    bool isConnected() const { return connected; }
    bool isRunning() const { return running; }
   

private:
    // Internal methods
    bool initializePort();
    bool reconnect();
    bool processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length);
    bool sendResponse(uint16_t flags);
    bool sendAcknowledgment(uint16_t flags, uint8_t status);
    
    // Asynchronous operation handlers
    void startAsyncRead();
    void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred);
    bool startAsyncWrite(uint16_t flags, MessageType msg_type);
    void handleWrite(const boost::system::error_code& ec, std::size_t bytes_transferred, bool is_request);
    void checkPendingRequests();
    void handleTimer(const boost::system::error_code& ec);
    void checkRequestTimeout();

    // Serial Port variables
    std::string port;
    boost::asio::io_service& io_service;
    std::unique_ptr<boost::asio::serial_port> serial_port;
    
    // Thread management
    std::unique_ptr<std::thread> io_thread;
    
    // State variables
    std::atomic<bool> running{false};
    std::atomic<bool> connected{false};
    static std::atomic<bool> read_in_progress;

    // Mutexes for thread safety
    mutable std::mutex data_mutex;
    std::mutex state_mutex;

    std::queue<uint16_t> pending_requests;

    std::atomic<bool> request_in_progress{false};
    std::atomic<uint32_t> last_request_time;
    std::atomic<uint8_t> request_retry_count;
    
    // Communication state
    std::atomic<uint16_t> request_flags;                // Current Jetson request flags
    std::atomic<uint16_t> response_flags; 
    std::atomic<bool> sendData{false};
    std::atomic<uint32_t> last_send_time;              // Timestamp of last data send
    std::atomic<uint32_t> last_received_time;          // Timestamp of last received data
    
    // Async operation variables 
    boost::asio::deadline_timer timer;
    std::vector<uint8_t> read_buffer;
    size_t buffer_index;
    
    // Storage for Data From Brain
    Position2D left_gps_position;
    Position2D left_gps_offset;
    Position2D right_gps_position;
    Position2D right_gps_offset;
    Position2D sister_position;
    uint32_t brain_battery;

    MotorCommand current_motor_command;
    ControlFlags current_control_flags;
    uint32_t current_battery_lvl;
    
    // Statistics and error tracking
    StatsManager stats;
};

} // namespace Brain

#endif





















