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
#include <condition_variable>

namespace Brain {

class BrainComm {
public:
    explicit BrainComm(boost::asio::io_service& service, const std::string& usb_port = "");
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

    // Data setting
    void setMotorVoltages(float left, float right);
    void setMacroBits(uint32_t macro_bits);
    void setJetsonBattery(uint32_t level);

    // Data retrieval
    Position2D getLeftGPSData() const;
    Position2D getRightGPSData() const;
    Position2D getSisterPosition() const;
    Position2D getLeftGPSOffset() const;
    Position2D getRightGPSOffset() const;
    uint32_t getBrainBattery() const;

    // Statistics and error handling
    const StatsManager& getStats() const { return stats; }
    uint16_t getCurrentResponse() const { return response_flags; }
    uint16_t getCurrentRequests() const { return request_flags; }
    bool isTransmitting() const { return sendData; }

    // Status checks
    bool isConnected() const { return connected; }
    bool isRunning() const { return running; }
   
private:
    // Thread functions
    void readLoop();
    void writeLoop();
    
    // Internal methods
    bool initializePort();
    bool reconnect();
    bool processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length);
    
    // Message handling methods
    bool sendResponse(uint16_t flags);
    bool sendAcknowledgment(uint16_t flags, uint8_t status);
    bool sendRequests(uint16_t flags);
    bool handleMessage(const uint8_t* buffer, size_t length);
    
    // Timeout checking
    void checkRequestTimeout();

    // Serial Port variables
    std::string port;
    boost::asio::io_service& io_service;
    std::unique_ptr<boost::asio::serial_port> serial_port;
    std::atomic<bool> connected{false};
    
    // Thread management
    std::unique_ptr<std::thread> read_thread;
    std::unique_ptr<std::thread> write_thread;

    // Synchronization
    std::condition_variable write_condition;
    
    // State variables
    std::atomic<bool> running{false};
    
    // Mutexes for thread safety
    mutable std::mutex data_mutex;
    std::mutex state_mutex;
    std::mutex write_mutex;

    // Request management
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
    
    // Buffer for reading
    // std::vector<uint8_t> read_buffer;
    // size_t buffer_index;
    
    // Storage for Data From Brain
    Position2D left_gps_position;
    Position2D left_gps_offset;
    Position2D right_gps_position;
    Position2D right_gps_offset;
    Position2D sister_position;
    uint32_t BrainBatteryLvl;

    // Data to send to Brain
    MotorCommand current_motor_command;
    ControlFlags current_control_flags;
    uint32_t current_battery_lvl;
    
    // Statistics and error tracking
    StatsManager stats;
};

} // namespace Brain

#endif































