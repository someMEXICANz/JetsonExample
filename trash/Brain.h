#ifndef BRAIN_H
#define BRAIN_H

#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include "BrainPacket.h"

class Brain {
public:
    Brain(boost::asio::io_service& io_service, const std::string& port);
    ~Brain();

    bool start();
    void stop();
    bool restart();
    bool reconnect();
    
    void sendRequest(uint16_t requestFlags);

    // Getters for monitoring
    bool isConnected() const { return connected_; }
    float getLeftVoltage() const { return left_voltage_; }
    float getRightVoltage() const { return right_voltage_; }

private:
    // Serial port handling
    boost::asio::io_service& ioService;
    std::string port_;
    std::unique_ptr<boost::asio::serial_port> serialPort;
    
    // Thread management
    bool running_;
    bool connected_;
    std::unique_ptr<std::thread> read_thread_;
    
    // Active request tracking
    uint16_t active_brain_request_;
    
    // Data storage
    float left_voltage_;
    float right_voltage_;
    uint16_t macroState_;
    
    // Constants
    static constexpr auto RECONNECT_DELAY = std::chrono::milliseconds(1000);
    static constexpr auto READ_DELAY = std::chrono::milliseconds(20);
    
    // Core functions
    bool initializePort();
    void readLoop();
    void processBrainRequest(const std::vector<uint8_t>& buffer);
    void parseBrainResponse(const std::vector<uint8_t>& buffer);
    
};

#endif // BRAIN_H