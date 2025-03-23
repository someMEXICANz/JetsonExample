#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <deque>
#include "GPS.h"
#include "IMU.h"
#include "BrainComm.h"
#include "Position.h"

class RobotPosition {
public:
    // Constructor and initialization
    RobotPosition(Brain::BrainComm& brain, IMU& imu);
    ~RobotPosition();
    bool initialize(boost::asio::io_service& io_service);
    
    // Thread control
    bool startUpdateThread(int update_frequency_hz = 50);
    void stopUpdateThread();
    bool isThreadRunning() const { return update_thread_running_; }
    
    // Manual update (if not using thread)
    void update();
    
    // Position access methods
    Position getPosition() const;
    void get2DPosition(float& x, float& y, float& heading) const;
    float getHeading() const;
    bool hasReliablePosition() const;  // Returns true if confidence > threshold
    
    // Motion data access
    float getVelocity() const;
    float getAngularVelocity() const;
    
    // Calibration and diagnostics
    void calibrateIMUHeading();
    bool areGPSIdentified() const { return gps_identified_; }
    
private:
    // References to external components
    Brain::BrainComm& brain_;
    IMU& imu_;
    
    // GPS sensors managed by this class
    std::unique_ptr<GPS> gps1_;
    std::unique_ptr<GPS> gps2_;
    
    // Pointers to identified sensors (not owned)
    GPS* left_gps_ = nullptr;
    GPS* right_gps_ = nullptr;
    
    // Current position state
    Position robot_position_;
    float position_confidence_ = 0.0f;

    // GPS offsets from Brain
    Brain::Position2D left_gps_offset_;
    Brain::Position2D right_gps_offset_;
    bool offsets_received_ = false;
    
    // Heading calibration
    float heading_offset_ = 0.0f;
    std::chrono::system_clock::time_point last_calibration_time_;  // Changed to system_clock
    
    // Thread safety
    mutable std::mutex position_mutex_;
    
    // Update thread
    std::thread update_thread_;
    std::atomic<bool> update_thread_running_{false};
    
    // Sensor state
    bool gps_identified_ = false;
    
    // Position filtering and processing
    std::deque<Position> position_history_;
    const size_t position_history_max_size_ = 5;
    
    // Velocity calculation
    Position last_position_;
    std::chrono::system_clock::time_point last_velocity_update_;  // Changed to system_clock
    float current_velocity_ = 0.0f;
    float current_angular_velocity_ = 0.0f;
    
    // Position validation parameters
    const float MAX_POSITION_JUMP = 0.25f;
    const float MAX_ANGLE_JUMP = 30.0f;
    const float MAX_COORDINATE = 100.0f;
    const float MIN_CONFIDENCE_THRESHOLD = 0.3f;  // Minimum confidence for reliable position
    
    // Helper methods
    bool identifyGPSSensors();
    float getHeadingFromIMU() const;
    bool isRobotStationary() const;
    Position averagePositions(const std::vector<Position>& positions);
    void filterPosition(Position& position);
    bool detectPositionJump(const Position& current, const Position& previous) const;
    void updateVelocity(const Position& current_position);
    float calculateGPSQuality(uint32_t status) const;
    bool isPositionValid(const Position& position) const;
    
    // Thread function
    void updateThreadFunction(int update_frequency_hz);
};

#endif // ROBOT_POSITION_H