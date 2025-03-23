#include "RobotPosition.h"
#include "PortDetector.h"

// In RobotPosition.cpp - Initialize offsets in constructor
RobotPosition::RobotPosition(Brain::BrainComm& brain, IMU& imu)
    : brain_(brain),
      imu_(imu),
      gps_identified_(false),
      offsets_received_(false),
      heading_offset_(0.0f),
      update_thread_running_(false),
      position_confidence_(0.0f),
      current_velocity_(0.0f),
      current_angular_velocity_(0.0f)
{
    // Initialize position data
    robot_position_ = Position();
    last_position_ = Position();
    
    // Initialize GPS offsets
    left_gps_offset_ = {0.0f, 0.0f, 0.0f};
    right_gps_offset_ = {0.0f, 0.0f, 0.0f};
    
    // Initialize timestamps with system_clock
    last_calibration_time_ = std::chrono::system_clock::now();
    last_velocity_update_ = std::chrono::system_clock::now();
}

RobotPosition::~RobotPosition() {
    // Stop update thread if running
    stopUpdateThread();
    
    // Stop GPS sensors
    if (gps1_) gps1_->stop();
    if (gps2_) gps2_->stop();
}

bool RobotPosition::initialize(boost::asio::io_service& io_service) {
    // Find available GPS ports
    auto gps_ports = PortDetector::findGPSPorts();
    if (gps_ports.size() < 2) {
        std::cerr << "Error: Not enough GPS devices found!" << std::endl;
        return false;
    }
    
    // Initialize GPS sensors
    std::cout << "Initializing GPS1 on port: " << gps_ports[0] << std::endl;
    gps1_ = std::make_unique<GPS>(io_service, gps_ports[0]);
    if (!gps1_->start()) {
        std::cerr << "Failed to start GPS1" << std::endl;
        return false;
    }
    
    std::cout << "Initializing GPS2 on port: " << gps_ports[1] << std::endl;
    gps2_ = std::make_unique<GPS>(io_service, gps_ports[1]);
    if (!gps2_->start()) {
        std::cerr << "Failed to start GPS2" << std::endl;
        return false;
    }
    
    // Wait for GPS to get initial readings
    std::cout << "Waiting for GPS readings..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Identify which GPS is which
    if (!identifyGPSSensors()) {
        std::cerr << "Warning: Could not identify GPS sensors. Using default assignment." << std::endl;
        left_gps_ = gps1_.get();
        right_gps_ = gps2_.get();
    }
    
    // Initialize position data with first readings
    update();
    
    return true;
}

// In RobotPosition.cpp - Update identifyGPSSensors to retrieve offsets
bool RobotPosition::identifyGPSSensors() {
    // Request GPS data from Brain (both left and right)
    brain_.updateRequests(static_cast<uint16_t>(Brain::RequestFlag::LeftGPSData) | static_cast<uint16_t>(Brain::RequestFlag::RightGPSData));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Get current offsets from Brain
    left_gps_offset_ = brain_.getLeftGPSOffset();
    right_gps_offset_ = brain_.getRightGPSOffset();
    offsets_received_ = true;
    
    std::cout << "Received GPS offsets from Brain:" << std::endl;
    std::cout << "Left GPS offset: (" << left_gps_offset_.x << ", " 
              << left_gps_offset_.y << ")" << std::endl;
    std::cout << "Right GPS offset: (" << right_gps_offset_.x << ", " 
              << right_gps_offset_.y << ")" << std::endl;
    
    // Try multiple samples for more reliable identification
    const int NUM_SAMPLES = 10;
    std::vector<Position> gps1_samples;
    std::vector<Position> gps2_samples;
    std::vector<Brain::Position2D> brain_left_samples;
    std::vector<Brain::Position2D> brain_right_samples;
    
    std::cout << "Collecting position samples for GPS identification..." << std::endl;
    
    // Collect samples
    for (int i = 0; i < NUM_SAMPLES; i++) {
        gps1_samples.push_back(gps1_->getRawPosition());
        gps2_samples.push_back(gps2_->getRawPosition());
        brain_left_samples.push_back(brain_.getLeftGPSData());
        brain_right_samples.push_back(brain_.getRightGPSData());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Calculate average positions
    Position avg_gps1 = averagePositions(gps1_samples);
    Position avg_gps2 = averagePositions(gps2_samples);
    
    // Calculate average brain positions
    Brain::Position2D avg_brain_left;
    avg_brain_left.x = 0;
    avg_brain_left.y = 0;
    avg_brain_left.heading = 0;
    
    for (const auto& pos : brain_left_samples) {
        avg_brain_left.x += pos.x;
        avg_brain_left.y += pos.y;
        avg_brain_left.heading += pos.heading;
    }
    avg_brain_left.x /= brain_left_samples.size();
    avg_brain_left.y /= brain_left_samples.size();
    avg_brain_left.heading /= brain_left_samples.size();
    
    // Calculate raw positions (remove offsets from brain positions)
    float brain_left_raw_x = avg_brain_left.x - left_gps_offset_.x;
    float brain_left_raw_y = avg_brain_left.y - left_gps_offset_.y;
    
    // Calculate distances to identify sensors
    float dist1_to_brain_left = std::sqrt(
        std::pow(avg_gps1.x - brain_left_raw_x, 2) + 
        std::pow(avg_gps1.y - brain_left_raw_y, 2));
    
    float dist2_to_brain_left = std::sqrt(
        std::pow(avg_gps2.x - brain_left_raw_x, 2) + 
        std::pow(avg_gps2.y - brain_left_raw_y, 2));
    
    std::cout << "Distance from GPS1 to inferred left position: " << dist1_to_brain_left << std::endl;
    std::cout << "Distance from GPS2 to inferred left position: " << dist2_to_brain_left << std::endl;
    
    // Determine which GPS is which based on closest match
    if (dist1_to_brain_left < dist2_to_brain_left) {
        std::cout << "GPS1 identified as LEFT sensor" << std::endl;
        left_gps_ = gps1_.get();
        right_gps_ = gps2_.get();
    } else {
        std::cout << "GPS2 identified as LEFT sensor" << std::endl;
        left_gps_ = gps2_.get();
        right_gps_ = gps1_.get();
    }
    
    gps_identified_ = true;
    return true;
}

Position RobotPosition::averagePositions(const std::vector<Position>& positions) {
    if (positions.empty()) {
        return Position();
    }
    
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float sin_sum_azimuth = 0.0f, cos_sum_azimuth = 0.0f;
    float sin_sum_elevation = 0.0f, cos_sum_elevation = 0.0f;
    float sin_sum_rotation = 0.0f, cos_sum_rotation = 0.0f;
    
    for (const auto& pos : positions) {
        sum_x += pos.x;
        sum_y += pos.y;
        sum_z += pos.z;
        
        // For angular values, use sine/cosine averaging
        float az_rad = pos.azimuth * M_PI / 180.0f;
        float el_rad = pos.elevation * M_PI / 180.0f;
        float rot_rad = pos.rotation * M_PI / 180.0f;
        
        sin_sum_azimuth += std::sin(az_rad);
        cos_sum_azimuth += std::cos(az_rad);
        sin_sum_elevation += std::sin(el_rad);
        cos_sum_elevation += std::cos(el_rad);
        sin_sum_rotation += std::sin(rot_rad);
        cos_sum_rotation += std::cos(rot_rad);
    }
    
    size_t count = positions.size();
    
    // Calculate average angles
    float avg_azimuth = std::atan2(sin_sum_azimuth, cos_sum_azimuth) * 180.0f / M_PI;
    if (avg_azimuth < 0) avg_azimuth += 360.0f;
    
    float avg_elevation = std::atan2(sin_sum_elevation, cos_sum_elevation) * 180.0f / M_PI;
    float avg_rotation = std::atan2(sin_sum_rotation, cos_sum_rotation) * 180.0f / M_PI;
    
    return Position(
        sum_x / count,
        sum_y / count,
        sum_z / count,
        avg_azimuth,
        avg_elevation,
        avg_rotation
    );
}

float RobotPosition::calculateGPSQuality(uint32_t status) const {
    // Start with maximum quality
    float quality = 1.0f;
    
    // Critical errors - position is unreliable
    if (!(status & GPS::STATUS_CONNECTED) || (status & GPS::STATUS_NOSOLUTION)) {
        return 0.1f;  // Very low quality
    }
    
    // Major data acquisition issues
    if ((status & GPS::STATUS_NODOTS) || (status & GPS::STATUS_NOBITS)) {
        quality *= 0.3f;
    }
    
    // Data processing issues
    if ((status & GPS::STATUS_NORAWBITS) || (status & GPS::STATUS_NOGROUPS) || 
        (status & GPS::STATUS_PIXELERROR)) {
        quality *= 0.5f;
    }
    
    // Position estimation or jumps
    if ((status & GPS::STATUS_ANGLEJUMP) || (status & GPS::STATUS_POSJUMP)) {
        quality *= 0.7f;
    }
    
    // Minor issues
    if ((status & GPS::STATUS_SOLVER) || (status & GPS::STATUS_KALMAN_EST)) {
        quality *= 0.9f;
    }
    
    return quality;
}

bool RobotPosition::isPositionValid(const Position& position) const {
    // Check if coordinates are within reasonable bounds
    if (std::abs(position.x) > MAX_COORDINATE || 
        std::abs(position.y) > MAX_COORDINATE || 
        std::abs(position.z) > MAX_COORDINATE) {
        return false;
    }
    
    // Check if orientation angles are within expected ranges
    if (position.azimuth < 0.0f || position.azimuth >= 360.0f ||
        std::abs(position.elevation) > 90.0f || 
        std::abs(position.rotation) > 180.0f) {
        return false;
    }
    
    // Check for sudden jumps if we have position history
    if (!position_history_.empty()) {
        const Position& prev = position_history_.back();
        
        if (detectPositionJump(position, prev)) {
            return false;
        }
    }
    
    return true;
}

bool RobotPosition::detectPositionJump(const Position& current, const Position& previous) const {
    // Check for position jumps
    float distance = current.distanceTo2D(previous);
    if (distance > MAX_POSITION_JUMP) {
        return true;
    }
    
    // Check for angle jumps
    float heading_diff = std::abs(current.azimuth - previous.azimuth);
    if (heading_diff > 180.0f) {
        heading_diff = 360.0f - heading_diff;
    }
    
    if (heading_diff > MAX_ANGLE_JUMP) {
        return true;
    }
    
    return false;
}

// In RobotPosition.cpp - Update the update method to use the offsets
void RobotPosition::update() {
    // Check if we have received offsets yet
    if (!offsets_received_) {
        // Request GPS offsets if not already received
        brain_.updateRequests(static_cast<uint16_t>(Brain::RequestFlag::LeftGPSData) | static_cast<uint16_t>(Brain::RequestFlag::RightGPSData));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Get current offsets from Brain
        left_gps_offset_ = brain_.getLeftGPSOffset();
        right_gps_offset_ = brain_.getRightGPSOffset();
        offsets_received_ = true;
        
        std::cout << "Retrieved GPS offsets from Brain:" << std::endl;
        std::cout << "Left GPS offset: (" << left_gps_offset_.x << ", " 
                  << left_gps_offset_.y << ")" << std::endl;
        std::cout << "Right GPS offset: (" << right_gps_offset_.x << ", " 
                  << right_gps_offset_.y << ")" << std::endl;
    }

    // Get raw position data from GPS sensors
    Position left_pos = left_gps_->getRawPosition();
    Position right_pos = right_gps_->getRawPosition();
    
    // Get GPS status and calculate quality
    uint32_t left_status = left_gps_->getStatus();
    uint32_t right_status = right_gps_->getStatus();
    float left_quality = calculateGPSQuality(left_status);
    float right_quality = calculateGPSQuality(right_status);
    
    // Get brain-reported positions (already have offsets applied)
    Brain::Position2D brain_left = brain_.getLeftGPSData();
    Brain::Position2D brain_right = brain_.getRightGPSData();
    
    // Create a new position estimate
    Position new_position;
    
    // Calculate weighted position based on quality
    float total_quality = left_quality + right_quality;
    
    if (total_quality > 0.1f) {
        // At least one GPS has some usable data
        float left_weight = left_quality / total_quality;
        float right_weight = right_quality / total_quality;
        
        // Use brain-reported positions (with offsets already applied)
        // This is more accurate than applying offsets ourselves
        new_position.x = brain_left.x * left_weight + brain_right.x * right_weight;
        new_position.y = brain_left.y * left_weight + brain_right.y * right_weight;
        
        // Z coordinate typically less important for 2D navigation
        new_position.z = (left_pos.z * left_weight + right_pos.z * right_weight);
        
        // Use IMU for heading (azimuth)
        new_position.azimuth = getHeadingFromIMU();
        
        // For elevation and rotation, use weighted average from GPS
        new_position.elevation = left_pos.elevation * left_weight + right_pos.elevation * right_weight;
        new_position.rotation = left_pos.rotation * left_weight + right_pos.rotation * right_weight;
        
        // Set confidence based on GPS quality
        new_position.confidence = total_quality / 2.0f;  // Average quality
    } else {
        // No reliable GPS data - use last known position
        std::lock_guard<std::mutex> lock(position_mutex_);
        new_position = robot_position_;
        
        // Still update heading from IMU
        new_position.azimuth = getHeadingFromIMU();
        
        // Decay confidence over time
        new_position.confidence *= 0.8f;
    }
    
    // Set timestamp
    new_position.timestamp = std::chrono::system_clock::now();
    
    // Apply filtering
    filterPosition(new_position);
    
    // Update velocity estimation
    updateVelocity(new_position);
    
    // Update stored position
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        robot_position_ = new_position;
        position_confidence_ = new_position.confidence;
    }
}


void RobotPosition::filterPosition(Position& position) {
    // Add to history
    position_history_.push_back(position);
    if (position_history_.size() > position_history_max_size_) {
        position_history_.pop_front();
    }
    
    // Need at least 3 points for filtering
    if (position_history_.size() < 3) {
        return;
    }
    
    // Apply weighted average filter
    float x_sum = 0.0f, y_sum = 0.0f, z_sum = 0.0f;
    float sin_sum = 0.0f, cos_sum = 0.0f;
    float weight_sum = 0.0f;
    
    for (size_t i = 0; i < position_history_.size(); i++) {
        // More weight to newer readings and higher confidence
        float age_factor = 0.6f + 0.4f * i / (position_history_.size() - 1);
        float weight = age_factor * position_history_[i].confidence;
        
        x_sum += position_history_[i].x * weight;
        y_sum += position_history_[i].y * weight;
        z_sum += position_history_[i].z * weight;
        
        float heading_rad = position_history_[i].azimuth * M_PI / 180.0f;
        sin_sum += std::sin(heading_rad) * weight;
        cos_sum += std::cos(heading_rad) * weight;
        
        weight_sum += weight;
    }
    
    // Apply filtered values if we have valid weights
    if (weight_sum > 0.001f) {
        position.x = x_sum / weight_sum;
        position.y = y_sum / weight_sum;
        position.z = z_sum / weight_sum;
        
        // Process heading
        float heading = std::atan2(sin_sum, cos_sum) * 180.0f / M_PI;
        if (heading < 0) heading += 360.0f;
        
        // Only update heading from filter if it's not too different from IMU
        float heading_diff = std::abs(heading - position.azimuth);
        if (heading_diff > 180.0f) {
            heading_diff = 360.0f - heading_diff;
        }
        
        // If filter heading is close to IMU heading, use it (smooths small variations)
        if (heading_diff < 15.0f) {
            position.azimuth = heading;
        }
    }
}

void RobotPosition::updateVelocity(const Position& current_position) {
    auto now = std::chrono::system_clock::now();  // Changed to system_clock
    
    // Calculate time difference in seconds
    float dt = std::chrono::duration<float>(now - last_velocity_update_).count();
    
    if (dt > 0.001f) {  // Avoid division by very small numbers
        // Calculate linear velocity (distance/time)
        float distance = last_position_.distanceTo2D(current_position);
        current_velocity_ = distance / dt;
        
        // Calculate angular velocity (heading change/time)
        float heading_diff = current_position.azimuth - last_position_.azimuth;
        // Normalize heading difference to -180 to 180
        while (heading_diff > 180) heading_diff -= 360;
        while (heading_diff < -180) heading_diff += 360;
        current_angular_velocity_ = heading_diff / dt;
        
        // Update last position and time
        last_position_ = current_position;
        last_velocity_update_ = now;
    }
}

float RobotPosition::getHeadingFromIMU() const {
    float mx, my, mz;
    if (imu_.readMagnetometer(mx, my, mz)) {
        // Calculate heading from magnetometer
        float heading = atan2(my, mx) * 180.0f / M_PI;
        
        // Normalize to 0-360
        while (heading < 0) heading += 360.0f;
        while (heading >= 360) heading -= 360.0f;
        
        // Apply calibration offset
        heading += heading_offset_;
        while (heading < 0) heading += 360.0f;
        while (heading >= 360) heading -= 360.0f;
        
        return heading;
    }
    
    // If IMU read fails, return last known heading
    std::lock_guard<std::mutex> lock(position_mutex_);
    return robot_position_.azimuth;
}

bool RobotPosition::isRobotStationary() const {
    // Read accelerometer and gyroscope
    float ax, ay, az, gx, gy, gz;
    if (!imu_.readAccelerometer(ax, ay, az) || !imu_.readGyroscope(gx, gy, gz)) {
        return false;
    }
    
    // Check if acceleration is close to gravity only
    float accel_magnitude = std::sqrt(ax*ax + ay*ay + az*az);
    if (std::abs(accel_magnitude - 1.0f) > 0.05f) {
        return false;
    }
    
    // Check if gyroscope readings are close to zero
    float gyro_magnitude = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (gyro_magnitude > 1.0f) {
        return false;
    }
    
    return true;
}

void RobotPosition::calibrateIMUHeading() {
    // Only calibrate if we have valid GPS data
    if (!gps_identified_ || !isRobotStationary()) {
        std::cerr << "Cannot calibrate heading: Robot must be stationary and GPS identified" << std::endl;
        return;
    }
    
    // Get GPS positions
    Position left_pos = left_gps_->getRawPosition();
    Position right_pos = right_gps_->getRawPosition();
    
    // Calculate GPS-based heading (from left to right GPS)
    float dx = right_pos.x - left_pos.x;
    float dy = right_pos.y - left_pos.y;
    
    if (std::abs(dx) < 0.01f && std::abs(dy) < 0.01f) {
        std::cerr << "GPS positions too close for reliable heading" << std::endl;
        return;
    }
    
    float gps_heading = std::atan2(dy, dx) * 180.0f / M_PI + 90.0f;  // Add 90° for perpendicular heading
    while (gps_heading < 0) gps_heading += 360.0f;
    while (gps_heading >= 360) gps_heading -= 360.0f;
    
    // Get raw IMU heading (without current offset)
    float mx, my, mz;
    if (!imu_.readMagnetometer(mx, my, mz)) {
        std::cerr << "Failed to read magnetometer" << std::endl;
        return;
    }
    
    float raw_imu_heading = std::atan2(my, mx) * 180.0f / M_PI;
    while (raw_imu_heading < 0) raw_imu_heading += 360.0f;
    while (raw_imu_heading >= 360) raw_imu_heading -= 360.0f;
    
    // Calculate new offset
    heading_offset_ = gps_heading - raw_imu_heading;
    last_calibration_time_ = std::chrono::system_clock::now();  // Using system_clock consistently
    
    std::cout << "IMU heading calibrated: offset = " << heading_offset_ 
              << "° (GPS: " << gps_heading << "°, Raw IMU: " << raw_imu_heading << "°)" << std::endl;
}

bool RobotPosition::startUpdateThread(int update_frequency_hz) {
    if (update_thread_running_) {
        return true;  // Already running
    }
    
    update_thread_running_ = true;
    update_thread_ = std::thread(&RobotPosition::updateThreadFunction, this, update_frequency_hz);
    
    return true;
}

void RobotPosition::stopUpdateThread() {
    update_thread_running_ = false;
    
    if (update_thread_.joinable()) {
        update_thread_.join();
    }
}


void RobotPosition::updateThreadFunction(int update_frequency_hz) {
    const auto update_period = std::chrono::milliseconds(1000 / update_frequency_hz);
    
    while (update_thread_running_) {
        // Use steady_clock for timing the loop since it's monotonic
        auto start_time = std::chrono::steady_clock::now();
        
        update();  // Process position data
        
        // Calculate time to sleep using steady_clock for consistent timing
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < update_period) {
            std::this_thread::sleep_for(update_period - elapsed);
        } else {
            // Log if we're not keeping up with desired frequency
            std::cerr << "Position update took longer than period: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() 
                      << "ms (target: " << update_period.count() << "ms)" << std::endl;
        }
    }
}

Position RobotPosition::getPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return robot_position_;
}

void RobotPosition::get2DPosition(float& x, float& y, float& heading) const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    x = robot_position_.x;
    y = robot_position_.y;
    heading = robot_position_.azimuth;
}

float RobotPosition::getHeading() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return robot_position_.azimuth;
}

bool RobotPosition::hasReliablePosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return position_confidence_ >= MIN_CONFIDENCE_THRESHOLD;
}

float RobotPosition::getVelocity() const {
    return current_velocity_;
}

float RobotPosition::getAngularVelocity() const {
    return current_angular_velocity_;
}