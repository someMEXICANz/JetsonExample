#include <GPS.h>


GPS::GPS(boost::asio::io_service& io_service, const std::string& port, size_t history_size)
    : ioService(io_service)
    , port_(port)
    , position_history_(history_size)
{
    if (!port_.empty()) {
        initializePort();
    }
}

bool GPS::initializePort() {
    try {
        serialPort = std::make_unique<boost::asio::serial_port>(ioService);
        serialPort->open(port_);
        serialPort->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serialPort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort->set_option(boost::asio::serial_port_base::character_size(8));
        serialPort->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serialPort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        
        connected_ = true;
        std::cout << "GPS initialized on port " << port_ << std::endl;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Failed to initialize GPS port: " << e.what() << std::endl;
        connected_ = false;
        return false;
    }
}


GPS::~GPS() 
{
    stop();
}

bool GPS::start() {
    if (running_) return true;
    
    if (!connected_ && !reconnect()) {
        return false;
    }
    
    running_ = true;
    read_thread_ = std::make_unique<std::thread>(&GPS::readLoop, this);
    return true;
}

void GPS::stop() {
    running_ = false;
    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }
    read_thread_.reset();
    
    if (serialPort && serialPort->is_open()) {
        serialPort->close();
    }
    connected_ = false;
}

bool GPS::restart() 
{
    stop();
    return start();
}

bool GPS::reconnect() {
    if (serialPort && serialPort->is_open()) {
        serialPort->close();
    }
    return initializePort();
}

void GPS::readLoop() {
    std::cout << "Starting read loop" << std::endl;
    boost::asio::deadline_timer timer(ioService);
    
    while (running_) {
        try {
            if (!connected_ && !reconnect()) {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }

            std::vector<unsigned char> buffer(16);
            boost::system::error_code ec;
            
            // Set timeout
            timer.expires_from_now(boost::posix_time::milliseconds(READ_TIMEOUT.count()));
            
            size_t bytes_read = boost::asio::read(*serialPort, boost::asio::buffer(buffer), ec);
            
            if (ec) {
                std::cerr << "Read error: " << ec.message() << std::endl;
                failed_reads_++;
                connected_ = false;
                continue;
            }

            total_reads_++;
            
            if (bytes_read == 16 && buffer[15] == 0x33 && buffer[14] == 0xCC) {
                processBuffer(buffer);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in read loop: " << e.what() << std::endl;
            connected_ = false;
            std::this_thread::sleep_for(RECONNECT_DELAY);
        }
    }
}


void GPS::processBuffer(const std::vector<unsigned char>& buffer) 
{
    status_ = buffer[1];  // ✅ Status now belongs to GPS

    int16_t x_raw, y_raw, z_raw, az_raw, el_raw, rot_raw;
    std::memcpy(&x_raw, &buffer[2], 2);
    std::memcpy(&y_raw, &buffer[4], 2);
    std::memcpy(&z_raw, &buffer[6], 2);
    std::memcpy(&az_raw, &buffer[8], 2);
    std::memcpy(&el_raw, &buffer[10], 2);
    std::memcpy(&rot_raw, &buffer[12], 2);

    Position new_position(x_raw / 10000.0f, y_raw / 10000.0f, z_raw / 10000.0f,
                          az_raw / 32768.0f * 180.0f, el_raw / 32768.0f * 180.0f,
                          rot_raw / 32768.0f * 180.0f);
    
    if (validatePosition(new_position)) {
        std::lock_guard<std::mutex> lock(position_mutex_);
        position_ = new_position;
        position_history_.push_back(new_position);
    }
    else 
    {
        invalid_positions_++;
    }
}

bool GPS::validatePosition(const Position& pos) {
    if (!pos.isValid()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(position_mutex_);
    if (!position_history_.empty()) {
        const Position& last_pos = position_history_.back();
        
        // Check for position jumps
        if (pos.distanceTo(last_pos) > MAX_POSITION_JUMP) {
            return false;
        }
        
        // Check for angle jumps
        float angle_diff = std::abs(pos.azimuth - last_pos.azimuth);
        if (angle_diff > 180.0f) {
            angle_diff = 360.0f - angle_diff;
        }
        if (angle_diff > MAX_ANGLE_JUMP) {
            return false;
        }
    }
    
    return true;
}




Position GPS::getPosition() {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return position_;
}

Position GPS::getLastValidPosition() {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return position_history_.empty() ? Position() : position_history_.back();
}

std::vector<Position> GPS::getPositionHistory() {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return std::vector<Position>(position_history_.begin(), position_history_.end());
}
