#include "GPS.h"

GPS::GPS(boost::asio::io_service& io_service, const std::string& port)
    : ioService(io_service),
      port_(port),
      running_(false),
      connected_(false),
      status_(0)
{
    // Initialize position with zeros
    position_ = Position();
    
    if (!port_.empty()) {
        initializePort();
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

bool GPS::restart() {
    stop();
    return start();
}

bool GPS::reconnect() {
    if (serialPort && serialPort->is_open()) {
        serialPort->close();
    }
    return initializePort();
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

void GPS::readLoop() {
    std::cout << "GPS read loop started" << std::endl;
    boost::asio::deadline_timer timer(ioService);
    
    while (running_) {
        try {
            if (!connected_ && !reconnect()) {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }

            std::vector<unsigned char> buffer(16); // Based on Python code packet size
            boost::system::error_code ec;
            
            // Set timeout
            timer.expires_from_now(boost::posix_time::milliseconds(READ_TIMEOUT.count()));
            
            // Read until end marker 0xCC33 (based on Python code)
            std::size_t bytes_read = boost::asio::read(*serialPort, boost::asio::buffer(buffer), 
                                     boost::asio::transfer_at_least(16), ec);
            
            if (ec) {
                std::cerr << "GPS read error: " << ec.message() << std::endl;
                connected_ = false;
                continue;
            }

            if (bytes_read == 16 && buffer[14] == 0xCC && buffer[15] == 0x33) {
                processBuffer(buffer);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in GPS read loop: " << e.what() << std::endl;
            connected_ = false;
            std::this_thread::sleep_for(RECONNECT_DELAY);
        }
    }
    
    std::cout << "GPS read loop stopped" << std::endl;
}

void GPS::processBuffer(const std::vector<unsigned char>& buffer) {
    // Extract status byte
    status_ = buffer[1];

    // Parse position data (following Python implementation)
    int16_t x_raw, y_raw, z_raw, az_raw, el_raw, rot_raw;
    std::memcpy(&x_raw, &buffer[2], 2);
    std::memcpy(&y_raw, &buffer[4], 2);
    std::memcpy(&z_raw, &buffer[6], 2);
    std::memcpy(&az_raw, &buffer[8], 2);
    std::memcpy(&el_raw, &buffer[10], 2);
    std::memcpy(&rot_raw, &buffer[12], 2);

    // Convert to proper units, just like in Python code
    float x = x_raw / 10000.0f;
    float y = y_raw / 10000.0f;
    float z = z_raw / 10000.0f;
    float azimuth = az_raw / 32768.0f * 180.0f; 
    float elevation = el_raw / 32768.0f * 180.0f;
    float rotation = rot_raw / 32768.0f * 180.0f;

    // Update position
    std::lock_guard<std::mutex> lock(position_mutex_);
    position_ = Position(x, y, z, azimuth, elevation, rotation);
}

Position GPS::getRawPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return position_;
}