#include "Brain.h"


Brain::Brain(boost::asio::io_service& io_service, const std::string& port)
    : ioService(io_service)
    , port_(port)
    , running_(false)
    , connected_(false)
    , active_brain_request_(0)
    , left_voltage_(-11.23)
    , right_voltage_(5.23)
    , macroState_(0) {
    if (!port_.empty()) {
        initializePort();
    }
}

Brain::~Brain() {
    stop();
}

bool Brain::initializePort() {
    try {
        serialPort = std::make_unique<boost::asio::serial_port>(ioService);
        serialPort->open(port_);
        serialPort->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serialPort->set_option(boost::asio::serial_port_base::character_size(8));
        serialPort->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serialPort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serialPort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        connected_ = true;
        std::cout << "Brain initialized on port " << port_ << std::endl;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Failed to initialize Brain port: " << e.what() << std::endl;
        connected_ = false;
        return false;
    }
}

bool Brain::start() {
    if (running_) return true;
    if (!connected_ && !reconnect()) {
        return false;
    }
    running_ = true;
    read_thread_ = std::make_unique<std::thread>(&Brain::readLoop, this);
    std::cout << "Created Thread for brain" << std::endl;
    return true;
}

void Brain::stop() {
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

bool Brain::restart() {
    stop();
    return start();
}

bool Brain::reconnect() {
    if (serialPort && serialPort->is_open()) {
        serialPort->close();
    }
    return initializePort();
}

void Brain::readLoop() {
    std::cout << "Starting read loop" << std::endl;
    
    while (running_) {
        try {
            if (!connected_ && !reconnect()) {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }
            std::cout << "Top of read loop" << std::endl;
            // First read just the header (4 bytes)
            std::vector<uint8_t> header(16);
            boost::system::error_code ec;
            size_t bytes_read = boost::asio::read(*serialPort, boost::asio::buffer(header), ec);

            if (ec) {
                std::cerr << "Read error: " << ec.message() << std::endl;
                connected_ = false;
                continue;
            }

            // Parse flags from header
            uint16_t flags = header[2] | (header[3] << 8);
            
            // Determine if additional data needs to be read
            size_t additional_bytes = 0;
            if (flags & BrainPacket::RESPONSE) {
                if (flags & BrainPacket::BRAIN_REQUEST_MACRO_STATE) additional_bytes += sizeof(uint16_t);
                if (flags & BrainPacket::BRAIN_REQUEST_CONTROL_VDC) additional_bytes += sizeof(float) * 2;
            } else {
                if (flags & BrainPacket::REQUEST_VEX_LINK_POS) additional_bytes += sizeof(float) * 3;
                if (flags & BrainPacket::REQUEST_BATTERY_LEVEL) additional_bytes += sizeof(float);
                if (flags & BrainPacket::REQUEST_GPS_INIT) additional_bytes += sizeof(float) * 2;
            }

            // Read additional data if needed
            std::vector<uint8_t> full_packet = header;
            if (additional_bytes > 0) {
                std::vector<uint8_t> payload(additional_bytes);
                bytes_read = boost::asio::read(*serialPort, boost::asio::buffer(payload), ec);
                if (ec) {
                    std::cerr << "Payload read error: " << ec.message() << std::endl;
                    continue;
                }
                full_packet.insert(full_packet.end(), payload.begin(), payload.end());
            }

            // Process the packet
            if (flags & BrainPacket::RESPONSE) {
                parseBrainResponse(full_packet);
            } else {
                active_brain_request_ = flags;
                processBrainRequest(std::vector<uint8_t>());
            }

            std::this_thread::sleep_for(READ_DELAY);

        } catch (const std::exception& e) {
            std::cerr << "Error in read loop: " << e.what() << std::endl;
            connected_ = false;
            std::this_thread::sleep_for(RECONNECT_DELAY);
        }
    }
}






void Brain::processBrainRequest(const std::vector<uint8_t>& buffer) 
{
    try {
        // Create response packet
        std::vector<uint8_t> responseData = BrainPacket(active_brain_request_ | BrainPacket::RESPONSE, true).toBytes();

        // Add data based on request type
        if (active_brain_request_ & BrainPacket::BRAIN_REQUEST_MACRO_STATE) {

            responseData.insert(responseData.end(), 
                             reinterpret_cast<uint8_t*>(&macroState_),
                             reinterpret_cast<uint8_t*>(&macroState_) + sizeof(uint16_t));
        }

        if (active_brain_request_ & BrainPacket::BRAIN_REQUEST_CONTROL_VDC) {
            float voltages[2] = {left_voltage_, right_voltage_};
            responseData.insert(responseData.end(),
                             reinterpret_cast<uint8_t*>(&voltages),
                             reinterpret_cast<uint8_t*>(&voltages) + sizeof(voltages));
        }

        // Send the response
        boost::asio::write(*serialPort, boost::asio::buffer(responseData));

    } catch (const std::exception& e) {
        std::cerr << "Failed to process request: " << e.what() << std::endl;
    }
}

void Brain::parseBrainResponse(const std::vector<uint8_t>& buffer) {
    if (buffer.size() < 4) {
        std::cerr << "Error: Received incomplete response." << std::endl;
        return;
    }

    uint16_t flags = buffer[2] | (buffer[3] << 8);
    if (!(flags & BrainPacket::RESPONSE)) {
        std::cerr << "Error: Expected response but received request!" << std::endl;
        return;
    }

    size_t offset = 4;

    //  Parse VEX Link Position (X, Y, Heading)
    if (flags & BrainPacket::REQUEST_VEX_LINK_POS) {
        if (offset + sizeof(float) * 3 > buffer.size()) {
            std::cerr << "Error: Incomplete position data in response." << std::endl;
            return;
        }

        float xPos, yPos, heading;
        std::memcpy(&xPos, &buffer[offset], sizeof(float)); offset += sizeof(float);
        std::memcpy(&yPos, &buffer[offset], sizeof(float)); offset += sizeof(float);
        std::memcpy(&heading, &buffer[offset], sizeof(float)); offset += sizeof(float);

        std::cout << "VEX Link Position: X=" << xPos << ", Y=" << yPos << ", Heading=" << heading << "°" << std::endl;
    }

    // Parse Battery Level
    if (flags & BrainPacket::REQUEST_BATTERY_LEVEL) {
        if (offset + sizeof(float) > buffer.size()) {
            std::cerr << "Error: Incomplete battery data in response." << std::endl;
            return;
        }

        float batteryLevel;
        std::memcpy(&batteryLevel, &buffer[offset], sizeof(float)); offset += sizeof(float);

        std::cout << "Battery Level: " << batteryLevel << "%" << std::endl;
    }

    // Parse GPS Initialization Data (Offsets)
    if (flags & BrainPacket::REQUEST_GPS_INIT) {
        if (offset + sizeof(float) * 2 > buffer.size()) {
            std::cerr << "Error: Incomplete GPS offset data in response." << std::endl;
            return;
        }

        float gpsOffsetX, gpsOffsetY;
        std::memcpy(&gpsOffsetX, &buffer[offset], sizeof(float)); offset += sizeof(float);
        std::memcpy(&gpsOffsetY, &buffer[offset], sizeof(float)); offset += sizeof(float);

        std::cout << "GPS Offsets: X=" << gpsOffsetX << ", Y=" << gpsOffsetY << std::endl;
    }

    // Add other response parsing as needed
}

void Brain::sendRequest(uint16_t requestFlags) {
    if (!connected_) {
        std::cerr << "Error: Not connected to VEX Brain." << std::endl;
        return;
    }

    BrainPacket packet(requestFlags, false);
    std::vector<uint8_t> requestData = packet.toBytes();


    boost::asio::write(*serialPort, boost::asio::buffer(requestData));
}


