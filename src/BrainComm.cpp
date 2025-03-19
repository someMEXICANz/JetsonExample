#include "BrainComm.h"
#include <iomanip>
using namespace std;

namespace Brain {


// Constructor
BrainComm::BrainComm(boost::asio::io_service& service, const std::string& usb_port)
    : io_service(service)
    , port(usb_port)
    , running(false)
    , connected(false)
    , serial_port(nullptr)
    , request_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , response_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , request_in_progress(false)
    , request_retry_count(0)
    , sendData(false)
    , last_send_time(0)
    , last_received_time(0)
    , last_request_time(0)
    // , read_buffer(CommConstants::MAX_BUFFER_SIZE)
    // , buffer_index(0)
    , BrainBatteryLvl(0)
{
    // Initialize positions to zero values
    left_gps_position = {0.0, 0.0, 0.0};
    right_gps_position = {0.0, 0.0, 0.0};
    sister_position = {0.0, 0.0, 0.0};
    left_gps_offset = {0.0, 0.0, 0.0};
    right_gps_offset = {0.0, 0.0, 0.0};
    
    // Initialize command structures
    current_motor_command = {0.0, 0.0, 0};
    current_control_flags = {0};
    current_battery_lvl = 0;
    
    // Try to connect if port is provided
    if (!port.empty()) {
        initializePort();
    }
}

BrainComm::~BrainComm() 
{
    stop();
}



bool BrainComm::start() {
    if (running) return true;
    
    if (!connected && !reconnect()) {
        return false;
    }
 
    running = true;
    read_thread = std::make_unique<std::thread>(&BrainComm::readLoop, this);
    write_thread = std::make_unique<std::thread>(&BrainComm::writeLoop, this);
    return true;
}

void BrainComm::stop() 
{
    running = false;
    write_condition.notify_all();
    
    // Cancel any pending operations on the serial port
    if (serial_port && serial_port->is_open()) {
        serial_port->cancel();
    }
    
    // Wait for threads to finish
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
    }
    read_thread.reset();
    
    if (write_thread && write_thread->joinable()) {
        write_thread->join();
    }
    write_thread.reset();
    
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
    connected = false;
}

bool BrainComm::restart() {
    stop();
    return start();
}




bool BrainComm::reconnect() 
{
    if (serial_port && serial_port->is_open()) {
            serial_port->close();
    }
     
    return initializePort();
}


bool BrainComm::initializePort() 
{
    try {
        serial_port = make_unique<boost::asio::serial_port>(io_service);
        serial_port->open(port);
        serial_port->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_port->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        connected = true;
        stats.recordConnectionStatus(true);
        cerr << "Brain communication initialized on port " << port << endl;
        return true;

    } catch (const boost::system::system_error& e) 
    {
        stats.logError(CommError::ConnectionLost, "Failed to initialize port: " + string(e.what()));
        cerr << "Failed to initialize port: " << endl ;
        connected = false;
        return false;
    }
}



bool BrainComm::updateRequests(uint16_t flags) 
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pending_requests.push(flags);
    cerr << "Request queued with flags: 0x" << hex << flags << dec << endl;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BrainComm::readLoop() {
    std::cout << "Read thread started" << std::endl;
    
    boost::asio::deadline_timer timer(io_service);
    std::vector<uint8_t> buffer(CommConstants::MAX_BUFFER_SIZE);
    size_t buffer_index = 0;
    
    while (running) 
    {

            if (!connected) 
            {
                if (reconnect()) 
                {
                    std::cout << "Successfully reconnected in read thread" << std::endl;
                } 
                else 
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }
            
            // Create a small buffer for reading one byte at a time
            uint8_t read_byte;
            boost::system::error_code ec;
            

            
            // Read a single byte
            size_t bytes_read = boost::asio::read(
                *serial_port, 
                boost::asio::buffer(&read_byte, 1),
                ec
            );
    
            
            if (ec) 
            {
                if (ec != boost::asio::error::operation_aborted ) 
                {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                    stats.incrementReceiveStats(false);
                    std::lock_guard<std::mutex> lock(state_mutex);
                    connected = false;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            if (bytes_read > 0) 
            {
    
                if (buffer_index < CommConstants::MAX_BUFFER_SIZE) 
                {
                    buffer[buffer_index++] = read_byte;
                } 
                else 
                {
                    stats.recordBufferOverrun();
                    buffer_index = 0;
                    continue;
                }
               
                
                // Now process the buffer looking for complete messages
                bool found_start = false;
                size_t start_pos = 0;
                
                // Search for start marker
                if (buffer_index >= 2) 
                {  // Prevent underflow
                    for (size_t i = 0; i <= buffer_index - 2; i++) 
                    {
                        if (buffer[i] == CommConstants::START_MARKER_1 && 
                            buffer[i+1] == CommConstants::START_MARKER_2) 
                        {
                            found_start = true;
                            start_pos = i;
                            break;
                        }
                    }
                }
                
                // If start marker is not at the beginning, shift buffer
                if (found_start && start_pos > 0) 
                {
                    // cerr << " Start Marker Found but not at the beginning, shifting buffer " << endl;
                    memmove(buffer.data(), buffer.data() + start_pos, buffer_index - start_pos);
                    buffer_index -= start_pos;
                    start_pos = 0;
                }
               
                
                // If we have enough data for a header
                if (found_start && buffer_index >= sizeof(RequestHeader)) 
                {
                    RequestHeader* header = reinterpret_cast<RequestHeader*>(buffer.data());
                    size_t total_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
                    
                    if (buffer_index >= total_length) 
                    {
                        // Verify end marker
                        EndMarker* end = reinterpret_cast<EndMarker*>(buffer.data() + sizeof(RequestHeader) 
                                                                                    + header->length);
                            
                        if (end->marker[0] == CommConstants::END_MARKER_1 && 
                            end->marker[1] == CommConstants::END_MARKER_2) 
                        {
                            
                            // Process the message
                            handleMessage(buffer.data(), total_length);
                            // Move any remaining data to the start of the buffer
                            if (buffer_index > total_length) 
                            {
                                memmove(buffer.data(), buffer.data() + total_length, 
                                       buffer_index - total_length);
                                buffer_index -= total_length;
                            } 
                            else 
                            {
                                buffer_index = 0;
                            }
                        } 

                        else 
                        {
                            std::cerr << "Invalid end marker in message" << std::endl;
                            // Shift past the start marker to look for next valid message
                            if (buffer_index > 2) 
                            {
                                memmove(buffer.data(), buffer.data() + 2, buffer_index - 2);
                                buffer_index -= 2;
                            } 
                            else 
                            {
                                buffer_index = 0;
                            }
                        }
                    }
                }
            }
    }
    
    std::cout << "Read thread stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::writeLoop() 
{

    
    boost::asio::deadline_timer timer(io_service);
    uint16_t request_flag;
    uint16_t response_flag;
    bool sendRequest;
    bool shouldSendData;
    auto current_time = chrono::duration_cast<std::chrono::milliseconds>(
                        chrono::steady_clock::now().time_since_epoch()).count();
    
    while (running) 
    {
        request_flag = 0;
        sendRequest = false;
        shouldSendData = false;

        current_time = chrono::duration_cast<std::chrono::milliseconds>(
                       chrono::steady_clock::now().time_since_epoch()).count();

        if (!connected) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (!request_in_progress && !pending_requests.empty()) 
            {
                request_flag = pending_requests.front();
                sendRequest = true;
                request_in_progress = true;
                last_request_time = current_time;
                state_mutex.unlock();
            }
        }
            
        if(sendRequest)
        {
            sendRequests(request_flags);
        } 
        
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            shouldSendData = sendData && (current_time - last_send_time >= CommConstants::RESPONSE_UPDATE_PERIOD.count());
            response_flag = response_flags;
            state_mutex.unlock();

        

            if(shouldSendData)
            {
                sendResponse(response_flag);
                std::lock_guard<std::mutex> lock(state_mutex);
                last_send_time = current_time;
                state_mutex.unlock();
            }
        }


        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if(request_in_progress)
            {
                if(current_time - last_request_time >= CommConstants::READ_TIMEOUT.count())
                {
                     if (request_retry_count >= CommConstants::MAX_REQUEST_RETRIES)
                     {
                        stats.logError(CommError::RequestTimeout, "Max retries exceeded for request");
                        pending_requests.pop();
                        request_retry_count = 0;
                        request_in_progress = false;
                        state_mutex.unlock();
                        cerr << "Request timed out after " << CommConstants::MAX_REQUEST_RETRIES 
                             << " retries (flags: 0x" << hex << request_flag 
                             << dec << ")" << endl;

                     }
                     
                     else
                     {
                        request_retry_count++;
                        request_in_progress = false;
                        state_mutex.unlock();
                        cerr << "Retrying request (flags: 0x" << hex << pending_requests.front() 
                             << dec << ", attempt: " << static_cast<int>(request_retry_count) 
                             << ")" << endl;
                     }

                }

                else
                {
                    state_mutex.unlock();
                }

            }

            else
            {
                state_mutex.unlock();
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::cout << "Write thread stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::sendRequests(uint16_t flags) {
    if (!running || !connected || !serial_port || !serial_port->is_open()) {
        return false;
    }
    
    // Prepare header and end marker
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Request);
    header.flags = flags;
    header.length = 0;
    
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    try {
        boost::system::error_code ec;
        boost::asio::deadline_timer timer(io_service);
        
        // Set timeout for write operation
        timer.expires_from_now(boost::posix_time::milliseconds(CommConstants::READ_TIMEOUT.count()));
        bool timed_out = false;
        
        timer.async_wait([&](const boost::system::error_code& error) {
            if (!error) {
                timed_out = true;
                if (serial_port && serial_port->is_open()) {
                    serial_port->cancel();
                }
            }
        });
        
        // Write header
        size_t header_written = boost::asio::write(*serial_port, 
                                                 boost::asio::buffer(&header, sizeof(header)), 
                                                 ec);
        
        if (!ec && header_written == sizeof(header)) {
            // Write end marker
            size_t end_written = boost::asio::write(*serial_port, 
                                                  boost::asio::buffer(&end_marker, sizeof(end_marker)), 
                                                  ec);
            
            timer.cancel();  // Cancel timeout timer
            
            if (!ec && end_written == sizeof(end_marker)) {
                std::cout << "Request sent with flags: 0x" << std::hex << flags << std::dec << std::endl;
                stats.incrementTransmitStats(true);
                return true;
            }
        }
        
        timer.cancel();  // Make sure timer is canceled
        
        if (ec || timed_out) {
            throw boost::system::system_error(ec ? ec : boost::asio::error::timed_out);
        }
        
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Failed to send request: " << e.what() << std::endl;
        stats.incrementTransmitStats(false);
        stats.logError(CommError::TransmissionFailed, "Failed to send request: " + std::string(e.what()));
        connected = false;
        return false;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::handleMessage(const uint8_t* buffer, size_t length) {
    // Ensure we have at least a header
    if (length < sizeof(RequestHeader)) {
        std::cerr << "Message too short for header" << std::endl;
        return false;
    }
    
    const RequestHeader* header = reinterpret_cast<const RequestHeader*>(buffer);
    
    // Verify start marker
    if (header->start_marker[0] != CommConstants::START_MARKER_1 || 
        header->start_marker[1] != CommConstants::START_MARKER_2) {
        std::cerr << "Invalid start marker in message" << std::endl;
        return false;
    }
    
    // Calculate total expected length
    size_t expected_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
    if (length < expected_length) {
        std::cerr << "Message shorter than expected length" << std::endl;
        return false;
    }
    
    // Verify end marker
    const EndMarker* end = reinterpret_cast<const EndMarker*>(
        buffer + sizeof(RequestHeader) + header->length);
        
    if (end->marker[0] != CommConstants::END_MARKER_1 || 
        end->marker[1] != CommConstants::END_MARKER_2) {
        std::cerr << "Invalid end marker in message" << std::endl;
        return false;
    }
    
    // Process based on message type
    MessageType msg_type = static_cast<MessageType>(header->message_type);
    uint16_t flags = header->flags;
    
    switch (msg_type) {
        case MessageType::Request:
            std::cerr << "Received request message with flags: 0x" 
                     << std::hex << flags << std::dec << std::endl;
            
            {
                // Use a lock to update shared state
                std::lock_guard<std::mutex> lock(state_mutex);
                
                // Set response flags based on what the Brain is requesting
                response_flags = flags;
                
                // Now we should send data (but only if the Brain actually requested something)
                sendData = (flags != static_cast<uint16_t>(Brain::RequestFlag::NoData));
                
                // Update the last received time
                last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                    
                // If we got a valid message, the device must be connected
                // connected = true;
            }
            
            // Signal the write thread to wake up
            {
                std::unique_lock<std::mutex> lock(write_mutex);
                write_condition.notify_one();
            }
            
            // Send acknowledgment (status 0 = success)
            sendAcknowledgment(flags, 0x00);
            
            // Update statistics
            stats.incrementReceiveStats(true);
            break;
            
        case MessageType::Acknowledgment:
            if (header->length == 1) {
                // Get status byte
                uint8_t status = *(buffer + sizeof(RequestHeader));
                
                std::cerr << "Received acknowledgment message with flags: 0x" 
                         << std::hex << flags << ", status: " << static_cast<int>(status) 
                         << std::dec << std::endl;
                
                std::lock_guard<std::mutex> lock(state_mutex);
                if (request_in_progress && !pending_requests.empty() && 
                    pending_requests.front() == flags) {
                    
                    // Process acknowledgment
                    request_flags = flags;  // Store what we requested
                    pending_requests.pop();
                    request_in_progress = false;
                    request_retry_count = 0;
                    
                    // Update connection status and timestamp
                    // connected = true;
                    last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    
                    std::cerr << "Acknowledgment processed for flags: 0x" 
                             << std::hex << flags 
                             << ", status: " << static_cast<int>(status) 
                             << std::dec << std::endl;
                }
                
                // Update statistics
                stats.incrementReceiveStats(true);
            } else {
                std::cerr << "Invalid acknowledgment message length" << std::endl;
                stats.incrementReceiveStats(false);
                return false;
            }
            break;
            
        case MessageType::Response:
            // std::cerr << "Received response message with flags: 0x" 
            //          << std::hex << flags << std::dec 
            //          << ", length: " << header->length << std::endl;
            
            // Process the received data
            if (!processReceivedData(flags, 
                                   buffer + sizeof(RequestHeader), 
                                   header->length)) {
                stats.incrementReceiveStats(false);
                return false;
            }
            
            // Update connection status and timestamp
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                // connected = true;
                last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            }
            
            // Update statistics
            stats.incrementReceiveStats(true);
            break;
            
        case MessageType::Handshake:
            std::cerr << "Received handshake message" << std::endl;
            
            // Update connection status and timestamp
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                // connected = true;
                last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            }
            
            // Signal the write thread
            {
                std::unique_lock<std::mutex> lock(write_mutex);
                write_condition.notify_one();
            }
            
            // Respond to handshake with acknowledgment
            sendAcknowledgment(0, 0x00);
            
            // Update statistics
            stats.incrementReceiveStats(true);
            break;
            
        default:
            std::cerr << "Unknown message type: " 
                     << static_cast<int>(header->message_type) << std::endl;
            stats.incrementReceiveStats(false);
            return false;
    }
    
    return true;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::sendResponse(uint16_t flags) {
    // Calculate total payload size based on active flags
    size_t payload_size = 0;
    
    if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) 
        payload_size += sizeof(MotorCommand);
        
    if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) 
        payload_size += sizeof(ControlFlags);

    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel))
        payload_size += sizeof(uint32_t);
    
    // Prepare request header
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Response);
    header.flags = flags;
    header.length = payload_size;
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    // Prepare payload buffer
    std::vector<uint8_t> payload(payload_size);
    size_t offset = 0;
    
    // Add data based on flags - lock data mutex while accessing shared data
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        // Motor voltages
        if (flags & static_cast<uint16_t>(RequestFlag::MotorVoltages)) {
            // Update timestamp before sending
            current_motor_command.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
                
            memcpy(payload.data() + offset, &current_motor_command, sizeof(MotorCommand));
            offset += sizeof(MotorCommand);
        }
        
        // Macro controls
        if (flags & static_cast<uint16_t>(RequestFlag::MacroControls)) {
            memcpy(payload.data() + offset, &current_control_flags, sizeof(ControlFlags));
            offset += sizeof(ControlFlags);
        }
        // Jetson Battery Level
        if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) {
            memcpy(payload.data() + offset, &current_battery_lvl, sizeof(uint32_t));
            offset += sizeof(uint32_t);
        }
    }
    
    // Send header, payload, and end marker using async_write
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    
    if (payload_size > 0) {
        buffers.push_back(boost::asio::buffer(payload));
    }
    
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));

    boost::system::error_code ec;
    size_t bytes_written = boost::asio::write(*serial_port, buffers, ec);
    
    // std::cerr << "DEBUG: Sending response - flags: 0x" << std::hex << flags 
    //          << std::dec << ", payload size: " << payload_size << std::endl;
             

    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::sendAcknowledgment(uint16_t flags, uint8_t status) 
{
    // Create a header with flags, message type, and length=1
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Acknowledgment);
    header.flags = flags;
    header.length = 1;  // 1 byte payload for status
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    // Use scatter-gather approach for efficiency
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    buffers.push_back(boost::asio::buffer(&status, sizeof(status)));
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));
    
    boost::system::error_code ec;

    size_t bytes_written = boost::asio::write(*serial_port, buffers, ec);
    std::cerr << "DEBUG: Sending acknowledgment - flags: 0x" 
             << std::hex << flags << ", status: " << static_cast<int>(status) 
             << std::dec << std::endl;
             
    
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BrainComm::processReceivedData(uint16_t flags, const uint8_t* data, uint16_t length) {
    // Calculate expected payload size based on flags
    size_t expected_size = 0;
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData))
        expected_size += sizeof(Position2D) * 2;  // position and offset
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData))
        expected_size += sizeof(Position2D) * 2;  // position and offset
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData))
        expected_size += sizeof(Position2D);
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel))
        expected_size += sizeof(uint32_t);
    
    // Validate total payload size
    if (length < expected_size) {
        stats.logError(CommError::InvalidPacket, "Received data size mismatch");
        cerr << "Data packet size mismatch. Expected: " << expected_size 
             << ", got: " << length << endl;
        return false;
    }
    
    // Lock the data mutex to safely update stored values
    std::lock_guard<std::mutex> lock(data_mutex);
    
    size_t offset = 0;
    
    // Process left GPS data
    if (flags & static_cast<uint16_t>(RequestFlag::LeftGPSData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&left_gps_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // Get the offset data
            if (offset + sizeof(Position2D) <= length) {
                memcpy(&left_gps_offset, data + offset, sizeof(Position2D));
                offset += sizeof(Position2D);
                
                // cerr << "Received left GPS position: (" 
                //      << left_gps_position.x << ", " 
                //      << left_gps_position.y << ", " 
                //      << left_gps_position.heading << ")" << endl;
            }
        }
    }
    
    // Process right GPS data
    if (flags & static_cast<uint16_t>(RequestFlag::RightGPSData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&right_gps_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // Get the offset data
            if (offset + sizeof(Position2D) <= length) {
                memcpy(&right_gps_offset, data + offset, sizeof(Position2D));
                offset += sizeof(Position2D);
                
                // cerr << "Received right GPS position: (" 
                //      << right_gps_position.x << ", " 
                //      << right_gps_position.y << ", " 
                //      << right_gps_position.heading << ")" << endl;
            }
        }
    }
    
    // Process sister robot data
    if (flags & static_cast<uint16_t>(RequestFlag::SisterData)) {
        if (offset + sizeof(Position2D) <= length) {
            memcpy(&sister_position, data + offset, sizeof(Position2D));
            offset += sizeof(Position2D);
            
            // cerr << "Received sister position: (" 
            //      << sister_position.x << ", " 
            //      << sister_position.y << ", " 
            //      << sister_position.heading << ")" << endl;
        }
    }
    
    // Process battery level
    if (flags & static_cast<uint16_t>(RequestFlag::BatteryLevel)) 
    {
        if (offset + sizeof(uint32_t) <= length) {
            memcpy(&BrainBatteryLvl, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            
            // cerr << "Received Brain battery level: " 
            //      << BrainBatteryLvl << "%" << endl;
        }
    }

    stats.incrementReceiveStats(true);
    return true;
}

void BrainComm::setMotorVoltages(float left, float right) 
{
    lock_guard<mutex> lock(data_mutex);
    current_motor_command.left_voltage = left;
    current_motor_command.right_voltage = right;
    current_motor_command.timestamp = 
        chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

void BrainComm::setMacroBits(uint32_t macro_bits) 
{
    lock_guard<mutex> lock(data_mutex);
    current_control_flags.macro_bits = macro_bits;
}

void BrainComm::setJetsonBattery(uint32_t level)
{
    lock_guard<mutex> lock(data_mutex);
    current_battery_lvl = level;
}

Position2D BrainComm::getLeftGPSData() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return left_gps_position;
}

Position2D BrainComm::getRightGPSData() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return right_gps_position;
}

Position2D BrainComm::getSisterPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return sister_position;
}

Position2D BrainComm::getLeftGPSOffset() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return left_gps_offset;
}

Position2D BrainComm::getRightGPSOffset() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return right_gps_offset;
}

uint32_t BrainComm::getBrainBattery() const {
    std::lock_guard<std::mutex> lock(data_mutex);
    return BrainBatteryLvl;
}

} // namespace Brain