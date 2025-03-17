#include "BrainComm.h"
#include <iomanip>
using namespace std;

namespace Brain {
std::atomic<bool> BrainComm::read_in_progress{false};
BrainComm::BrainComm(boost::asio::io_service& service, const std::string& usb_port)
    : io_service(service)
    , port(usb_port)
    , running(false)
    , connected(false)
    , serial_port(nullptr)
    , io_thread(nullptr)
    , request_in_progress(false)
    , request_retry_count(0)
    , request_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , response_flags(static_cast<uint16_t>(RequestFlag::NoData))

    , sendData(false)
    , last_send_time(0)
    , last_received_time(0)
    , last_request_time(0)
    , timer(io_service)
    , read_buffer(CommConstants::MAX_BUFFER_SIZE)
    , buffer_index(0)
    , brain_battery(0)
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


    
    // Try to connect if port is provided
    if (!port.empty()) {
        initializePort();
    }
}

BrainComm::~BrainComm() {
    stop();
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
        cout << "Brain communication initialized on port " << port << endl;
        return true;

    } catch (const boost::system::system_error& e) 
    {
        stats.logError(CommError::ConnectionLost, "Failed to initialize port: " + string(e.what()));
        connected = false;
        return false;
    }
}

bool BrainComm::start() {
    if (running) return true;
    
    if (!connected && !reconnect()) {
        return false;
    }
    
    running = true;
    buffer_index = 0;
    
    // Start reading from the serial port
    startAsyncRead();
    
    // Start the timer for periodic operations
    timer.expires_from_now(boost::posix_time::milliseconds(20));
    timer.async_wait(std::bind(&BrainComm::handleTimer, this, std::placeholders::_1));
    
    // Create a thread to run the io_service
	io_thread = make_unique<thread>([this]() {
    try {
        boost::system::error_code ec;
        while (running) {
            // Run until there are no more handlers to execute
            io_service.poll(ec);
            
            // Reset io_service so it can be reused
            io_service.reset();
            
            // Sleep a short time to avoid high CPU usage
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    } 
    catch (const std::exception& e) {
        cerr << "Exception in io_service thread: " << e.what() << endl;
    }
});
    
    return true;
}

void BrainComm::stop() {
    running = false;
    
    // Cancel any pending asynchronous operations
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->cancel();
        } catch (...) {}
    }
    
    timer.cancel();
    
    if (io_thread && io_thread->joinable()) {
        io_thread->join();
    }
    
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->close();
        } catch (...) {}
    }
    
    connected = false;
}

bool BrainComm::restart() 
{
    stop();
    return start();
}

bool BrainComm::reconnect() {
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->close();
        } catch (...) {}
    }
    
    // Reset communication state
    std::lock_guard<std::mutex> lock(state_mutex);
    while (!pending_requests.empty()) pending_requests.pop();
    request_in_progress = false;
    request_retry_count = 0;
    sendData = false;
    buffer_index = 0;  // Reset buffer
    
    // Attempt to initialize the port
    if (initializePort()) {
        // Start the read loop immediately after successful reconnection
        startAsyncRead();
        return true;
    }
    return false;
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


void BrainComm::startAsyncRead() {
    if (!running) {
        return;
    }
    
    // Skip if already reading - prevents double reads
    if (read_in_progress) {
        return;
    }
    
    // Skip if serial port isn't available
    if (!serial_port || !serial_port->is_open()) {
        return;
    }
    
    read_in_progress = true;
    
    serial_port->async_read_some(
        boost::asio::buffer(&read_buffer[buffer_index], 1),
        std::bind(&BrainComm::handleRead, this,
                  std::placeholders::_1,
                  std::placeholders::_2)
    );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred) 
{
    if (!running) {
        return;
    }
    
    // Mark that we're no longer in a read operation
    read_in_progress = false;
    
    if (ec) {
        if (ec != boost::asio::error::operation_aborted) {
            std::cerr << "DEBUG: Read error: " << ec.message() << std::endl;
            stats.logError(CommError::ReadTimeout, "Read error: " + ec.message());
            stats.incrementReceiveStats(false);
            
            // Only mark as disconnected for serious errors, not timeouts
            if (ec == boost::asio::error::bad_descriptor || 
                ec == boost::asio::error::broken_pipe ||
                ec == boost::asio::error::not_connected) {
                connected = false;
                std::cerr << "DEBUG: Connection lost to Brain" << std::endl;
            }
        }
        
        // Always try to restart reading if port is open
        if (serial_port && serial_port->is_open()) {
            buffer_index = 0;  // Reset buffer index for clean state
            startAsyncRead();
        }
        return;
    }
    
    // Successfully read data
    if (bytes_transferred > 0) {
        // Safety check for buffer overflow
        if (buffer_index + bytes_transferred > CommConstants::MAX_BUFFER_SIZE) {
            std::cerr << "DEBUG: Buffer would overflow, resetting buffer" << std::endl;
            buffer_index = 0;
        }
        
        std::cerr << "DEBUG: Received " << bytes_transferred << " bytes. Buffer size now: " 
                 << buffer_index << " + " << bytes_transferred << " = " 
                 << (buffer_index + bytes_transferred) << std::endl;
                 
        // Hex dump of received bytes
        std::cerr << "DEBUG: Raw bytes: ";
        for (size_t i = 0; i < bytes_transferred; i++) {
            std::cerr << std::hex << std::setw(2) << std::setfill('0') 
                     << static_cast<int>(read_buffer[buffer_index + i]) << " ";
        }
        std::cerr << std::dec << std::endl;
        
        // Increment buffer index
        buffer_index += bytes_transferred;
        
        // Search for start marker in buffer - only if we have enough bytes
        bool found_start = false;
        size_t start_pos = 0;
        
        if (buffer_index >= 2) {
            for (size_t i = 0; i <= buffer_index - 2; i++) {
                if (read_buffer[i] == CommConstants::START_MARKER_1 && 
                    read_buffer[i+1] == CommConstants::START_MARKER_2) {
                    
                    found_start = true;
                    start_pos = i;
                    
                    // If we found a start marker not at the beginning, shift buffer
                    if (i > 0) {
                        std::cerr << "DEBUG: Found start marker at offset " << i 
                                 << ", shifting buffer" << std::endl;
                        memmove(read_buffer.data(), read_buffer.data() + i, buffer_index - i);
                        buffer_index -= i;
                    }
                    
                    break;
                }
            }
        }
        
        // If no start marker found, keep reading
        if (!found_start) {
            std::cerr << "DEBUG: No start marker found, continuing to read" << std::endl;
            startAsyncRead();
            return;
        }
        
        // Check if we have enough data for a complete header
        if (buffer_index >= sizeof(RequestHeader)) {
            RequestHeader* header = reinterpret_cast<RequestHeader*>(read_buffer.data());
            
            // Verify start marker (redundant check, but good for safety)
            if (header->start_marker[0] != CommConstants::START_MARKER_1 || 
                header->start_marker[1] != CommConstants::START_MARKER_2) {
                std::cerr << "DEBUG: Invalid start marker in header, discarding" << std::endl;
                buffer_index = 0;
                startAsyncRead();
                return;
            }
            
            std::cerr << "DEBUG: Header fields - type: " << static_cast<int>(header->message_type)
                     << ", flags: 0x" << std::hex << header->flags 
                     << ", length: " << std::dec << header->length << std::endl;
            
            // Validate header length
            if (header->length > CommConstants::MAX_BUFFER_SIZE - sizeof(RequestHeader) - sizeof(EndMarker)) {
                std::cerr << "DEBUG: Invalid message length: " << header->length << std::endl;
                buffer_index = 0;
                startAsyncRead();
                return;
            }
            
            // Calculate total message length including header and end marker
            size_t total_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
            
            // Check if we have enough data for the complete message
            if (buffer_index >= total_length) {
                // Check end marker
                EndMarker* end = reinterpret_cast<EndMarker*>(
                    read_buffer.data() + sizeof(RequestHeader) + header->length);
                    
                if (end->marker[0] != CommConstants::END_MARKER_1 || 
                    end->marker[1] != CommConstants::END_MARKER_2) {
                    std::cerr << "DEBUG: Invalid end marker, discarding" << std::endl;
                    
                    // Look for another start marker after the current one
                    for (size_t i = 2; i <= buffer_index - 2; i++) {
                        if (read_buffer[i] == CommConstants::START_MARKER_1 && 
                            read_buffer[i+1] == CommConstants::START_MARKER_2) {
                            
                            std::cerr << "DEBUG: Found another start marker at offset " << i 
                                     << ", shifting buffer" << std::endl;
                            memmove(read_buffer.data(), read_buffer.data() + i, buffer_index - i);
                            buffer_index -= i;
                            startAsyncRead();
                            return;
                        }
                    }
                    
                    // No other start marker found, discard everything
                    buffer_index = 0;
                    startAsyncRead();
                    return;
                }
                
                // Process based on message type
                MessageType msg_type = static_cast<MessageType>(header->message_type);
                uint16_t flags = header->flags;
                
                switch (msg_type) {
                    case MessageType::Request:
                        std::cerr << "DEBUG: Received request message with flags: 0x" 
                                 << std::hex << flags << std::dec << std::endl;
                        
                        // Set response flags based on what the Brain is requesting
                        response_flags = flags;
                        
                        // Now we should send data (but only if the Brain actually requested something)
                        sendData = (flags != static_cast<uint16_t>(Brain::RequestFlag::NoData));
                        
                        // Update the last received time since we got a valid message
                        last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                            
                        // If we got a valid message, the device must be connected
                        connected = true;
                        
                        // Send acknowledgment (status 0 = success)
                        sendAcknowledgment(flags, 0x00);
                        break;
                        
                    case MessageType::Acknowledgment:
                        if (header->length == 1) {
                            // Get status byte
                            uint8_t* status_byte = read_buffer.data() + sizeof(RequestHeader);
                            uint8_t status = *status_byte;
                            
                            std::cerr << "DEBUG: Received acknowledgment message with flags: 0x" 
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
                                
                                // Update connected status and last received time
                                connected = true;
                                last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch()).count();
                                
                                std::cerr << "Acknowledgment received for flags: 0x" 
                                         << std::hex << flags 
                                         << ", status: " << static_cast<int>(status) 
                                         << std::dec << std::endl;
                            }
                        } else {
                            std::cerr << "DEBUG: Invalid acknowledgment message length" << std::endl;
                        }
                        break;
                        
                    case MessageType::Response:
                        std::cerr << "DEBUG: Received response message with flags: 0x" 
                                 << std::hex << flags << std::dec 
                                 << ", length: " << header->length << std::endl;
                        
                        // Process the received data
                        processReceivedData(flags, 
                                           read_buffer.data() + sizeof(RequestHeader), 
                                           header->length);
                        
                        // Update connection status and timestamp
                        connected = true;
                        last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        break;
                        
                    case MessageType::Handshake:
                        std::cerr << "DEBUG: Received handshake message" << std::endl;
                        
                        // Update connection status and timestamp
                        connected = true;
                        last_received_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        
                        // Respond to handshake with acknowledgment
                        sendAcknowledgment(0, 0x00);
                        break;
                        
                    default:
                        std::cerr << "DEBUG: Unknown message type: " 
                                 << static_cast<int>(header->message_type) << std::endl;
                        break;
                }
                
                // Move any remaining data to the start of buffer
                if (buffer_index > total_length) {
                    memmove(read_buffer.data(), read_buffer.data() + total_length, 
                           buffer_index - total_length);
                    buffer_index -= total_length;
                    
                    std::cerr << "DEBUG: Moved " << buffer_index << " remaining bytes to buffer start" << std::endl;
                } else {
                    buffer_index = 0;
                }
            } else {
                std::cerr << "DEBUG: Incomplete message, have " << buffer_index 
                         << " bytes, need " << total_length << " bytes" << std::endl;
            }
        }
    }
    
    // Start the next read operation - ALWAYS do this to maintain the read chain
    startAsyncRead();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Start an asynchronous write operation
bool BrainComm::startAsyncWrite(uint16_t flags, MessageType msg_type)
{
    if (!running || !connected || !serial_port || !serial_port->is_open()) {
        std::cerr << "DEBUG: Cannot start async write - connection not ready" << std::endl;
        return false;
    }
    
    // Prepare header for message
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(msg_type);
    header.flags = flags;
    header.length = 0;  // Default to no payload
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    // Create a sequence of buffers to send
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&header, sizeof(header)));
    
    // If we have a payload, it would be added here
    // buffers.push_back(boost::asio::buffer(payload_data, payload_size));
    
    // Add end marker
    buffers.push_back(boost::asio::buffer(&end_marker, sizeof(end_marker)));
    
    std::cerr << "DEBUG: Sending message - type: " << static_cast<int>(msg_type) 
             << ", flags: 0x" << std::hex << flags << std::dec << std::endl;
             
    // Start asynchronous write
    boost::asio::async_write(*serial_port, buffers,
        std::bind(&BrainComm::handleWrite, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  true)  // true indicates this is a request
    );
    
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////














// Handler for write completion
void BrainComm::handleWrite(const boost::system::error_code& ec, std::size_t bytes_transferred, bool is_request) {
    if (!running) {
        return;
    }
    
    if (ec) {
        if (ec != boost::asio::error::operation_aborted) {
            cerr << "Write error: " << ec.message() << endl;
            stats.logError(CommError::TransmissionFailed, "Write error: " + ec.message());
            stats.incrementTransmitStats(false);
            connected = false;
        }
    }
    else {
        // Write succeeded
        stats.incrementTransmitStats(true);
        
        if (is_request) {
            //cerr << "Request sent successfully" << endl;
            last_request_time = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now().time_since_epoch()).count();
        }
        else {
			if(bytes_transferred > 0)
			{
            	//cerr << "Data sent successfully" << endl;
            	last_send_time = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now().time_since_epoch()).count();
			}

			else
			{
				cerr << "No Data sent successfully" << endl;
			}
		
				
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::handleTimer(const boost::system::error_code& ec) {
    if (!running) {
        return;
    }
    
    if (!ec) {
        // Periodic connection check & reconnection attempts
        static uint32_t last_reconnect_attempt = 0;
        static uint32_t last_read_check = 0;
        auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
            
        // Attempt reconnection every 2 seconds if not connected
        if (!connected && (current_time - last_reconnect_attempt >= 2000)) {
            last_reconnect_attempt = current_time;
            std::cerr << "DEBUG: Not connected to Brain, attempting to reconnect..." << std::endl;
            
            if (reconnect()) {
                std::cerr << "DEBUG: Successfully reconnected to Brain" << std::endl;
            } else {
                std::cerr << "DEBUG: Reconnection attempt failed, will try again" << std::endl;
            }
        }
        
        // Check for read operation state every 5 seconds
        if (current_time - last_read_check >= 5000) {
            last_read_check = current_time;
            
            // Use read_in_progress to see if we have an active read
            if (connected && serial_port && serial_port->is_open() && !read_in_progress.load()) {
                std::cerr << "DEBUG: No active read operation detected, restarting read" << std::endl;
                buffer_index = 0;
                startAsyncRead();
            }
        }
        
        // Regular timer operations
        checkPendingRequests();
        
        // Send periodic data if enabled
        if (sendData) {
            if (current_time - last_send_time >= CommConstants::RESPONSE_UPDATE_PERIOD.count()) {
                sendResponse(response_flags);
            }
        }
        
        // Check for request timeouts
        checkRequestTimeout();
        
        // Restart the timer
        timer.expires_from_now(boost::posix_time::milliseconds(20));
        timer.async_wait(std::bind(&BrainComm::handleTimer, this, std::placeholders::_1));
    }
    else if (ec != boost::asio::error::operation_aborted) 
    {
        std::cerr << "Timer error: " << ec.message() << std::endl;
        
        // Restart timer with a slightly longer delay
        timer.expires_from_now(boost::posix_time::milliseconds(100));
        timer.async_wait(std::bind(&BrainComm::handleTimer, this, std::placeholders::_1));
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::checkPendingRequests() {
    std::lock_guard<std::mutex> lock(state_mutex);
    
    if (!request_in_progress && !pending_requests.empty()) {
        uint16_t flags = pending_requests.front();
        request_in_progress = true;
        
        // Create a copy of the flags to use after releasing the lock
        uint16_t flags_copy = flags;
        
        startAsyncWrite(flags_copy,MessageType::Request);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BrainComm::checkRequestTimeout() {
    if (!request_in_progress) {
        return;
    }
    
    auto now_ms = chrono::duration_cast<chrono::milliseconds>(
        chrono::steady_clock::now().time_since_epoch()).count();
        
    if (now_ms - last_request_time >= CommConstants::READ_TIMEOUT.count()) {
        std::lock_guard<std::mutex> lock(state_mutex);
        
        // Check retry count
        if (request_retry_count >= CommConstants::MAX_REQUEST_RETRIES) {
            // Max retries reached, log error and abandon request
            string error_msg = "Request timeout after " + 
                               to_string(CommConstants::MAX_REQUEST_RETRIES) + " retries";
            stats.logError(CommError::RequestTimeout, error_msg);
            
            if (!pending_requests.empty()) {
                cerr << "Request timed out after " << CommConstants::MAX_REQUEST_RETRIES 
                     << " retries (flags: 0x" << hex << pending_requests.front() 
                     << dec << ")" << endl;
                pending_requests.pop();
            }
            
            request_retry_count = 0;
            request_in_progress = false;
        }
        else {
            // Retry the request
            request_retry_count++;
            request_in_progress = false;  // Allow resend on next timer tick
            
            if (!pending_requests.empty()) {
                cerr << "Retrying request (flags: 0x" << hex << pending_requests.front() 
                     << dec << ", attempt: " << static_cast<int>(request_retry_count) 
                     << ")" << endl;
            }
        }
    }
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
    
    std::cerr << "DEBUG: Sending response - flags: 0x" << std::hex << flags 
             << std::dec << ", payload size: " << payload_size << std::endl;
             
    boost::asio::async_write(*serial_port, buffers,
        std::bind(&BrainComm::handleWrite, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  false)  // false indicates this is not a request
    );
    
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
    
    std::cerr << "DEBUG: Sending acknowledgment - flags: 0x" 
             << std::hex << flags << ", status: " << static_cast<int>(status) 
             << std::dec << std::endl;
             
    // Start asynchronous write
    boost::asio::async_write(*serial_port, buffers,
        std::bind(&BrainComm::handleWrite, this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  false)  // false indicates this is not a request
    );
    
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
            memcpy(&brain_battery, data + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            
            // cerr << "Received Brain battery level: " 
            //      << brain_battery << "%" << endl;
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
    return brain_battery;
}

} // namespace Brain