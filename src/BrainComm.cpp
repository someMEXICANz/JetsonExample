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
    , request_in_progress(false)
    , request_retry_count(0)
    , request_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , response_flags(static_cast<uint16_t>(RequestFlag::NoData))
    , sendData(false)
    , last_send_time(0)
    , last_received_time(0)
    , last_request_time(0)
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
        std::cerr << "Failed to establish initial connection" << std::endl;
        // Continue anyway, threads will handle reconnection attempts
    }
    
    // Set running flag before starting threads
    running = true;
    buffer_index = 0;
    
    // Start the read and write threads
    try {
        read_thread = std::make_unique<std::thread>(&BrainComm::readLoop, this);
        write_thread = std::make_unique<std::thread>(&BrainComm::writeLoop, this);
    } catch (const std::exception& e) {
        std::cerr << "Failed to start threads: " << e.what() << std::endl;
        running = false;
        return false;
    }
    
    return true;
}

void BrainComm::stop() {
    // Set running to false to signal threads to exit
    running = false;
    
    // Signal the write condition variable to wake up the write thread
    write_condition.notify_all();
    
    // Cancel any pending operations on the serial port
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->cancel();
        } catch (...) {}
    }
    
    // Wait for threads to finish
    if (read_thread && read_thread->joinable()) {
        read_thread->join();
    }
    
    if (write_thread && write_thread->joinable()) {
        write_thread->join();
    }
    
    // Clean up resources
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->close();
        } catch (...) {}
    }
    
    connected = false;
}

bool BrainComm::restart() {
    stop();
    return start();
}




bool BrainComm::reconnect() {
    // Acquire the state mutex to prevent concurrent access to shared state
    std::lock_guard<std::mutex> lock(state_mutex);
    
    // Close the existing serial port if it's open
    if (serial_port && serial_port->is_open()) {
        try {
            serial_port->cancel();
            serial_port->close();
        } catch (const std::exception& e) {
            std::cerr << "Error closing serial port: " << e.what() << std::endl;
            // Continue anyway - we want to try reopening
        }
    }
    
    // Reset communication state
    while (!pending_requests.empty()) pending_requests.pop();
    request_in_progress = false;
    request_retry_count = 0;
    sendData = false;
    buffer_index = 0;  // Reset buffer
    
    // Attempt to initialize the port
    if (initializePort()) {
        std::cout << "Successfully reconnected to port " << port << std::endl;
        stats.recordConnectionStatus(true);
        return true;
    } else {
        std::cerr << "Failed to reconnect to port " << port << std::endl;
        return false;
    }
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



bool BrainComm::updateRequests(uint16_t flags) 
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pending_requests.push(flags);
    cerr << "Request queued with flags: 0x" << hex << flags << dec << endl;
    return true;
}


































// Read thread function
void BrainComm::readLoop() {
    std::cout << "Read thread started" << std::endl;
    
    // Buffer for reading
    std::vector<uint8_t> local_buffer(CommConstants::MAX_BUFFER_SIZE);
    size_t local_buffer_index = 0;
    
    while (running) {
        try {
            // Check if we're connected
            if (!connected) {
                // Attempt to reconnect if not connected
                std::lock_guard<std::mutex> lock(state_mutex);
                if (reconnect()) {
                    std::cout << "Successfully reconnected in read thread" << std::endl;
                } else {
                    // Sleep briefly before next attempt
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }
            
            // Attempt to read data with timeout
            if (serial_port && serial_port->is_open()) {
                // Set up a timer for timeout
                boost::asio::deadline_timer timer(io_service);
                timer.expires_from_now(boost::posix_time::milliseconds(CommConstants::READ_TIMEOUT.count()));
                
                boost::system::error_code ec;
                size_t bytes_read = 0;
                
                // Start asynchronous read
                serial_port->async_read_some(
                    boost::asio::buffer(&local_buffer[local_buffer_index], 1),
                    [&](const boost::system::error_code& error, size_t bytes) {
                        ec = error;
                        bytes_read = bytes;
                        timer.cancel(); // Cancel the timer when read completes
                    }
                );
                
                // Start timer handler
                timer.async_wait([&](const boost::system::error_code& error) {
                    if (!error) {
                        // Timeout occurred
                        serial_port->cancel(); // Cancel the ongoing read
                    }
                });
                
                // Run the io_service until one operation completes
                io_service.reset();
                io_service.run_one();
                
                // Process read result
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        // Read error occurred
                        std::cerr << "Read error: " << ec.message() << std::endl;
                        stats.logError(CommError::ReadTimeout, "Read error: " + ec.message());
                        stats.incrementReceiveStats(false);
                        
                        // Only mark as disconnected for serious errors
                        if (ec == boost::asio::error::bad_descriptor || 
                            ec == boost::asio::error::broken_pipe ||
                            ec == boost::asio::error::not_connected) {
                            connected = false;
                        }
                    }
                    // If operation was canceled by timer, we just continue
                    continue;
                }
                
                // Successfully read data
                if (bytes_read > 0) {
                    // Safety check for buffer overflow
                    if (local_buffer_index + bytes_read >= CommConstants::MAX_BUFFER_SIZE) {
                        std::cerr << "Buffer would overflow, resetting" << std::endl;
                        local_buffer_index = 0;
                    }
                    
                    // Increment buffer index
                    local_buffer_index += bytes_read;
                    
                    // Process the buffer for complete messages
                    size_t processed_bytes = 0;
                    while (processed_bytes < local_buffer_index) {
                        // Look for start marker
                        bool found_start = false;
                        size_t start_pos = 0;
                        
                        for (size_t i = processed_bytes; i <= local_buffer_index - 2; i++) {
                            if (local_buffer[i] == CommConstants::START_MARKER_1 && 
                                local_buffer[i+1] == CommConstants::START_MARKER_2) {
                                found_start = true;
                                start_pos = i;
                                break;
                            }
                        }
                        
                        if (!found_start) {
                            // No start marker found, discard processed bytes
                            if (processed_bytes > 0) {
                                memmove(local_buffer.data(), local_buffer.data() + processed_bytes, 
                                       local_buffer_index - processed_bytes);
                                local_buffer_index -= processed_bytes;
                            }
                            break;
                        }
                        
                        // If start marker is not at the beginning, shift buffer
                        if (start_pos > processed_bytes) {
                            memmove(local_buffer.data() + processed_bytes, 
                                   local_buffer.data() + start_pos, 
                                   local_buffer_index - start_pos);
                            local_buffer_index -= (start_pos - processed_bytes);
                            start_pos = processed_bytes;
                        }
                        
                        // Check if we have enough data for a complete header
                        if (start_pos + sizeof(RequestHeader) <= local_buffer_index) {
                            RequestHeader* header = reinterpret_cast<RequestHeader*>(
                                local_buffer.data() + start_pos);
                            
                            // Calculate total message length
                            size_t total_length = sizeof(RequestHeader) + header->length + sizeof(EndMarker);
                            
                            // Check if we have the complete message
                            if (start_pos + total_length <= local_buffer_index) {
                                // Process the complete message
                                if (handleMessage(local_buffer.data() + start_pos, total_length)) {
                                    // Message handled successfully
                                    processed_bytes = start_pos + total_length;
                                } else {
                                    // Message handling failed, move past this start marker
                                    processed_bytes = start_pos + 2;
                                }
                            } else {
                                // Incomplete message, need more data
                                break;
                            }
                        } else {
                            // Not enough data for header, need to read more
                            break;
                        }
                    }
                    
                    // Move any remaining unprocessed data to the beginning of the buffer
                    if (processed_bytes > 0 && processed_bytes < local_buffer_index) {
                        memmove(local_buffer.data(), local_buffer.data() + processed_bytes,
                               local_buffer_index - processed_bytes);
                        local_buffer_index -= processed_bytes;
                    } else if (processed_bytes == local_buffer_index) {
                        local_buffer_index = 0;
                    }
                }
            } else {
                // Not connected or port not open
                connected = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception in read thread: " << e.what() << std::endl;
            connected = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "Read thread stopped" << std::endl;
}

// Write thread function
void BrainComm::writeLoop() {
    std::cout << "Write thread started" << std::endl;
    
    // For tracking periodic operations
    uint32_t last_reconnect_attempt = 0;
    uint32_t last_check_time = 0;
    
    while (running) {
        try {
            auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            
            // Check connection status
            if (!connected) {
                if (current_time - last_reconnect_attempt >= 1000) {
                    last_reconnect_attempt = current_time;
                    std::lock_guard<std::mutex> lock(state_mutex);
                    if (reconnect()) {
                        std::cout << "Successfully reconnected in write thread" << std::endl;
                    }
                }
                
                // Wait a bit before next iteration if not connected
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            
            // Check for pending requests
            bool request_sent = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                if (!request_in_progress && !pending_requests.empty()) {
                    uint16_t flags = pending_requests.front();
                    request_in_progress = true;
                    
                    // Release the lock before sending
                    state_mutex.unlock();
                    
                    if (sendRequest(flags)) {
                        last_request_time = current_time;
                        request_sent = true;
                    } else {
                        // Request failed
                        state_mutex.lock();
                        request_in_progress = false;
                    }
                }
            }
            
            // Check for request timeouts
            if (!request_sent) {
                checkRequestTimeout();
            }
            
            // Send periodic data if enabled
            if (sendData && current_time - last_send_time >= CommConstants::RESPONSE_UPDATE_PERIOD.count()) {
                sendResponse(response_flags);
            }
            
            // Wait for a short time or until signaled
            {
                std::unique_lock<std::mutex> lock(write_mutex);
                write_condition.wait_for(lock, std::chrono::milliseconds(20));
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Exception in write thread: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "Write thread stopped" << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool BrainComm::sendRequest(uint16_t flags) {
    if (!running || !connected || !serial_port || !serial_port->is_open()) {
        std::cerr << "Cannot send request - connection not ready" << std::endl;
        return false;
    }
    
    // Prepare header for message
    RequestHeader header;
    header.start_marker[0] = CommConstants::START_MARKER_1;
    header.start_marker[1] = CommConstants::START_MARKER_2;
    header.message_type = static_cast<uint8_t>(MessageType::Request);
    header.flags = flags;
    header.length = 0;  // No payload for requests
    
    // Set up end marker
    EndMarker end_marker;
    end_marker.marker[0] = CommConstants::END_MARKER_1;
    end_marker.marker[1] = CommConstants::END_MARKER_2;
    
    try {
        // Use a synchronous write for simplicity in the dedicated thread model
        boost::system::error_code ec;
        
        // Write header
        boost::asio::write(*serial_port, boost::asio::buffer(&header, sizeof(header)), ec);
        if (ec) throw boost::system::system_error(ec);
        
        // Write end marker
        boost::asio::write(*serial_port, boost::asio::buffer(&end_marker, sizeof(end_marker)), ec);
        if (ec) throw boost::system::system_error(ec);
        
        std::cerr << "Request sent with flags: 0x" << std::hex << flags << std::dec << std::endl;
        stats.incrementTransmitStats(true);
        
        return true;
    }
    catch (const std::exception& e) {
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
                connected = true;
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
                    connected = true;
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
            std::cerr << "Received response message with flags: 0x" 
                     << std::hex << flags << std::dec 
                     << ", length: " << header->length << std::endl;
            
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
                connected = true;
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
                connected = true;
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