#include <Position.h>

Position::Position() 
    : x(0), y(0), z(0), azimuth(0), elevation(0), rotation(0),
      timestamp(std::chrono::system_clock::now()) {}

Position::Position(float x, float y, float z, float azimuth, float elevation, float rotation)
    : x(x), y(y), z(z), azimuth(azimuth), elevation(elevation), rotation(rotation),
      timestamp(std::chrono::system_clock::now()) {}

bool Position::isValid() const {
    const float MAX_COORDINATE = 100.0f;
    return x >= -MAX_COORDINATE && x <= MAX_COORDINATE &&
           y >= -MAX_COORDINATE && y <= MAX_COORDINATE &&
           z >= -MAX_COORDINATE && z <= MAX_COORDINATE &&
           azimuth >= 0.0f && azimuth < 360.0f &&
           elevation >= -90.0f && elevation <= 90.0f &&
           rotation >= -180.0f && rotation <= 180.0f;
}

float Position::distanceTo(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    float dz = z - other.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void Position::printPosition() const {
    std::cout << "Position (x, y, z): (" << std::fixed << std::setprecision(3) 
              << x << ", " << y << ", " << z << ") meters\n"
              << "Orientation (azimuth, elevation, rotation): (" 
              << std::setprecision(1) << azimuth << "°, " 
              << elevation << "°, " << rotation << "°)\n";
}




















// Position::Position() 
//     : status(0), x(0), y(0), z(0), azimuth(0), elevation(0), rotation(0),
//       timestamp(std::chrono::system_clock::now()) 
// {}

// Position::Position(uint32_t status, float x, float y, float z, float azimuth, float elevation, float rotation)
//     : status(status), x(x), y(y), z(z), azimuth(azimuth), elevation(elevation), rotation(rotation),
//       timestamp(std::chrono::system_clock::now()) 
// {}

// bool Position::isValid() const {
//     // Check if position has reasonable values
//     const float MAX_COORDINATE = 100.0f;  // meters
//     const float MIN_COORDINATE = -100.0f; // meters
    
//     return x >= MIN_COORDINATE && x <= MAX_COORDINATE &&
//            y >= MIN_COORDINATE && y <= MAX_COORDINATE &&
//            z >= MIN_COORDINATE && z <= MAX_COORDINATE &&
//            azimuth >= 0.0f && azimuth < 360.0f &&
//            elevation >= -90.0f && elevation <= 90.0f &&
//            rotation >= -180.0f && rotation <= 180.0f;
// }

// float Position::distanceTo(const Position& other) const {
//     float dx = x - other.x;
//     float dy = y - other.y;
//     float dz = z - other.z;
//     return std::sqrt(dx*dx + dy*dy + dz*dz);
// }


// void Position::printPosition() const {
//     // Print coordinate values
//     std::cout << "Coordinates (x, y, z): (" 
//               << std::fixed << std::setprecision(3) << x << ", " 
//               << y << ", " 
//               << z << ") meters" << std::endl;
    
//     // Print orientation angles
//     std::cout << "Orientation (azimuth, elevation, rotation): (" 
//               << std::fixed << std::setprecision(1) << azimuth << "°, " 
//               << elevation << "°, " 
//               << rotation << "°)" << std::endl;
    
//     // Convert timestamp to readable format
//     auto time = std::chrono::system_clock::to_time_t(timestamp);
//     std::cout << "Timestamp: " << std::ctime(&time);
    
//     // Print status
//     std::cout << "Status: " << getStatusString() << std::endl;
//     std::cout << "Status (hex): " << getStatusHex() << std::endl;
// }




// std::string Position::getStatusString() const {
//     std::vector<std::string> active_flags;
    
//     if (status & STATUS_CONNECTED)    active_flags.push_back("CONNECTED");
//     if (status & STATUS_NODOTS)       active_flags.push_back("NO_DOTS");
//     if (status & STATUS_NORAWBITS)    active_flags.push_back("NO_RAW_BITS");
//     if (status & STATUS_NOGROUPS)     active_flags.push_back("NO_GROUPS");
//     if (status & STATUS_NOBITS)       active_flags.push_back("NO_BITS");
//     if (status & STATUS_PIXELERROR)   active_flags.push_back("PIXEL_ERROR");
//     if (status & STATUS_SOLVER)       active_flags.push_back("SOLVER");
//     if (status & STATUS_ANGLEJUMP)    active_flags.push_back("ANGLE_JUMP");
//     if (status & STATUS_POSJUMP)      active_flags.push_back("POSITION_JUMP");
//     if (status & STATUS_NOSOLUTION)   active_flags.push_back("NO_SOLUTION");
//     if (status & STATUS_KALMAN_EST)   active_flags.push_back("KALMAN_ESTIMATE");

//     if (active_flags.empty()) {
//         return "NO_FLAGS_SET";
//     }

//     std::string result;
//     for (size_t i = 0; i < active_flags.size(); ++i) {
//         result += active_flags[i];
//         if (i < active_flags.size() - 1) {
//             result += " | ";
//         }
//     }
//     return result;
// }

// // For raw hex value
// std::string Position::getStatusHex() const {
//     std::stringstream ss;
//     ss << "0x" << std::hex << std::uppercase << status;
//     return ss.str();
// }