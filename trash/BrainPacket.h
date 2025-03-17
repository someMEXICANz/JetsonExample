
#ifndef BRAIN_PACKET_H
#define BRAIN_PACKET_H

#include <vector>
#include <cstdint>
#include <iostream>

class BrainPacket {
public:
    // Direction flag
    static const uint16_t RESPONSE = 0x8000;
    
    // Jetson → Brain Requests
    static const uint16_t REQUEST_VEX_LINK_POS  = 0x0001;
    static const uint16_t REQUEST_BATTERY_LEVEL = 0x0002;
    static const uint16_t REQUEST_GPS_INIT      = 0x0004;
    
    // Brain → Jetson Requests
    static const uint16_t BRAIN_REQUEST_MACRO_STATE = 0x0100;
    static const uint16_t BRAIN_REQUEST_CONTROL_VDC = 0x0200;

    // Constructor
    BrainPacket(uint16_t flags = 0, bool isResponse = false);

    // Convert to/from byte vector
    std::vector<uint8_t> toBytes() const;
    static BrainPacket fromBytes(const std::vector<uint8_t>& data);

    // Getters/Setters
    uint16_t getFlags() const { return flags_; }
    void setFlags(uint16_t flags) { flags_ = flags; }
    bool isResponse() const { return (flags_ & RESPONSE) != 0; }

private:
    uint16_t flags_;
};

#endif // BRAIN_PACKET_H