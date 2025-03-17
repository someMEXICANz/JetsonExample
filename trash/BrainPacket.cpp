#include "BrainPacket.h"


BrainPacket::BrainPacket(uint16_t flags, bool isResponse) {
    flags_ = flags | (isResponse ? RESPONSE : 0);
}

std::vector<uint8_t> BrainPacket::toBytes() const {
    return { 0xAA, 0x55, static_cast<uint8_t>(flags_), static_cast<uint8_t>(flags_ >> 8) };
}

BrainPacket BrainPacket::fromBytes(const std::vector<uint8_t>& data) {
    if (data.size() < 4 || data[0] != 0xAA || data[1] != 0x55) {
        std::cerr << "Warning: Invalid packet received!" << std::endl;
        return BrainPacket(0x0000, false); // Default invalid packet
    }

    uint16_t flags = data[2] | (data[3] << 8);
    return BrainPacket(flags, (flags & BrainPacket::RESPONSE) != 0);
}
