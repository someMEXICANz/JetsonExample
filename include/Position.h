#ifndef POSITION_H
#define POSITION_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <iomanip>

class Position {
public:
    float x, y, z;
    float azimuth, elevation, rotation;
    std::chrono::system_clock::time_point timestamp;
   

    Position();
    Position(float x, float y, float z, float azimuth, float elevation, float rotation);
    
    void printPosition() const;
    bool isValid() const;
    float distanceTo(const Position& other) const;

   
};

#endif
