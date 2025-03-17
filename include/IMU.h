#ifndef IMU_H
#define IMU_H

#include <string>
#include <cstdint>

class IMU {
public:
    // Constructor with default I2C bus path
    IMU(const std::string& i2cBus = "/dev/i2c-1");
    
    // Destructor
    ~IMU();

    // Initialize the IMU sensors
    bool initialize();
    
    // Read sensor data
    bool readAccelerometer(float& ax, float& ay, float& az);
    bool readGyroscope(float& gx, float& gy, float& gz);
    bool readMagnetometer(float& mx, float& my, float& mz);
    bool readTemperature(float& temp);
    
    // Read all sensor data at once
    bool readAll(float& ax, float& ay, float& az,
                float& gx, float& gy, float& gz,
                float& mx, float& my, float& mz,
                float& temp);
    
    // Calibration methods
    bool calibrateAccelerometer();
    bool calibrateGyroscope();
    bool calibrateMagnetometer();
    
    // Get the last error message
    std::string getLastError() const;

private:
    // I2C device properties
    std::string _i2cBus;
    int _i2cFd;
    std::string _lastError;
    
    // Sensor addresses
    static constexpr uint8_t LSM6DS3_ADDR = 0x6A;
    static constexpr uint8_t LIS3MDL_ADDR = 0x1C;
    
    // Calibration offsets
    float _accelOffsetX, _accelOffsetY, _accelOffsetZ;
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;
    float _magOffsetX, _magOffsetY, _magOffsetZ;
    
    // Scale factors (can be adjusted based on configuration)
    float _accelScale;  // g per LSB
    float _gyroScale;   // dps per LSB
    float _magScale;    // gauss per LSB
    
    // Helper methods for I2C communication
    bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t data);
    bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length);
    
    // Set error message
    void setError(const std::string& error);
};

#endif // IMU_H

























// #ifndef IMU_HPP
// #define IMU_HPP

// #include <cstdint>
// #include <string>

// class IMU {
// public:
//     IMU(const std::string& i2cBus = "/dev/i2c-1");
//     ~IMU();

//     bool initialize();
    
//     // Read sensor data
//     bool readAccelerometer(float& ax, float& ay, float& az);
//     bool readGyroscope(float& gx, float& gy, float& gz);
//     bool readMagnetometer(float& mx, float& my, float& mz);

// private:
//     int i2c_fd; // File descriptor for I2C device
//     std::string i2cBus;

//     static constexpr uint8_t LSM6DS3_ADDR = 0x6A;
//     static constexpr uint8_t LIS3MDL_ADDR = 0x1C;

//     // I2C helper functions
//     bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t data);
//     bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length);
// };

// #endif // IMU_HPP

























