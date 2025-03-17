#include "IMU.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

// LSM6DS3TR-C registers
#define LSM6DS3_WHO_AM_I          0x0F
#define LSM6DS3_CTRL1_XL          0x10  // Accelerometer control register
#define LSM6DS3_CTRL2_G           0x11  // Gyroscope control register
#define LSM6DS3_CTRL3_C           0x12  // Control register 3
#define LSM6DS3_OUT_TEMP_L        0x20  // Temperature LSB
#define LSM6DS3_OUT_TEMP_H        0x21  // Temperature MSB
#define LSM6DS3_OUTX_L_G          0x22  // Gyroscope X-axis LSB
#define LSM6DS3_OUTX_H_G          0x23  // Gyroscope X-axis MSB
#define LSM6DS3_OUTY_L_G          0x24  // Gyroscope Y-axis LSB
#define LSM6DS3_OUTY_H_G          0x25  // Gyroscope Y-axis MSB
#define LSM6DS3_OUTZ_L_G          0x26  // Gyroscope Z-axis LSB
#define LSM6DS3_OUTZ_H_G          0x27  // Gyroscope Z-axis MSB
#define LSM6DS3_OUTX_L_XL         0x28  // Accelerometer X-axis LSB
#define LSM6DS3_OUTX_H_XL         0x29  // Accelerometer X-axis MSB
#define LSM6DS3_OUTY_L_XL         0x2A  // Accelerometer Y-axis LSB
#define LSM6DS3_OUTY_H_XL         0x2B  // Accelerometer Y-axis MSB
#define LSM6DS3_OUTZ_L_XL         0x2C  // Accelerometer Z-axis LSB
#define LSM6DS3_OUTZ_H_XL         0x2D  // Accelerometer Z-axis MSB

// LIS3MDL registers
#define LIS3MDL_WHO_AM_I          0x0F
#define LIS3MDL_CTRL_REG1         0x20  // Control register 1
#define LIS3MDL_CTRL_REG2         0x21  // Control register 2
#define LIS3MDL_CTRL_REG3         0x22  // Control register 3
#define LIS3MDL_CTRL_REG4         0x23  // Control register 4
#define LIS3MDL_OUT_TEMP_L        0x2E  // Temperature LSB
#define LIS3MDL_OUT_TEMP_H        0x2F  // Temperature MSB
#define LIS3MDL_OUT_X_L           0x28  // X-axis LSB
#define LIS3MDL_OUT_X_H           0x29  // X-axis MSB
#define LIS3MDL_OUT_Y_L           0x2A  // Y-axis LSB
#define LIS3MDL_OUT_Y_H           0x2B  // Y-axis MSB
#define LIS3MDL_OUT_Z_L           0x2C  // Z-axis LSB
#define LIS3MDL_OUT_Z_H           0x2D  // Z-axis MSB

// Constructor
IMU::IMU(const std::string& i2cBus) 
    : _i2cBus(i2cBus), 
      _i2cFd(-1),
      _accelOffsetX(0.0f), _accelOffsetY(0.0f), _accelOffsetZ(0.0f),
      _gyroOffsetX(0.0f), _gyroOffsetY(0.0f), _gyroOffsetZ(0.0f),
      _magOffsetX(0.0f), _magOffsetY(0.0f), _magOffsetZ(0.0f),
      _accelScale(0.061f / 1000.0f),  // ±2g range: 0.061 mg/LSB
      _gyroScale(70.0f / 1000.0f),    // ±2000 dps range: 70 mdps/LSB
      _magScale(0.58f / 1000.0f)      // ±16 gauss range: 0.58 mgauss/LSB
{
}

// Destructor
IMU::~IMU() {
    if (_i2cFd >= 0) {
        close(_i2cFd);
        _i2cFd = -1;
    }
}

// Initialize the IMU sensors
bool IMU::initialize() {
    // Open I2C device
    _i2cFd = open(_i2cBus.c_str(), O_RDWR);
    if (_i2cFd < 0) {
        setError("Failed to open I2C bus: " + _i2cBus);
        return false;
    }

    // Initialize accelerometer and gyroscope
    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL1_XL, 0x60)) {  // 208Hz, ±2g
        setError("Failed to configure accelerometer");
        return false;
    }

    if (!writeByte(LSM6DS3_ADDR, LSM6DS3_CTRL2_G, 0x6C)) {  // 208Hz, ±2000dps
        setError("Failed to configure gyroscope");
        return false;
    }

    // Initialize magnetometer
    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x7C)) {  // Ultra-high performance, 80Hz
        setError("Failed to configure magnetometer");
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x40)) {  // ±16 gauss
        setError("Failed to set magnetometer range");
        return false;
    }

    if (!writeByte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)) {  // Continuous mode
        setError("Failed to set magnetometer mode");
        return false;
    }

    return true;
}

// Read accelerometer data
bool IMU::readAccelerometer(float& ax, float& ay, float& az) {
    uint8_t buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, buffer, 6)) {
        setError("Failed to read accelerometer data");
        return false;
    }
    
    // Combine high and low bytes (LSM6DS3 is little-endian)
    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];
    
    // Convert to g units using scale factor and apply calibration offset
    ax = (rawX * _accelScale) - _accelOffsetX;
    ay = (rawY * _accelScale) - _accelOffsetY;
    az = (rawZ * _accelScale) - _accelOffsetZ;
    
    return true;
}

// Read gyroscope data
bool IMU::readGyroscope(float& gx, float& gy, float& gz) {
    uint8_t buffer[6];
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, buffer, 6)) {
        setError("Failed to read gyroscope data");
        return false;
    }
    
    // Combine high and low bytes
    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];
    
    // Convert to degrees per second and apply calibration offset
    gx = (rawX * _gyroScale) - _gyroOffsetX;
    gy = (rawY * _gyroScale) - _gyroOffsetY;
    gz = (rawZ * _gyroScale) - _gyroOffsetZ;
    
    return true;
}

// Read magnetometer data
bool IMU::readMagnetometer(float& mx, float& my, float& mz) {
    uint8_t buffer[6];
    if (!readBytes(LIS3MDL_ADDR, LIS3MDL_OUT_X_L, buffer, 6)) {
        setError("Failed to read magnetometer data");
        return false;
    }
    
    // Combine high and low bytes (LIS3MDL is little-endian)
    int16_t rawX = (buffer[1] << 8) | buffer[0];
    int16_t rawY = (buffer[3] << 8) | buffer[2];
    int16_t rawZ = (buffer[5] << 8) | buffer[4];
    
    // Convert to gauss using scale factor and apply calibration offset
    mx = (rawX * _magScale) - _magOffsetX;
    my = (rawY * _magScale) - _magOffsetY;
    mz = (rawZ * _magScale) - _magOffsetZ;
    
    return true;
}

// Read temperature data
bool IMU::readTemperature(float& temp) {
    uint8_t buffer[2];
    
    // Read from LSM6DS3 (could also read from LIS3MDL)
    if (!readBytes(LSM6DS3_ADDR, LSM6DS3_OUT_TEMP_L, buffer, 2)) {
        setError("Failed to read temperature data");
        return false;
    }
    
    // Combine high and low bytes
    int16_t rawTemp = (buffer[1] << 8) | buffer[0];
    
    // Convert to Celsius (according to datasheet)
    temp = static_cast<float>(rawTemp) / 16.0f + 25.0f;
    
    return true;
}

// Read all sensor data at once
bool IMU::readAll(float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz,
                 float& mx, float& my, float& mz,
                 float& temp) {
    
    if (!readAccelerometer(ax, ay, az)) {
        return false;
    }
    
    if (!readGyroscope(gx, gy, gz)) {
        return false;
    }
    
    if (!readMagnetometer(mx, my, mz)) {
        return false;
    }
    
    if (!readTemperature(temp)) {
        return false;
    }
    
    return true;
}

// Calibrate accelerometer
bool IMU::calibrateAccelerometer() {
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readAccelerometer(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        usleep(10000);  // 10ms delay
    }
    
    _accelOffsetX = sumX / numSamples;
    _accelOffsetY = sumY / numSamples;
    _accelOffsetZ = (sumZ / numSamples) - 1.0f;  // Remove gravity (1g)
    
    return true;
}

// Calibrate gyroscope
bool IMU::calibrateGyroscope() {
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readGyroscope(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        usleep(10000);  // 10ms delay
    }
    
    _gyroOffsetX = sumX / numSamples;
    _gyroOffsetY = sumY / numSamples;
    _gyroOffsetZ = sumZ / numSamples;
    
    return true;
}

// Calibrate magnetometer
bool IMU::calibrateMagnetometer() {
    const int numSamples = 100;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    
    for (int i = 0; i < numSamples; ++i) {
        float x, y, z;
        if (!readMagnetometer(x, y, z)) {
            return false;
        }
        
        sumX += x;
        sumY += y;
        sumZ += z;
        
        usleep(10000);  // 10ms delay
    }
    
    _magOffsetX = sumX / numSamples;
    _magOffsetY = sumY / numSamples;
    _magOffsetZ = sumZ / numSamples;
    
    return true;
}

// Get the last error message
std::string IMU::getLastError() const {
    return _lastError;
}

// Write to a register
bool IMU::writeByte(uint8_t devAddr, uint8_t reg, uint8_t data) {
    if (ioctl(_i2cFd, I2C_SLAVE, devAddr) < 0) {
        setError("Failed to set I2C slave address");
        return false;
    }
    
    uint8_t buffer[2] = {reg, data};
    if (write(_i2cFd, buffer, 2) != 2) {
        setError("Failed to write to register");
        return false;
    }
    
    return true;
}

// Read from registers
bool IMU::readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
    if (ioctl(_i2cFd, I2C_SLAVE, devAddr) < 0) {
        setError("Failed to set I2C slave address");
        return false;
    }
    
    if (write(_i2cFd, &reg, 1) != 1) {
        setError("Failed to write register address");
        return false;
    }
    
    if (read(_i2cFd, buffer, length) != static_cast<ssize_t>(length)) {
        setError("Failed to read data");
        return false;
    }
    
    return true;
}

// Set error message
void IMU::setError(const std::string& error) {
    _lastError = error;
    std::cerr << "IMU Error: " << error << std::endl;
}






































































// #include "IMU.h"
// #include <iostream>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <cstring>

// IMU::IMU(const std::string& i2cBus) : i2cBus(i2cBus), i2c_fd(-1) {}

// IMU::~IMU() {
//     if (i2c_fd >= 0) {
//         close(i2c_fd);
//     }
// }

// bool IMU::initialize() {
//     i2c_fd = open(i2cBus.c_str(), O_RDWR);
//     if (i2c_fd < 0) {
//         std::cerr << "Failed to open I2C bus: " << i2cBus << std::endl;
//         return false;
//     }

//     // Enable accelerometer and gyroscope (CTRL1_XL and CTRL2_G)
//     return writeByte(LSM6DS3_ADDR, 0x10, 0x60) && // 208Hz, ±2g for accel
//            writeByte(LSM6DS3_ADDR, 0x11, 0x60) && // 208Hz, 2000 dps for gyro
//            writeByte(LIS3MDL_ADDR, 0x20, 0x7C);   // Magnetometer: Ultra-high performance
// }

// bool IMU::readAccelerometer(float& ax, float& ay, float& az) {
//     uint8_t buffer[6];
//     if (!readBytes(LSM6DS3_ADDR, 0x28, buffer, 6)) return false;

//     int16_t raw_x = (buffer[1] << 8) | buffer[0];
//     int16_t raw_y = (buffer[3] << 8) | buffer[2];
//     int16_t raw_z = (buffer[5] << 8) | buffer[4];

//     ax = raw_x * 0.061f / 1000.0f; // Convert to g
//     ay = raw_y * 0.061f / 1000.0f;
//     az = raw_z * 0.061f / 1000.0f;
//     return true;
// }

// bool IMU::readGyroscope(float& gx, float& gy, float& gz) {
//     uint8_t buffer[6];
//     if (!readBytes(LSM6DS3_ADDR, 0x22, buffer, 6)) return false;

//     int16_t raw_x = (buffer[1] << 8) | buffer[0];
//     int16_t raw_y = (buffer[3] << 8) | buffer[2];
//     int16_t raw_z = (buffer[5] << 8) | buffer[4];

//     gx = raw_x * 70.0f / 1000.0f; // Convert to dps
//     gy = raw_y * 70.0f / 1000.0f;
//     gz = raw_z * 70.0f / 1000.0f;
//     return true;
// }

// bool IMU::readMagnetometer(float& mx, float& my, float& mz) {
//     uint8_t buffer[6];
//     if (!readBytes(LIS3MDL_ADDR, 0x28, buffer, 6)) return false;

//     int16_t raw_x = (buffer[1] << 8) | buffer[0];
//     int16_t raw_y = (buffer[3] << 8) | buffer[2];
//     int16_t raw_z = (buffer[5] << 8) | buffer[4];

//     mx = raw_x * 0.58f; // Convert to uT
//     my = raw_y * 0.58f;
//     mz = raw_z * 0.58f;
//     return true;
// }

// bool IMU::writeByte(uint8_t devAddr, uint8_t reg, uint8_t data) {
//     uint8_t buffer[2] = { reg, data };
//     if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) return false;
//     return write(i2c_fd, buffer, 2) == 2;
// }

// bool IMU::readBytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, size_t length) {
//     if (ioctl(i2c_fd, I2C_SLAVE, devAddr) < 0) return false;
//     if (write(i2c_fd, &reg, 1) != 1) return false;
//     return read(i2c_fd, buffer, length) == (ssize_t)length;
// }
























































