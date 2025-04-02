#include <Camera.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <GPS.h>
#include <PortDetector.h>
#include <IMU.h>
#include <iostream>
#include <BrainComm.h>
#include <RobotPosition.h>
#include <Position.h>
#include <UPS.h>
#include <random>
#include <chrono>

using namespace std;



void printIMUdata(IMU &imu)
{
     float ax, ay, az,                                           // Accelerometer data
           gx, gy, gz,                                           // Gyroscope data
           mx, my, mz,                                           // Magnetometer data
           temperature;                                          // Temperature data

    // Read all sensor data
    if (imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz, temperature)) 
    {
        std::cout << "--------------------------------------------" << std::endl;
        // Display IMU Data
        std::cout << "Accel (g):  X: " << ax << "  Y: " << ay << "  Z: " << az << std::endl;    // Accelerometer data in g
        std::cout << "Gyro (dps): X: " << gx << "  Y: " << gy << "  Z: " << gz << std::endl;    // Gyroscope data in degrees per second
        std::cout << "Mag (gauss): X: " << mx << "  Y: " << my << "  Z: " << mz << std::endl;   // Magnetometer data in gauss
        std::cout << "Temperature: " << temperature << " °C" << std::endl;                      // Temperature in Celsius
         std::cout << "--------------------------------------------" << std::endl;
    } 
    
}

void printUPSdata(UPS &ups)
{
    //Get voltage and current measurements
    float bus_voltage = ups.getBusVoltage_V();              // voltage on V- (load side)
    float shunt_voltage = ups.getShuntVoltage_mV() / 1000;  // voltage between V+ and V- across the shunt (in V)
    float current = ups.getCurrent_mA();                    // current in mA
    float power = ups.getPower_W();                         // power in W
    float percentage = ups.getBatteryPercentage();          // battery percentage


     std::cout << "--------------------------------------------" << std::endl;
    // Display UPS data 
    std::cout << "PSU Voltage:   " << (bus_voltage + shunt_voltage) << " V" << std::endl;
    std::cout << "Load Voltage:  " << bus_voltage << " V" << std::endl;
    std::cout << "Current:       " << (current / 1000.0f) << " A" << std::endl;
    std::cout << "Power:         " << power << " W" << std::endl;
    std::cout << "Percent:       " << percentage << "%" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
}

// void printPositionData(RobotPosition &pos)
// {

//     Position position = pos.getPosition();
//     float velocity = pos.getVelocity();
//     std::cout << "--------------------------------------------" << std::endl;
//     // Print current position data
//     std::cout << "Position: (" << position.x << ", " << position.y 
//                 << "), Heading: " << position.azimuth 
//                 << "°, Confidence: " << position.confidence 
//                 << ", Velocity: " << velocity << " m/s" << std::endl;
//     std::cout << "--------------------------------------------" << std::endl;

// }


int main() {

    UPS ups;
    ups.initialize();

    IMU imu;
    imu.initialize();
    imu.start();


    imu.calibrateGyroscope();
    imu.calibrateAccelerometer();
    imu.calibrateMagnetometer();
    sleep(1);

    boost::asio::io_service myService;
    Brain::BrainComm brain(myService);

    // // Initialize RobotPosition (manages GPS internally)
    // std::cout << "Initializing position tracking..." << std::endl;
    // RobotPosition robotPosition(brain, imu);
    // if (!robotPosition.initialize(myService)) {
    //     std::cerr << "Failed to initialize position tracking" << std::endl;
    //     return 1;
    // }

    // robotPosition.start();

    Camera camera;
    camera.start();
    
    Model model;
    ObjectDetection objdet;

 
    while (true) 
    {
    
        camera.preprocessFrames(model.inferInput);
        camera.getPointCloud();
        model.runInference();
        std::vector<DetectedObject> Det = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        brain.setJetsonBattery(ups.getBatteryPercentage());

        
        // printIMUdata(imu);
        //  printUPSdata(ups);
        // printPositionData(robotPosition);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    brain.stop();
    
    
    return 0;
}




