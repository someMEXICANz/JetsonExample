#include <Camera.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <GPS.h>
#include <PortDetector.h>
#include <IMU.h>
#include <iostream>
#include <BrainComm.h>
//#include <CommsMonitor.h>
#include <RobotPosition.h>
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

void printPositionData(RobotPosition &pos)
{

    Position position = pos.getPosition();
    float velocity = pos.getVelocity();
    std::cout << "--------------------------------------------" << std::endl;
    // Print current position data
    std::cout << "Position: (" << position.x << ", " << position.y 
                << "), Heading: " << position.azimuth 
                << "°, Confidence: " << position.confidence 
                << ", Velocity: " << velocity << " m/s" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

}


int main() {

    UPS ups;
    ups.initialize();

    IMU imu;
    imu.initialize();
    imu.start();


    imu.calibrateMagnetometer();
    imu.calibrateGyroscope();
    imu.calibrateAccelerometer();
    sleep(1);

    // boost::asio::io_service myService;

    // std::string brain_port = PortDetector::findBrainPorts()[0]; // Get first brain
    // Brain::BrainComm brain(myService, brain_port);
    // brain.start();

    // // Initialize RobotPosition (manages GPS internally)
    // std::cout << "Initializing position tracking..." << std::endl;
    // RobotPosition robotPosition(brain, imu);
    // if (!robotPosition.initialize(myService)) {
    //     std::cerr << "Failed to initialize position tracking" << std::endl;
    //     return 1;
    // }

    // robotPosition.start();

    // Camera camera;
    // Model model;
    // ObjectDetection objdet;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> volt_dist(-12.0, 12.0); // Replace with your desired range

    // Time interval in seconds
    const double volt_interval = .1; // Change to your desired interval
    auto volt_lastTime = std::chrono::steady_clock::now();     // Get starting time point


    while (true) 
    {

         //Exit on ESC
        // if (cv::waitKey(1) == 27) break;
        // brain.sendRequest(static_cast<u_int16_t>(Brain::RequestFlag::LeftGPSData));
        // camera.updateFrames();
        // camera.preprocessFrames(model.inferInput);
        // model.runInference();
        // std::vector<DetectedObject> Det = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
        // camera.displayDetections(Det);
    
        
        // auto currentTime = std::chrono::steady_clock::now();
        // double elapsedSeconds = std::chrono::duration<double>(currentTime - volt_lastTime).count();
        
        // if (elapsedSeconds >= volt_interval) 
        // {
        //     // Generate random number
        //     float randomNum = volt_dist(gen);
        //     brain.setMotorVoltages(randomNum,randomNum);
        //     volt_lastTime = currentTime;
        // }

        // brain.setJetsonBattery(ups.getBatteryPercentage());

        
        // printIMUdata(imu);
         printUPSdata(ups);
        // printPositionData(robotPosition);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));    
    }
    // brain.stop();
    
    
    return 0;
}




