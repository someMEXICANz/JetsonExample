#include <Camera.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <GPS.h>
#include <PortDetector.h>
#include <IMU.h>
#include <iostream>
#include <BrainComm.h>
//#include <CommsMonitor.h>
#include <UPS.h>
#include <iostream>
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
    if (imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz, temperature)) {
        std::cout << "--------------------------------------------" << std::endl;
        // Display IMU Data
        std::cout << "Accel (g):  X: " << ax << "  Y: " << ay << "  Z: " << az << std::endl;    // Accelerometer data in g
        std::cout << "Gyro (dps): X: " << gx << "  Y: " << gy << "  Z: " << gz << std::endl;    // Gyroscope data in degrees per second
        std::cout << "Mag (gauss): X: " << mx << "  Y: " << my << "  Z: " << mz << std::endl;   // Magnetometer data in gauss
        std::cout << "Temperature: " << temperature << " °C" << std::endl;                      // Temperature in Celsius
         std::cout << "--------------------------------------------" << std::endl;
    } 
    else {
         std::cout << "--------------------------------------------" << std::endl;
        std::cerr << "Error reading sensor data: " << imu.getLastError() << std::endl;
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






int main() {

    UPS ups;
    ups.initialize();

    IMU imu;
    imu.initialize();


    imu.calibrateMagnetometer();
    imu.calibrateGyroscope();
    imu.calibrateAccelerometer();
    sleep(1);

    boost::asio::io_service myService;

    std::string brain_port = PortDetector::findBrainPorts()[0]; // Get first brain
    Brain::BrainComm brain(myService, brain_port);
    brain.start();

    std::string GPS1_port = PortDetector::findGPSPorts()[0]; // Get first gps
    GPS gps1(myService, GPS1_port);
    gps1.start();


    std::string GPS2_port = PortDetector::findGPSPorts()[1]; // Get second gps
    GPS gps2(myService, GPS2_port);
    gps2.start();

    // Camera camera;
    // Model model;
    // ObjectDetection objdet;


    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> request_dist(1, 3); // Replace with your desired range
    std::uniform_real_distribution<float> volt_dist(0, 12.0); // Replace with your desired range

    // Time interval in seconds
    const double request_interval = 10; // Change to your desired interval
    const double volt_interval = .5; // Change to your desired interval

    auto request_lastTime = std::chrono::steady_clock::now();     // Get starting time point
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
    
        
        auto currentTime = std::chrono::steady_clock::now();
        // Calculate elapsed time in seconds
        double elapsedSeconds = std::chrono::duration<double>(currentTime - request_lastTime).count();
        
        // Check if interval has passed
        if (elapsedSeconds >= request_interval) 
        {
            // Generate random number
            int randomNum = request_dist(gen);
            switch(randomNum)
            {
                case 1:
                    brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::LeftGPSData));
                    std::cout << "Requesting Left GPS Data" << std::endl;
                    break;
                case 2:
                    brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::SisterData));
                    std::cout << "Requesting Sister GPS Data" << std::endl;
                    break;
                case 3:
                    brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::BatteryLevel));
                    std::cout << "Requesting Right GPS Data" << std::endl;
                    break;
            }
            
            // Reset timer
            request_lastTime = currentTime;
        }

        if (elapsedSeconds >= volt_interval) 
        {
            // Generate random number
            float randomNum = volt_dist(gen);
            brain.setMotorVoltages(randomNum,randomNum);
            volt_lastTime = currentTime;
        }

        brain.setJetsonBattery(ups.getBatteryPercentage());

        
        printIMUdata(imu);
        printUPSdata(ups);

        usleep(100000);  // 100ms refresh rate
    }

    // monitor.stop();
    brain.stop();
    
    
    return 0;
}




