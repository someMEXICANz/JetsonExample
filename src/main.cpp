#include <Camera.h>
#include <Model.h>
#include <ObjectDetection.h>
#include <GPS.h>
#include <PortDetector.h>
#include <IMU.h>
#include <iostream>
#include <BrainComm.h>
#include <CommsMonitor.h>
#include <UPS.h>
#include <iostream>
#include <random>
#include <chrono>

using namespace std;






int main() {
   

    boost::asio::io_service myService;
    
    std::string brain_port = PortDetector::findBrainPorts()[0]; // Get first brain
    Brain::BrainComm brain(myService, brain_port);
    UPS ups;
    ups.initialize();
    // Brain::CommsMonitor monitor("Brain Communication Monitor");

 

    brain.start();
    sleep(1);
    brain.updateRequests(static_cast<uint16_t>(Brain::RequestFlag::BatteryLevel));
    // monitor.start();
 

    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<int> dist(1, 3); // Replace with your desired range
    // // Time interval in seconds
    // const double interval = 10; // Change to your desired interval
    // auto lastTime = std::chrono::steady_clock::now();     // Get starting time point

    while (true) 
    {

    
        
        // auto currentTime = std::chrono::steady_clock::now();
        // // Calculate elapsed time in seconds
        // double elapsedSeconds = std::chrono::duration<double>(currentTime - lastTime).count();
        
        // // Check if interval has passed
        // if (elapsedSeconds >= interval) 
        // {
        //     // Generate random number
        //     int randomNum = dist(gen);
        //     switch(randomNum)
        //     {
        //         case 1:
        //             brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::LeftGPSData));
        //             std::cout << "Requesting Left GPS Data" << std::endl;
        //             break;
        //         case 2:
        //             brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::SisterData));
        //             std::cout << "Requesting Sister GPS Data" << std::endl;
        //             break;
        //         case 3:
        //             brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::RightGPSData));
        //             std::cout << "Requesting Right GPS Data" << std::endl;
        //             break;
        //     }
            
        //     // Reset timer
        //     lastTime = currentTime;
        // }
        // monitor.updateData(brain);
        // if (cv::waitKey(1) == 27) break;
        brain.setJetsonBattery(ups.getBatteryPercentage());
        cout << "Brain Battery: " << brain.getBrainBattery() << "%" << endl;
        usleep(200000);  // 100ms refresh rate
       
        
       
    }

    // monitor.stop();
    brain.stop();
    
    
    return 0;
}
















































































































// int main() {
   
//     
//     // IMU imu;
//     boost::asio::io_service myService;
  
    
//     std::string brain_port = PortDetector::findBrainPorts()[0]; // Get first brain
//     Brain::BrainComm brain(myService, brain_port);
//     Brain::CommsMonitor monitor("Brain Communication Monitor");

//     // std::string GPS1_port = PortDetector::findGPSPorts()[0]; // Get first gps
//     // GPS gps1(myService, GPS1_port);

//     // std::string GPS2_port = PortDetector::findGPSPorts()[1]; // Get second gps
//     // GPS gps2(myService, GPS2_port);

//     brain.start();
//     monitor.start();
//     // gps1.start();
//     // gps2.start();


//     // ups.initialize();
//     // imu.initialize();

//     // imu.calibrateMagnetometer();
//     // imu.calibrateGyroscope();
//     // sleep(2);
    
//     // Camera camera;
//     // Model model;
//     // ObjectDetection objdet;

   
//     // float ax, ay, az,                                           // Accelerometer data
//     //       gx, gy, gz,                                           // Gyroscope data
//     //       mx, my, mz;                                           // Magnetometer data
    
//     // float temperature,                                          // Temperature data
//     //       bus_voltage,                                          // voltage on V- (load side)
//     //       shunt_voltage,                                        // voltage between V+ and V- across the shunt (in V)
//     //       current,                                              // current in mA
//     //       power,                                                // power in W
//     //       percentage;                                           // battery percentage

//     //  // Set precision for floating-point output
//     // std::cout << std::fixed << std::setprecision(3);

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<int> dist(1, 3); // Replace with your desired range
//     // Time interval in seconds
//     const double interval = 10; // Change to your desired interval
//     auto lastTime = std::chrono::steady_clock::now();     // Get starting time point

//     while (true) {

//         // Process camera frames and run inference every frame
        
//         // camera.updateFrames();
//         // camera.preprocessFrames(model.inferInput);
//         // model.runInference();
//         // std::vector<DetectedObject> Detections = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
//         // camera.displayDetections(Detections);

//         monitor.updateData(brain);

//         // // Get voltage and current measurements
//         // bus_voltage = ups.getBusVoltage_V();              // voltage on V- (load side)
//         // shunt_voltage = ups.getShuntVoltage_mV() / 1000;  // voltage between V+ and V- across the shunt (in V)
//         // current = ups.getCurrent_mA();                    // current in mA
//         // power = ups.getPower_W();                         // power in W
//         // percentage = ups.getBatteryPercentage();          // battery percentage

//         auto currentTime = std::chrono::steady_clock::now();
        
//         // Calculate elapsed time in seconds
//         double elapsedSeconds = std::chrono::duration<double>(currentTime - lastTime).count();
        
//         // Check if interval has passed
//         if (elapsedSeconds >= interval) {
//             // Generate random number
//             int randomNum = dist(gen);
//             switch(randomNum)
//             {
//                 case 1:
//                     brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::LeftGPSData));
//                     std::cout << "Requesting Left GPS Data" << std::endl;
//                     break;
//                 case 2:
//                     brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::SisterData));
//                     std::cout << "Requesting Sister GPS Data" << std::endl;
//                     break;
//                 case 3:
//                     brain.updateRequests(static_cast<u_int16_t>(Brain::RequestFlag::RightGPSData));
//                     std::cout << "Requesting Right GPS Data" << std::endl;
//                     break;
//             }
            
//             // Reset timer
//             lastTime = currentTime;
//         }

//         if (cv::waitKey(1) == 27) break;
//         usleep(100000);  // 100ms refresh rate
       
        
       
//     }

//     monitor.stop();
//     brain.stop();
    
    
//     return 0;
// }





// #include <Camera.h>
// #include <Model.h>
// #include <ObjectDetection.h>
// #include <GPS.h>
// #include <PortDetector.h>
// #include <iostream>
// #include <BrainComm.h>

// using namespace std;

// int main() {
 

//     // Initialize the model
        
//         std::string brain_port = PortDetector::findBrainPorts()[0]; // Get first brain
//         std::string GPS1_port = PortDetector::findGPSPorts()[0]; // Get first gps
//         std::string GPS2_port = PortDetector::findGPSPorts()[1]; // Get second gps
        
//         boost::asio::io_service myService ;
    
//         Brain::BrainComm brain(myService, brain_port);
//         GPS gps1(myService, GPS1_port);
//         GPS gps2(myService, GPS2_port);
//         brain.start();
//         gps1.start();
//         gps2.start();


//         Camera camera;
//         Model model;
//         ObjectDetection objdet;

//         while (true) 
//         {
//             //Exit on ESC
//             if (cv::waitKey(1) == 27) break;
//             brain.sendRequest(static_cast<u_int16_t>(Brain::RequestFlag::LeftGPSData));
//             camera.updateFrames();
//             camera.preprocessFrames(model.inferInput);
//             model.runInference();
//             std::vector<DetectedObject> Det = objdet.decodeOutputs(model.inferOutput1, model.inferOutput2);
//             camera.displayDetections(Det);




                        // // Read all sensor data
                // if (imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz, temperature)) {
                //     std::cout << "--------------------------------------------" << std::endl;
                //     // Display IMU Data
                //     std::cout << "Accel (g):  X: " << ax << "  Y: " << ay << "  Z: " << az << std::endl;    // Accelerometer data in g
                //     std::cout << "Gyro (dps): X: " << gx << "  Y: " << gy << "  Z: " << gz << std::endl;    // Gyroscope data in degrees per second
                //     std::cout << "Mag (gauss): X: " << mx << "  Y: " << my << "  Z: " << mz << std::endl;   // Magnetometer data in gauss
                //     std::cout << "Temperature: " << temperature << " °C" << std::endl;                      // Temperature in Celsius
                // } else {
                //     std::cerr << "Error reading sensor data: " << imu.getLastError() << std::endl;
                // }

                // std::cout << "--------------------------------------------" << std::endl;
                // // Display UPS data 
                // std::cout << "PSU Voltage:   " << (bus_voltage + shunt_voltage) << " V" << std::endl;
                // std::cout << "Load Voltage:  " << bus_voltage << " V" << std::endl;
                // std::cout << "Current:       " << (current / 1000.0f) << " A" << std::endl;
                // std::cout << "Power:         " << power << " W" << std::endl;
                // std::cout << "Percent:       " << percentage << "%" << std::endl;
                
                // Wait before next reading
//         }


//     return 0;
// }


