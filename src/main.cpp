#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"
#include "CAN/can_utils.hpp"

#include <chrono>
#include <iostream>
#include <iomanip>


void sleep(int millis) {
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}


int main() {
    CANConnection canConnection("can0");

    SparkMaxMC motor1(canConnection, 1);
    SparkMaxMC motor2(canConnection, 2);

    motor1.setToFactoryDefaults();
    motor2.setToFactoryDefaults();

    motor2.setOutputRatio(1/7);
    motor2.setTicksPerEncoderRevolution(42);

    motor1.burnFlash();
    motor2.burnFlash();

    
    CANNetwork canNetwork(canConnection);
    canNetwork.addDevice(motor1);
    canNetwork.addDevice(motor2);


    std::cout << "\n";

    while(1) {
        //motor1.identify();
        std::cout << '\r' << std::left << std::setw(20) << motor2.getPosition() << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // sleep for 5 seconds before identifying again
    }

    return 0;
}
