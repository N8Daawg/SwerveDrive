#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"

#include <iostream>

int main() {
    CANConnection canConnection("can0");
    CANNetwork canNetwork(canConnection);

    SparkMaxMC motor1(canConnection, 1);
    SparkMaxMC motor2(canConnection, 2);

    canNetwork.addDevice(motor2);
    motor2.tareEncoder();
    motor1.tareEncoder();
    //canNetwork.addDevice(motor2);

    //motor1.dutyCycleSet(0.2);
    //std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again
    //motor1.dutyCycleSet(0);
    
    // motor1.smartVelocitySet(75);
    // motor2.smartVelocitySet(75);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again
    // motor1.smartVelocitySet(0);
    // motor2.smartVelocitySet(0);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again

    // motor1.voltageSet(6);
    // motor2.voltageSet(6);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again
    // motor1.voltageSet(0);
    // motor2.voltageSet(0);
    // std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again


    // motor1.positionSet(75);
    // motor2.positionSet(75);
    // std::this_thread::sleep_for(std::chrono::seconds(4)); // sleep for 5 seconds before identifying again
    // motor1.positionSet(0);
    // motor2.positionSet(0);
    // std::this_thread::sleep_for(std::chrono::seconds(4)); // sleep for 5 seconds before identifying again

    // motor1.smartPositionSet(75);
    // motor2.smartPositionSet(75);
    // std::this_thread::sleep_for(std::chrono::seconds(4)); // sleep for 5 seconds before identifying again
    // motor1.smartPositionSet(0);
    // motor2.smartPositionSet(0);
    // std::this_thread::sleep_for(std::chrono::seconds(4)); // sleep for 5 seconds before identifying again





    while(1) {
        motor1.identify();
        motor2.tareEncoder();
        std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again
    }

    return 0;
}
