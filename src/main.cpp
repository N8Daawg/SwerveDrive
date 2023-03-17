#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"
#include "CAN/can_utils.hpp"
#include "Drive/SwerveModule.hpp"
#include "Drive/SwerveController.hpp"


void sleep(int millis) {
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}


int main() {        
    CANConnection canConnection("can0");

    SparkMaxMC motor1(canConnection, 1);
    SparkMaxMC motor2(canConnection, 2);
    motor1.dutyCycleSet(0);
    motor2.dutyCycleSet(0);

    motor1.setToFactoryDefaults();
    motor2.setToFactoryDefaults();

    motor1.setGearRatio(1/5.25);
    motor1.setTicksPerEncoderRevolution(4096);

    motor2.setGearRatio(1/(5.3333333 * 10));
    motor2.setTicksPerEncoderRevolution(42);

    motor2.setMotorReversed(true);

    motor1.burnFlash();
    motor2.burnFlash();

    
    CANNetwork canNetwork(canConnection);
    canNetwork.addDevice(motor1);
    canNetwork.addDevice(motor2);

    sleep(200);

    motor1.tareEncoder();
    motor2.tareEncoder();

    SwerveModule s1(motor1, motor2);
    s1.setMountLocation(-0.5, 0.5);
    s1.setUsePWM(true);
    //s1.moveRobotCentric(0, 1, 0);




    while(1) {
        s1.moveRobotCentric(1, 0, 0, -M_PI / 2);
        //motor1.identify();
        //motor2.printFaults(motor2.getFaults());
        //std::cout << '\r' << std::left << std::setw(20) << motor2.getAppliedOutput() << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
