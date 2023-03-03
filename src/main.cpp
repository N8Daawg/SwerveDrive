#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"

int main() {
    CANConnection canConnection("vcan0");
    CANNetwork canNetwork(canConnection);

    SparkMaxMC motor1(canConnection, 1);

    canNetwork.addDevice(motor1);

    while(1) {
        motor1.identify();

        std::this_thread::sleep_for(std::chrono::seconds(2)); // sleep for 5 seconds before identifying again
    }

    return 0;
}
