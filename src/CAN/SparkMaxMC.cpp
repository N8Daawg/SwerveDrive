#include <chrono>
#include <iostream>

#include "CAN/can_utils.hpp"
#include "CAN/SparkMaxMC.hpp"


// intializes members to either null or 0
SparkMaxMC::SparkMaxMC() {
    conn = NULL;
    deviceId = 0;
}


// initializes members to supplied values
SparkMaxMC::SparkMaxMC(CANConnection& connection, int canDeviceId) {
    conn = &connection;
    deviceId = canDeviceId;
}


// Creates the struct of parameters and then gets the 
// corresponding integer using a method from can_utils
uint32_t SparkMaxMC::getCanFrameId(int apiClass, int apiIndex) {
    can_id_params params = {2, 5, apiClass, apiIndex, deviceId};
    return genCanFrameID(&params);
}


void SparkMaxMC::debugIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) {
    can_id_params params;
    decodeCanFrameID(canFrameId, &params);

    printf("Decoding of message %x:\n", canFrameId);
    printf("    Device Type:   %d\n", params.deviceType);
    printf("    Manufacturer:  %d\n", params.manufacturer);
    printf("    API Class:     %d\n", params.apiClass);
    printf("    API Index:     %d\n", params.apiIndex);
    printf("    Device Number: %d\n", params.deviceNumber);
    printf("    ");
    for(int i = 0; i < PACKET_LENGTH; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");
}

// Takes an incoming CAN Frame and responds accordingly.
void SparkMaxMC::_parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) {
    can_id_params params;
    decodeCanFrameID(canFrameId, &params);

    if(params.apiClass == 6) {  // 6 corresponds to periodic data being sent back
        switch(params.apiIndex) {
            case 0: { // periodic status 0 - applied output, faults, sticky faults, is follower
                break;
            }

            case 1: {// periodic status 1 - velocity, temperature, voltage, current
                uint8_t velocityBytes[4] = {data[0], data[1], data[2], data[3]};
                float velocity = bytesToFloat(velocityBytes, 4);
                // std::cout << "Motor " << unsigned(deviceId) << " velocity: " << velocity << "\n";
                break;
            }

            case 2: { // periodic status 2 - motor position
                uint8_t positionBytes[4] = {data[0], data[1], data[2], data[3]};
                float position = bytesToFloat(positionBytes, 4);
                //std::cout << "Motor " << unsigned(deviceId) << " position: " << position << "\n";
                break;
            }

            case 3: { // periodic status 3 - analog sensor voltage, velocity, position
                break;
            }

            case 4: { // periodic status 4 - alternate encoder velocity, position
                break;
            }



        }
    }
}


// (Duty Cycle Set) sets the target duty cycle
int SparkMaxMC::dutyCycleSet(float percent) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(0, 2);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(percent, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Speed Set) sets the target velocity of the motor in RPM. 
int SparkMaxMC::velocitySet(float targetRPM) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(1, 2);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(targetRPM, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Smart Velocity Set) sets the target velocity of the motor in RPM. 
// Honors the max acceleration and max velocity from smart motion 
// parameters at the firmware level
int SparkMaxMC::smartVelocitySet(float targetRPM) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(1, 3);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(targetRPM, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Voltage Set) Sets the closed loop speed controller where the
// target voltage is in volts  
int SparkMaxMC::voltageSet(float targetVoltage) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(4, 2);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(targetVoltage, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Position Set) Sets the closed loop speed controller where the
// target position is in rotations      
int SparkMaxMC::positionSet(float targetRotations) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(3, 2);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(targetRotations, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Smart Motion Set) Sets the closed loop smart motion 
// controller where the target position is in rotations 
int SparkMaxMC::smartPositionSet(float targetRotations) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(5, 2);  // api class and api index

    uint8_t bytes[4];  // takes a 4 byte floating point number
    floatToBytes(targetRotations, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Parameter Access) writes the frame that updates a parameter on the spark max
int SparkMaxMC::setGenericParameter(E_SPARKMAX_PARAM param, uint8_t packet[PACKET_LENGTH]) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(48, 0);   // api class and api index
    canId |= (param << DEVICE_NUMBER_BITS);  // from docs, api section of id is or'd with the desired parameter

    return conn->writeFrame(canId, packet, 5);
}


// (Parameter Access) sends a frame to request the data and then polls the connection
// response queue until found or timeout is hit
int SparkMaxMC::readGenericParameter(E_SPARKMAX_PARAM param, uint8_t response[PACKET_LENGTH], int timeout_ms /*100*/) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(48, 0);   // api class and api index
    canId |= (param << DEVICE_NUMBER_BITS);  // from docs, api section of id is or'd with the desired parameter

    uint8_t bytes[0];
    conn->writeFrame(canId, bytes, 0);  // send 0 length message to view the parameter

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = begin;

    uint32_t bitmask = 0xFFFFFFFF;            // response can id should be the same so search for 
    uint32_t specifier = canId & bitmask;     // only that
    uint32_t responseId = 0;

    int dataRead = conn->readNextFrameIf(bitmask, specifier, &responseId, response);
    while(std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() < timeout_ms && dataRead != 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(400)); // wait for a bit before re-trying
        dataRead = conn->readNextFrameIf(bitmask, specifier, &responseId, response);

        end = std::chrono::steady_clock::now();
    }

    return dataRead;
}


// (Telemetry Update Mechanical Position Enoder Port) Uses
// api command to update the position, but only for setting
// the position to 0  
int SparkMaxMC::tareEncoder() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(10, 0);  // api class and api index

    uint8_t bytes[4];  // 4 byte floating point number for 0
    floatToBytes(99999, bytes, 4);

    return conn->writeFrame(canId, bytes, 4);
}


// (Identify) Causes the motor controller LED to flash
// rapidly so it can be identified 
int SparkMaxMC::identify() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 6);  // api class and api index

    uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // just send empty message

    return conn->writeFrame(canId, bytes, 0);
}
