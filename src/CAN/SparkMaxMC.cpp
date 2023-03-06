#include <chrono>
#include <iostream>
#include <cstring>

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
                uint8_t appliedBytes[2] = {data[0], data[1]};
                uint8_t faultBytes[2] = {data[2], data[3]};
                uint8_t stickyFaultBytes[2] = {data[4], data[5]};

                appliedOutput = bytesTouint64(appliedBytes, 2) / 32767.0;  // divide by int16 max to scale to [-1, 1]
                faults = bytesTouint64(faultBytes, 2);
                stickyFaults = bytesTouint64(stickyFaultBytes, 2);
                isFollower = data[7];

                break;
            }

            case 1: {// periodic status 1 - velocity, temperature, voltage, current
                uint8_t velocityBytes[4] = {data[0], data[1], data[2], data[3]};
                velocity_rpm = bytesToFloat(velocityBytes, 4);
                temperature_c = data[4];

                // TODO: read voltage and current

                break;
            }

            case 2: { // periodic status 2 - motor position
                uint8_t positionBytes[4] = {data[0], data[1], data[2], data[3]};
                float position = bytesToFloat(positionBytes, 4);
                std::cout << "Motor " << unsigned(deviceId) << " position: " << position << "\n";
                break;
            }

            case 4: { // periodic status 4 - alternate encoder velocity, position
                uint8_t velocityBytes[4] = {data[0], data[1], data[2], data[3]};
                uint8_t positionBytes[4] = {data[4], data[5], data[6], data[7]};
                altEncoderVelocity_rpm = bytesToFloat(velocityBytes, 4);
                _altEncoderPosition = bytesToFloat(positionBytes, 4);
                break;
            }

            // don't read periodic 3, 5, and 6 because we don't use them


        }
    }
}


// (Duty Cycle Set) sets the target duty cycle
int SparkMaxMC::dutyCycleSet(float percent) {
    if(conn == NULL) return -1;
    if(percent > 1 || percent < -1) return -4;  // invalid parameters

    uint32_t canId = getCanFrameId(0, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number, rest should be 0s unless 
                       // you really know what you are doing. This ignores any of the
                       // feed-foarward or pid data in the frame which should be fine
                       // because it is not used in a duty cycle command
    memset(bytes, 0, sizeof(bytes));

    floatToBytes(percent, bytes, 4);  // fill first 4 bytes with the setpoint

    return conn->writeFrame(canId, bytes, 8);
}


// (Speed Set) sets the target velocity of the motor in RPM. 
int SparkMaxMC::velocitySet(float targetRPM) {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(1, 2);  // api class and api index

    uint8_t bytes[8];  // takes a 4 byte floating point number
    memset(bytes, 0, sizeof(bytes));

    floatToBytes(targetRPM, bytes, 4);  // fill first 4 bytes with the setpoint

    return conn->writeFrame(canId, bytes, 8);
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


// makes parameter update call to set the motor reversed setting
int SparkMaxMC::setMotorReversed(bool reverse) {
    if(conn == NULL) return -1;

    uint8_t setReverse = 0;
    if(reverse) setReverse = 1;

    uint8_t data[5] = {setReverse, 0, 0, 0, sparkmax_bool};

    int response = setGenericParameter(kInverted, data);
    if(response == 0) {
        isReversed = setReverse;
    } 

    return response;
}


// makes parameter update call to set the encoder mode
int SparkMaxMC::setEncoderMode(bool alternate) {
    if(conn == NULL) return -1;

    uint8_t useAlt = 0;
    if(alternate) useAlt = 1;

    uint8_t data[5] = {useAlt, 0, 0, 0, sparkmax_bool};

    int response = setGenericParameter(kDataPortConfig, data);
    if(response == 0) {
        encoderMode = useAlt;
    } 

    return response;

}


// makes parameter update call to set idle mode
int SparkMaxMC::setIdleMode(uint8_t newIdleMode) {
    if(conn == NULL) return -1;

    uint8_t data[5] = {newIdleMode, 0, 0, 0, sparkmax_bool};

    return setGenericParameter(kIdleMode, data);
}


// makes call to parameter set function
int SparkMaxMC::setkP(float kP, int slot /*0*/) {
    uint8_t kP_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kP, kP_data, 4);
    kP_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kP_0, kP_data);
        case 1:  return setGenericParameter(kP_1, kP_data);
        case 2:  return setGenericParameter(kP_2, kP_data);
        case 3:  return setGenericParameter(kP_3, kP_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkI(float kI, int slot /*0*/) {
    uint8_t kI_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kI, kI_data, 4);
    kI_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kI_0, kI_data);
        case 1:  return setGenericParameter(kI_1, kI_data);
        case 2:  return setGenericParameter(kI_2, kI_data);
        case 3:  return setGenericParameter(kI_3, kI_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkD(float kD, int slot /*0*/) {
    uint8_t kD_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kD, kD_data, 4);
    kD_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kD_0, kD_data);
        case 1:  return setGenericParameter(kD_1, kD_data);
        case 2:  return setGenericParameter(kD_2, kD_data);
        case 3:  return setGenericParameter(kD_3, kD_data);
        default: return -1;
    }
}


// makes call to parameter set function
int SparkMaxMC::setkF(float kF, int slot /*0*/) {
    uint8_t kF_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    floatToBytes(kF, kF_data, 4);
    kF_data[4] = sparkmax_float32;
    switch(slot) {
        case 0:  return setGenericParameter(kF_0, kF_data);
        case 1:  return setGenericParameter(kF_1, kF_data);
        case 2:  return setGenericParameter(kF_2, kF_data);
        case 3:  return setGenericParameter(kF_3, kF_data);
        default: return -1;
    }
}


// makes call to all of the individual set functions, this
// is purely for convenience. Return value is sum of all 
// return values from each parameter set
int SparkMaxMC::setPIDF(float kP, float kI, float kD, float kF, int slot /*0*/) {
    int response = 0;
    response += setkP(kP, slot);
    response += setkI(kI, slot);
    response += setkD(kD, slot);
    response += setkF(kF, slot);
    return response;
}


// makes call to read parameter for each of the individual 
// pid constants
int SparkMaxMC::getFullPIDF(pidf_constants& constants, int slot /*0*/) {
    if(slot < 0 || slot > 3) return -4;  // not sent, invalid slot

    int response = 0;

    // array for the data to be read into
    uint8_t response_data[8];
    float values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    const int timeout = 50;  // don't wait longer than 50ms for each call

    E_SPARKMAX_PARAM paramsToRead[8];
    paramsToRead[0] = static_cast<E_SPARKMAX_PARAM>(kP_0 + (8 * slot));  // constants are all right next to each other
    paramsToRead[1] = static_cast<E_SPARKMAX_PARAM>(kI_0 + (8 * slot));  // with an offset of 8 for each group
    paramsToRead[2] = static_cast<E_SPARKMAX_PARAM>(kD_0 + (8 * slot));
    paramsToRead[3] = static_cast<E_SPARKMAX_PARAM>(kF_0 + (8 * slot));
    paramsToRead[4] = static_cast<E_SPARKMAX_PARAM>(kIZone_0 + (8 * slot));
    paramsToRead[5] = static_cast<E_SPARKMAX_PARAM>(kDFilter_0 + (8 * slot));
    paramsToRead[6] = static_cast<E_SPARKMAX_PARAM>(kOutputMin_0 + (8 * slot));
    paramsToRead[7] = static_cast<E_SPARKMAX_PARAM>(kOutputMax_0 + (8 * slot));

    for(int i = 0; i < 8; i++) {
        memset(response_data, 0, sizeof(response_data));  // clear data array to all 0 before reading again

        if(readGenericParameter(paramsToRead[i], response_data, 50) == 0) {  // read the parameter
            values[i] = bytesToFloat(response_data, 4);
        } else {
            response -= 1;
        }
    }

    constants.kP = values[0];
    constants.kI = values[1];
    constants.kD = values[2];
    constants.kF = values[3];
    constants.kIZone = values[4];
    constants.kDFilter = values[5];
    constants.kOutputMin = values[6];
    constants.kOutputMax = values[7];


    return response;
}


// (Telemetry Update Mechanical Position Enoder Port) Uses
// api command to update the position, but only for setting
// the position to 0  
int SparkMaxMC::tareEncoder() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(10, 0);  // api class and api index

    uint8_t bytes[8];  // 4 byte floating point number for 0
    memset(bytes, 0, sizeof(bytes));
    floatToBytes(0.0, bytes, 4);

    return conn->writeFrame(canId, bytes, 8);
}



// (Config Factory Defaults) Makes simple api call to device
int SparkMaxMC::setToFactoryDefaults() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 4);             // api class and api index

    uint8_t bytes[5] = {0, 0, 0, 0, sparkmax_bool};   // 5 bytes from documentation

    return conn->writeFrame(canId, bytes, 5);
}


// (Config Burn Flash) Makes simple api call to device  
int SparkMaxMC::burnFlash() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 2);  // api class and api index

    uint8_t bytes[2] = {0xA3, 0x3A};       // 2 bytes from documentation

    return conn->writeFrame(canId, bytes, 2);
}


// (Clear Faults) sends api command to clear faults
int SparkMaxMC::clearStickyFaults() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(6, 14);  // api class and api index

    return conn->writeFrame(canId, NULL, 0);
}


// prints faults in a human readable format. There are 16 possible
// faults. These were determined from the REV Hardware client, so 
// it is possible that they are not accurate (they were not documented
// anywhere else)
void SparkMaxMC::printFaults(uint16_t faultString) {
    const char* faultStrings[16] = {
        "Brownout", "Over Current", "Watchdog Reset", "Motor Type",
        "Sensor Fault", "Stall", "EEPROM", "CAN TX",
        "CAN RX", "Has Reset", "Gate Driver Fault", "Hardware Fault",
        "Soft Limit Forward", "Soft Limit Reverse", "Hard Limit Forward", "Hard Limit Reverse"
    };
    printf("Fault Decoding for %d:\n", faultString);
    for(int i = 0; i < 16; i++) {
        int bit = faultString & 1;
        printf("    %s: %d\n", faultStrings[i], bit);

        faultString >>= 1;  // shift to next bit
    }

    printf("\n");
}


// (Identify) Causes the motor controller LED to flash
// rapidly so it can be identified 
int SparkMaxMC::identify() {
    if(conn == NULL) return -1;

    uint32_t canId = getCanFrameId(7, 6);  // api class and api index

    uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // just send empty message

    return conn->writeFrame(canId, bytes, 0);
}
