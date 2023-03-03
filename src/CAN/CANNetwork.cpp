#include "CAN/can_utils.hpp"
#include "CAN/CANNetwork.hpp"


// Updates the current connection
CANNetwork::CANNetwork(CANConnection& newConn) {
    t = std::thread(&CANNetwork::_mainloop, this);
    conn = &newConn;
    runThread = true;
}


void CANNetwork::_mainloop() {
    while(true) {
        std::this_thread::sleep_for(std::chrono::microseconds(10000)); // sleep for 10ms

        // write the heartbeat
        can_id_params params = {2, 5, 11, 2, 0};
        uint32_t id = genCanFrameID(&params);
        uint8_t heartbeatBytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        conn->writeFrame(id, heartbeatBytes, 8);

        if(!runThread) continue;  // make sure we should be reading

        uint32_t canId;
        uint8_t data[PACKET_LENGTH];
        if(conn->readNextFrame(&canId, data) == 0) {
            int deviceId = decodeCanFrameDevice(canId);

            const std::lock_guard<std::mutex> lock(deviceMutex);  // take access of the mutex to iterate over devices
            for(CANDevice* c : devices) {
                if(c->getDeviceId() == deviceId) {
                    c->_parseIncomingFrame(canId, data);
                    break;
                }
            }
        }
    }
}

// Adds a new device to the list of registered devices
// if it is not already there. Compares based on device id
int CANNetwork::addDevice(CANDevice& newDevice) {
    int newId = newDevice.getDeviceId();
    for(CANDevice* c : devices) {  // thread is only a read operation, so this is guarenteed safe without the lock
        if(c->getDeviceId() == newId) {
            return -1;
        }
    }

    const std::lock_guard<std::mutex> lock(deviceMutex);  // take access of the mutex to add device
    devices.push_back(&newDevice);

    return 0;
}
