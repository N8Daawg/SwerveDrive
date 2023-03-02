// CanDevice.hpp
// Contains a template class for creating and using a device on the can network
//
// Author: Aiden Carney

#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include <stdint.h>

#include "CANConnection.hpp"


class CANDevice {
    protected:
        uint8_t deviceId;    // the device can id 
        CANConnection conn;  // the can network connection instance


        // Creates a can frame and sends it over the can network. Must be
        // overriden
        //
        // Params:
        //    canFrameId - the integer containing 29 bits that correspond to the 
        //                 id of the can frame
        //    data       - the data to be sent
        //    nBytes     - the number of bytes to send in the packet. Same as length of
        //                 data
        // Return:
        //    int - if the frame was written successfully
        virtual int _sendFrame(uint32_t canFrameId, uint8_t data[], int nBytes) = 0;


    public: 
        // Virtual Destructor. Must be overriden
        virtual ~CANDevice() {};

        // Takes an incoming CAN Frame and responds accordingly. Must be
        // overriden. This method should likely be used in a loop so that
        // all frames can be read.
        //
        // Params:
        //    canFrameId - the integer containing 29 bits that correspond to the 
        //                 id of the can frame
        //    data       - the data to be parsed
        // Return:
        //    None
        virtual void _parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) = 0;


        // Returns the can id of the device
        //
        // Params:
        //    None
        // Return:
        //    uint8_t - the device id for this instance
        uint8_t getDeviceId() const {return deviceId;}


        // Sets a new can id for the device
        // 
        // Params:
        //    newId - the new id for the device
        // Return:
        //    None
        void setDeviceId(uint8_t newId) {deviceId = newId;}
};


#endif
