// CANConnection.hpp
// Contains class that handles thread-safe communication with
// a can network
//
// Author: Aiden Carney

#ifndef CANCONNECTION_HPP
#define CANCONNECTION_HPP

#include <mutex>


#define PACKET_LENGTH 8  // number of bytes in each packet. This should always be 8

class CANConnection {
    private:
        bool connOpen;          // flag to know if the connection is currently open
        std::mutex conn_mutex;  // mutex to make read and write operations thread-safe

        int sockfd;                  // the socket where the CAN network is open


    public:

        // Default ctor for CANConnection
        // Sets the connection open flag to false
        //
        // Params:
        //    None
        // Return:
        //    the new CANConnection instance
        CANConnection();

        // ctor for CANConnection that attempts to open a new connection 
        // to the can bus
        // 
        // Params:
        //    interface_name - string with what the can interface is named ("can0", "vcan0", etc)
        // Return:
        //    the new CANConnection instance
        CANConnection(const char* interface_name);


        // Attempts to open and bind to the socket for the can network
        // 
        // Params:
        //    interface_name - string with what the can interface is named ("can0", "vcan0", etc)
        // Return:
        //    None
        int openConnection(const char* interface_name);


        // Writes a command to the can bus. Generates the frame based
        // on the id and the data. Thread safe
        //
        // Params:
        //    canId  - the 32 bit integer that is formatted for its intended target. Note: the 
        //             first 3 MSB's are not sent and are used for configuration
        //    data   - array of bytes that will be sent in the can frame. Up to documentation of
        //             CAN receiver as to what this should contain
        //    nBytes - the number of bytes to send in the frame. Same as length of data. Must be on
        //             the interval [0, PACKET_LENGTH]
        // Return:
        //    int -   0 if the operation was successful, -1 if connection not open, -2 if some other handled error
        int writeFrame(uint32_t canId, uint8_t data[], int nBytes);


        // Attempts to read a frame from the CAN network. Returns 0 if data was read
        //
        // Params:
        //    *canId - a pointer to a 32 bit integer where the can id will be placed
        //    data   - an array of bytes for the data to be placed if a frame is read
        // Return:
        //    int - 0 if successful, -1 if connection not open, -2 if some other error occurred (i.e. no frame to read)
        int readFrame(uint32_t *canId, uint8_t data[PACKET_LENGTH]);
};

#endif
