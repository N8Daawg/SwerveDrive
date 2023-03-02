#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "CAN/CANConnection.hpp"


// Sets the connection open flag to false
CANConnection::CANConnection() {
    connOpen = false;
}


// the connection open flag will be set by the function call
CANConnection::CANConnection(const char* interface_name) {
    openConnection(interface_name);
}


// Uses the linux built in api for interacting with sockets.
// Will not raise any exceptions. If exceptions occur, the terminal
// will show it, but execution will continue. Opens socket with the
// no-block flag so that any read operations will return immediately
int CANConnection::openConnection(const char* interface_name) {
    if ((sockfd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)) < 0) {  // might be AF_CAN
       perror("Socket Failed");
       connOpen = false;

       return -1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name);
    ioctl(sockfd, SIOCGIFINDEX, &ifr);

    connOpen = true;

    return 0;
}


// Aquires the mutex in an exception safe way and writes
// a command to the can bus if it is properly set up
int CANConnection::writeFrame(uint32_t canId, uint8_t data[], int nBytes) {
    if(!connOpen) return -1;  // make sure connection is opened properly
    if(nBytes < 0 || nBytes > PACKET_LENGTH) return -2;  // only accept valid packet sizes

    const std::lock_guard<std::mutex> lock(conn_mutex);

    struct can_frame frame;
    frame.can_id = canId;
    frame.can_dlc = PACKET_LENGTH;
    for(int i = 0; i < PACKET_LENGTH; i++) {  // copy data to frame
        frame.data[i] = data[i];
    }

    if (write(sockfd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
       perror("Failed to write");
       return -2;
    }

    return 0;
}


// Reads a can frame and stores the data in the supplied parameters
int CANConnection::readFrame(uint32_t *canId, uint8_t data[PACKET_LENGTH]) {
    if(!connOpen) return -1;  // make sure connection is opened properly

    int nbytes;
    struct can_frame frame;
    nbytes = read(sockfd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {  // don't show error on terminal because the no-block flag is set meaning there might not always be data
       return -2;
    }

    // copy over frame data
    *canId = frame.can_id;
    for(int i = 0; i < PACKET_LENGTH; i++) {
        data[i] = frame.data[i];
    }

    return 0;
}
