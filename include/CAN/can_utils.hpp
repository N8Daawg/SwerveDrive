// can_utils.hpp
// Contains various methods for working with FRC can frames with a rapsberry pi.
// See below for the documentation on the structure of CAN frames and working with
// CAN on a linux machine
//     https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
//     https://docs.huihoo.com/doxygen/linux/kernel/3.7/can_8h_source.html
//
// Author: Aiden Carney

#ifndef CAN_UTILS_HPP
#define CAN_UTILS_HPP

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


// Number of bits for each of the fields in the can frame id
#define DEVICE_BITS 5
#define MANUFACTURER_BITS 8
#define API_CLASS_BITS 6
#define API_INDEX_BITS 4
#define DEVICE_NUMBER_BITS 6


typedef struct {
    int deviceType;
    int manufacturer;
    int apiClass;
    int apiIndex;
    int deviceNumber;
} can_id_params;


// returns 32 bit can identifier based on FRC protocol for interacting
// with motor controller
//
// Params:
//    msg_params - pointer to struct that will be used to generate the can frame id
// Return:
//    uint32_t - the can frame id with the first 3 bits set according to the linux
//               documentation
uint32_t genCanFrameID(can_id_params* msg_params);


// Decodes an integer (hex string) into its components based on the 
// FRC Can frame encoding. 
//
// Params:
//    msg        - the can id to decode into its components
//    msg_params - the struct to write the data to (should be intialized and empty)
// Return:
//    None
void decodeCanFrameID(uint32_t msg, can_id_params* msg_params);


// decodes the device can id from a can frame id
// 
// Params:
//    msg - the can frame id
// Return:
//    int - the device id 
int decodeCanFrameDevice(uint32_t msg);


// populates an array of bytes from an unsigned integer
// with the msb at index 0 and the lsb at index numBytes - 1
//
// Params:
//    n        - the number to split into bytes
//    bytes    - the array where the bytes will be stored with the msb at 
//               index 0 and the lsb at index numBytes - 1
//    numBytes - the number of bytes to split the integer into (usually 8)
// Return:
//    None
void uint64ToBytes(uint64_t n, uint8_t bytes[], uint8_t numBytes);


// populates an array of bytes from a floating point number
// with the msb at index 0 and the lsb at index numBytes - 1
//
// Params:
//    n        - the number to split into bytes
//    bytes    - the array where the bytes will be stored with the msb at 
//               index 0 and the lsb at index numBytes - 1
//    numBytes - the number of bytes to split the integer into (usually 8)
// Return:
//    None
void floatToBytes(float n, uint8_t bytes[], uint8_t numBytes);


// given an array of bytes with the msb at index 0 and lsb and index
// numBytes - 1, creates and returns an unsigned 64 bit integer
//
// Params:
//    bytes    - an array of bytes to decode, must be same size as numBytes
//    numBytes - the number of bytes to decode (usually 8)
// Return:
//    int64_t - the decoded 64 bit integer
int64_t bytesTouint64(uint8_t bytes[], uint8_t numBytes);


// given an array of bytes with the msb at index 0 and lsb and index
// numBytes - 1, creates and returns floating point number
//
// Params:
//    bytes    - an array of bytes to decode, must be same size as numBytes
//    numBytes - the number of bytes to decode (usually 8)
// Return:
//    float - the decoded floating point number
float bytesToFloat(uint8_t bytes[], uint8_t numBytes);


#endif
