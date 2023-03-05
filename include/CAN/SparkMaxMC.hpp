// SparkMaxMC.hpp
// Contains class for dealing with a Spark Max Motor Controller based
// on their provided documentation
//
// Author: Aiden Carney

#ifndef SPARKMAXMC_HPP
#define SPARKMAXMC_HPP


#include "CANConnection.hpp"
#include "CANDevice.hpp"


// each parameter has a type associated with it that corresponds to an
// integer value, these are not documented anywhere else and were 
// found from trial and error. These are needed because to update 
// parameters, one of the bytes needs to be the type.
enum E_SPARKMAX_PARAM_TYPE {
    sparkmax_uint = 1,
    sparkmax_float32,
    sparkmax_bool
};

// the valid parameters on the sparkmax that can be viewed and
// changed. These are available in more detail on Rev Robotics
// website: https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters
enum E_SPARKMAX_PARAM {
    kCanID = 0,
    kInputMode,
    kMotorType,
    kCommAdvance,
    kSensorType,
    kCtrlType,
    kIdleMode,
    kInputDeadband,
    kFeedbackSensorPID0,
    kFeedbackSensorPID1,
    kPolePairs,
    kCurrentChop,
    kCurrentChopCycles,
    kP_0,
    kI_0,
    kD_0,
    kF_0,
    kIZone_0,
    kDFilter_0,
    kOutputMin_0,
    kOutputMax_0,
    kP_1,
    kI_1,
    kD_1,
    kF_1,
    kIZone_1,
    kDFilter_1,
    kOutputMin_1,
    kOutputMax_1,
    kP_2,
    kI_2,
    kD_2,
    kF_2,
    kIZone_2,
    kDFilter_2,
    kOutputMin_2,
    kOutputMax_2,
    kP_3,
    kI_3,
    kD_3,
    kF_3,
    kIZone_3,
    kDFilter_3,
    kOutputMin_3,
    kOutputMax_3,
    kInverted,
    kOutputRatio,
    kSerialNumberLow,
    kSerialNumberMid,
    kSerialNumberHigh,
    kLimitSwitchFwdPolarity,
    kLimitSwitchRevPolarity,
    kHardLimitFwdEn,
    kHardLimitRevEn,
    kSoftLimitFwdEn,
    kSoftLimitRevEn,
    kRampRate,
    kFollowerID,
    kFollowerConfig,
    kSmartCurrentStallLimit,
    kSmartCurrentFreeLimit,
    kSmartCurrentConfig,
    kSmartCurrentReserved,
    kMotorKv,
    kMotorR,
    kMotorL,
    kMotorRsvd1,
    kMotorRsvd2,
    kMotorRsvd3,
    kEncoderCountsPerRev,
    kEncoderAverageDepth,
    kEncoderSampleDelta,
    kEncoderInverted,
    kEncoderRsvd1,
    kClosedLoopVoltageMode,
    kCompensatedNominalVoltage,
    kSmartMotionMaxVelocity_0,
    kSmartMotionMaxAccel_0,
    kSmartMotionMinVelOutput_0,
    kSmartMotionAllowedClosedLoopError_0,
    kSmartMotionAccelStrategy_0,
    kSmartMotionMaxVelocity_1,
    kSmartMotionMaxAccel_1,
    kSmartMotionMinVelOutput_1,
    kSmartMotionAllowedClosedLoopError_1,
    kSmartMotionAccelStrategy_1,
    kSmartMotionMaxVelocity_2,
    kSmartMotionMaxAccel_2,
    kSmartMotionMinVelOutput_2,
    kSmartMotionAllowedClosedLoopError_2,
    kSmartMotionAccelStrategy_2,
    kSmartMotionMaxVelocity_3,
    kSmartMotionMaxAccel_3,
    kSmartMotionMinVelOutput_3,
    kSmartMotionAllowedClosedLoopError_3,
    kSmartMotionAccelStrategy_3,
    kIMaxAccum_0,
    kSlot3Placeholder1_0,
    kSlot3Placeholder2_0,
    kSlot3Placeholder3_0,
    kIMaxAccum_1,
    kSlot3Placeholder1_1,
    kSlot3Placeholder2_1,
    kSlot3Placeholder3_1,
    kIMaxAccum_2,
    kSlot3Placeholder1_2,
    kSlot3Placeholder2_2,
    kSlot3Placeholder3_2,
    kIMaxAccum_3,
    kSlot3Placeholder1_3,
    kSlot3Placeholder2_3,
    kSlot3Placeholder3_3,
    kPositionConversionFactor,
    kVelocityConversionFactor,
    kClosedLoopRampRate,
    kSoftLimitFwd,
    kSoftLimitRev,
    kSoftLimitRsvd0,
    kSoftLimitRsvd1,
    kAnalogPositionConversion,
    kAnalogVelocityConversion,
    kAnalogAverageDepth,
    kAnalogSensorMode,
    kAnalogInverted,
    kAnalogSampleDelta,
    kAnalogRsvd0,
    kAnalogRsvd1,
    kDataPortConfig,
    kAltEncoderCountsPerRev,
    kAltEncoderAverageDepth,
    kAltEncoderSampleDelta,
    kAltEncoderInverted,
    kAltEncoderPositionFactor,
    kAltEncoderVelocityFactor,
    kAltEncoderRsvd0,
    kAltEncoderRsvd1,
    kExtFFGain0,
    kExtFFGain1,
    kExtFFGain2,
    kExtFFGain3,
    kExtFFReserved0,
    kExtFFReserved1
};


class SparkMaxMC : public CANDevice {
    private:
        // method for generating the can frame ID for a spark max
        // motor controller based on its api class and index
        //
        // Params:
        //    apiClass - the API Class integer from the spark max docs
        //    apiIndex - the API Index integer from the spark max docs
        // Return:
        //    uint32_t - the can frame id
        uint32_t getCanFrameId(int apiClass, int apiIndex);


        // prints tothe terminal the corresponding id information based on the frc
        // protocal for a given frame
        //
        // Params:
        //    canFrameId - the id field of the can frame
        //    data       - array that contains the data sent with the frame
        // Return:
        //    None
        void debugIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]);


    public:
        // default ctor. Does not initialize anything about the device and this
        // should be done later if the instance is to be used successfully
        //
        // Params:
        //    None
        // Return:
        //    The new instance
        SparkMaxMC();


        // ctor. Initializes connection and the device id
        //
        // Params:
        //    connection  - a reference to the connection instance for the CAN network
        //    canDeviceId - the id to use for this motor controller. Can be set using
        //                  the REV Hardware client
        // Return:
        //    The new instance
        SparkMaxMC(CANConnection& connection, int canDeviceId);




        // Takes an incoming CAN Frame and responds accordingly.
        //
        // Params:
        //    canFrameId - the integer containing 29 bits that correspond to the 
        //                 id of the can frame
        //    data       - the data to be parsed
        // Return:
        //    None
        void _parseIncomingFrame(uint32_t canFrameId, uint8_t data[PACKET_LENGTH]) override;


        // (Duty Cycle Set) Sets the pwm duty cycle of the motor controller.
        //
        // Params:
        //    percent - value between [-1, 1] corresponding to the requested duty cycle
        // Return:
        //    int - if the command was sent successfully
        int dutyCycleSet(float percent);


        // (Speed Set) sets the target velocity of the motor in RPM. 
        //
        // Params:
        //    targetRPM - the new target speed in RPM
        // Return:
        //    int - if the command was sent successfully
        int velocitySet(float targetRPM);


        // (Smart Velocity Set) sets the target velocity of the motor in RPM. 
        // Honors the max acceleration and max velocity from smart motion 
        // parameters at the firmware level
        //
        // Params:
        //    targetRPM - the new target speed in RPM
        // Return:
        //    int - if the command was sent successfully
        int smartVelocitySet(float targetRPM);


        // (Voltage Set) Sets the closed loop speed controller where the
        // target voltage is in volts
        //
        // Params:
        //    targetVoltage: - the target voltage in units of volts
        // Return:
        //    int - if the command was sent successfully     
        int voltageSet(float targetVoltage);


        // (Position Set) Sets the closed loop speed controller where the
        // target position is in rotations
        //
        // Params:
        //    targetRotations: - the target position in units of rotations
        // Return:
        //    int - if the command was sent successfully       
        int positionSet(float targetRotations);


        // (Smart Motion Set) Sets the closed loop smart motion 
        // controller where the target position is in rotations
        //
        // Params:
        //    targetRotations: - the target position in rotations
        // Return:
        //    int - if the command was sent successfully  
        int smartPositionSet(float targetRotations);


        // (Parameter Access) Sets a generic parameter on the sparkmax. Must supply a the
        // correct packet structure
        // 
        // Params:
        //    param  - the parameter to write, of enumerated type
        //    packet - what to send to the sparkmax. Up to the caller
        //             to determine how this should be interpreted (read the docs)
        // Return:
        //    int - if the command was sent successfully
        int setGenericParameter(E_SPARKMAX_PARAM param, uint8_t packet[PACKET_LENGTH]);


        // (Parameter Access) reads a parameter from the sparkmax and places the output value into 
        // the array. Returns 0 if found a response within the maximum allowed
        // time. This function has an optional timeout because this operation
        // could spend a lot of time holding a mutex somewhere if it doesn't get a 
        // response immediately. 
        //
        // Params:
        //    param      - the parameter to read of enumerated type
        //    response   - where to place the response if any. Up to the caller
        //                 to determine how this should be interpreted (read the docs)
        //    timeout_ms - (optional) max time to spend in this function
        // Return:
        //    int  - 0 if response was received
        int readGenericParameter(E_SPARKMAX_PARAM param, uint8_t response[PACKET_LENGTH], int timeout_ms=100);
        

        // (Telemetry Update Mechanical Position Enoder Port) Zeros
        // the encoder value by updating the position held by the 
        // motor controller.
        //
        // Params:
        //    None
        // Return:
        //    int - if the command was sent successfully          
        int tareEncoder();
        

        // (Identify) Causes the motor controller LED to flash
        // rapidly so it can be identified
        //
        // Params:
        //    None:
        // Return;
        //    int - if the command was sent successfully  
        int identify();


};




#endif
