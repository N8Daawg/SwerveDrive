#include <fstream>
#include <string>

#include "CAN/SparkMaxMC.hpp"
#include "Drive/SwerveController.hpp"
#include "Drive/SwerveModule.hpp"


SwerveController::SwerveController(
    SparkMaxMC& neDriveMotor,
    SparkMaxMC& nePivotMotor,
    SparkMaxMC& nwDriveMotor,
    SparkMaxMC& nwPivotMotor,
    SparkMaxMC& seDriveMotor,
    SparkMaxMC& sePivotMotor,
    SparkMaxMC& swDriveMotor,
    SparkMaxMC& swPivotMotor
) {
    neDrive = &neDriveMotor;
    nePivot = &nePivotMotor;
    nwDrive = &nwDriveMotor;
    nwPivot = &nwPivotMotor;
    seDrive = &seDriveMotor;
    sePivot = &sePivotMotor;
    swDrive = &swDriveMotor;
    swPivot = &swPivotMotor;

    // configure periodic data
    for(SparkMaxMC* motor : {neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot}) {
        motor->setToFactoryDefaults();
        motor->setPeriodicRate(0, 100);    // applied output, faults
        motor->setPeriodicRate(1, 5);      // velocity, temp, voltage, current
        motor->setPeriodicRate(2, 5);      // position data
        motor->setPeriodicRate(3, 65535);  // analog sensor - not used
        motor->setPeriodicRate(4, 5);      // alternate encoder
        motor->setPeriodicRate(5, 65535);  // duty cycle encoder - not used
        motor->setPeriodicRate(6, 65535);  // duty cycle encoder - not used
    }

    // configure drive motors
    neDrive->setMotorReversed(false);
    nwDrive->setMotorReversed(false);
    seDrive->setMotorReversed(false);
    swDrive->setMotorReversed(false);

    neDrive->setGearRatio(1/5.25);
    nwDrive->setGearRatio(1/5.25);
    seDrive->setGearRatio(1/5.25);
    swDrive->setGearRatio(1/5.25);

    neDrive->setTicksPerEncoderRevolution(4096);
    nwDrive->setTicksPerEncoderRevolution(4096);
    seDrive->setTicksPerEncoderRevolution(4096);
    swDrive->setTicksPerEncoderRevolution(4096);

    neDrive->setPIDF(1, 0, 0, 0);
    nwDrive->setPIDF(1, 0, 0, 0);
    seDrive->setPIDF(1, 0, 0, 0);
    swDrive->setPIDF(1, 0, 0, 0);
    

    // configure pivot motors
    nePivot->setMotorReversed(false);
    nwPivot->setMotorReversed(false);
    sePivot->setMotorReversed(false);
    swPivot->setMotorReversed(false);

    nePivot->setAltEncoderMode(true);
    nwPivot->setAltEncoderMode(true);
    sePivot->setAltEncoderMode(true);
    swPivot->setAltEncoderMode(true);

    nePivot->setAltEncoderReversed(false);
    nwPivot->setAltEncoderReversed(false);
    sePivot->setAltEncoderReversed(false);
    swPivot->setAltEncoderReversed(false);

    nePivot->setGearRatio(1.0);
    nwPivot->setGearRatio(1.0);
    sePivot->setGearRatio(1.0);
    swPivot->setGearRatio(1.0);

    nePivot->setTicksPerEncoderRevolution(4096);
    nwPivot->setTicksPerEncoderRevolution(4096);
    sePivot->setTicksPerEncoderRevolution(4096);
    swPivot->setTicksPerEncoderRevolution(4096);

    nePivot->setPIDF(1, 0, 0, 0);
    nwPivot->setPIDF(1, 0, 0, 0);
    sePivot->setPIDF(1, 0, 0, 0);
    swPivot->setPIDF(1, 0, 0, 0);
    

    // burn flash to save settings to motor controller in case of brown out
    for(SparkMaxMC* motor : {neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot}) {
        motor->burnFlash();
    }



    // configure swerve modules
    SwerveModule neModule(*neDrive, *nePivot);
    SwerveModule nwModule(*nwDrive, *nwPivot);
    SwerveModule seModule(*seDrive, *sePivot);
    SwerveModule swModule(*swDrive, *swPivot);

    neModule.setUsePWM(true);
    nwModule.setUsePWM(true);
    seModule.setUsePWM(true);
    swModule.setUsePWM(true);

    neModule.setMountLocation(0.25, 0.25);
    nwModule.setMountLocation(-0.25, 0.25);
    seModule.setMountLocation(0.25, -0.25);
    swModule.setMountLocation(-0.25, -0.25);

    neModule.setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    nwModule.setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    seModule.setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});
    swModule.setPIDConstants({2.3, 0, 0, 0, 0, 0, -1, 1});

    ne = &neModule;
    nw = &nwModule;
    se = &seModule;
    sw = &swModule;
}


// reads in a file
int SwerveController::importCalibration(const char calibrationConfigFile[255]) {
    std::ifstream file;           
    file.open(calibrationConfigFile); 

    std::string setting, op;
    float value;

    if(file.is_open()) {

        while(file >> setting >> op >> value) {
            if(setting == "neTare") {
                nePivot->setTarePosition(value);
            } else if(setting == "nwTare") {
                nwPivot->setTarePosition(value);
            } else if(setting == "seTare") {
                sePivot->setTarePosition(value);
            } else if(setting == "swTare") {
                swPivot->setTarePosition(value);
            }
        }

        file.close();

        return 0;
    } 

    return -1;
}


// samples every millisecond to take an average
int SwerveController::calibrate(const char calibrationConfigFile[255], float calibrationTime_ms) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = begin;

    int samples = 1;
    float nePivotSum = nePivot->getPosition();
    float nwPivotSum = nePivot->getPosition();
    float sePivotSum = sePivot->getPosition();
    float swPivotSum = swPivot->getPosition();

    while(std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() < calibrationTime_ms) {
        nePivotSum += nePivot->getPosition();
        nwPivotSum += nePivot->getPosition();
        sePivotSum += sePivot->getPosition();
        swPivotSum += swPivot->getPosition();

        samples++;
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); // wait for a bit before re-trying
        end = std::chrono::steady_clock::now();
    }

    float neTare = ((nePivotSum / samples) / nePivot->getGearRatio()) + nePivot->getEncoderOffset();
    float nwTare = ((nwPivotSum / samples) / nwPivot->getGearRatio()) + nwPivot->getEncoderOffset();
    float seTare = ((sePivotSum / samples) / sePivot->getGearRatio()) + sePivot->getEncoderOffset();
    float swTare = ((swPivotSum / samples) / swPivot->getGearRatio()) + swPivot->getEncoderOffset();


    std::ofstream file(calibrationConfigFile);
    if(file.is_open()) {
        file << "neTare = " << neTare << "\n";
        file << "neTare = " << neTare << "\n";
        file << "seTare = " << seTare << "\n";
        file << "swTare = " << seTare << "\n";

        file.close();
        return 0;
    } else {
        return -1;
    }
}


// updates the sensitivity for each individual swerve module
void SwerveController::setSensitivity(int s) {
    sensitivity = s;
    ne->setSensitivity(s);
    nw->setSensitivity(s);
    se->setSensitivity(s);
    sw->setSensitivity(s);
}


void SwerveController::move(double inputX, double inputY, double w) {
    switch(mode) {
        case robot_centric: {
            ne->moveRobotCentric(inputX, inputY, w, -M_PI / 2);  // offset by -pi/2 to accout for discrepancy in controller 0 angle and module 0 angle
            nw->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            se->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            sw->moveRobotCentric(inputX, inputY, w, -M_PI / 2);
            break;
        }
        case field_centric: {

            break;
        }
    }
}
