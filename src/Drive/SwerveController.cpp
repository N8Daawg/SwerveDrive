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

    nePivot->setTicksPerEncoderRevolution(42);
    nwPivot->setTicksPerEncoderRevolution(42);
    sePivot->setTicksPerEncoderRevolution(42);
    swPivot->setTicksPerEncoderRevolution(42);

    nePivot->setPIDF(1, 0, 0, 0);
    nwPivot->setPIDF(1, 0, 0, 0);
    sePivot->setPIDF(1, 0, 0, 0);
    swPivot->setPIDF(1, 0, 0, 0);
    

    // burn flash
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
