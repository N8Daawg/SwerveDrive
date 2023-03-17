// SwerveController.hpp
// Contains code for controlling a standard swerve bot. Configuration
// for all the motors and swerve modules occurs in the constructor
//
// Author: Aiden Carney

#ifndef SWERVECONTROLLER_HPP
#define SWERVECONTROLLER_HPP

#include "SwerveModule.hpp"
#include "CAN/SparkMaxMC.hpp"


class SwerveController {
    public:
        enum drive_mode {
            robot_centric,
            field_centric
        };

        // ctor - constructs the swerve drive object based on the motors.
        // Performs configuration steps
        //
        // Params:
        //    xxDriveMotor - reference to the given drive motor (big NEO)
        //    xxPivotMotor - reference to the given pivot motor (little NEO)
        SwerveController(
            SparkMaxMC& neDriveMotor,
            SparkMaxMC& nePivotMotor,
            SparkMaxMC& nwDriveMotor,
            SparkMaxMC& nwPivotMotor,
            SparkMaxMC& seDriveMotor,
            SparkMaxMC& sePivotMotor,
            SparkMaxMC& swDriveMotor,
            SparkMaxMC& swPivotMotor
        );


        // Sets the sensitivity of the input. This is a simple constant multiplied
        // to the output of the velocity controller for the drive motors
        //
        // Params:
        //    s - the new sensitivity
        // Return:
        //    None
        void setSensitivity(int s);


        // sets the new drive mode for the swerve drivetrain
        //
        // Params:
        //    newMode - the new drive mode
        // Return:
        //    None
        void setDriveMode(drive_mode newMode) {mode = newMode;}


        // Moves the drivetrain based on the current mode and the
        // given inputs
        //
        // Params:
        //    inputX - the normalised x input
        //    inputY - the normalised y input
        //    w      - the normalised omega input
        void move(double inputX, double inputY, double w);

    private:
        SparkMaxMC* neDrive;
        SparkMaxMC* nePivot;
        SparkMaxMC* nwDrive;
        SparkMaxMC* nwPivot;
        SparkMaxMC* seDrive;
        SparkMaxMC* sePivot;
        SparkMaxMC* swDrive;
        SparkMaxMC* swPivot;

        SwerveModule* ne; 
        SwerveModule* nw;
        SwerveModule* se;
        SwerveModule* sw;

        double sensitivity = 1.0;
        drive_mode mode = robot_centric;
};


#endif
