// SwerveModule.hpp
// Contains class for controlling a swerve module consisting of 
// two sparkmax motors
//
// Author: Aiden Carney

#ifndef SWERVEMODULE_HPP
#define SWERVEMODULE_HPP

#include <cmath>

#include "CAN/SparkMaxMC.hpp"
#include "util/PIDController.hpp"
#include "util/VectorMath.hpp"


// Uses clockwise as positive w as convention
class SwerveModule {
    private:        
        double xPos_m;  // width of robot in meters
        double yPos_m;  // length of robot in meters

        SparkMaxMC* driveMotor;
        SparkMaxMC* pivotMotor;

        int maxDriveVelocity = 5800;
        int maxPivotVelocity = 11000;

        PIDController controller;  // the position pid controller for the pivot motor


        // updates the motor motion for each degree of freedom on the module
        // based on the inputs. Calculates where to pivot to and how fast
        // the drive motor should be spinning. The magnitude of x and y should
        // be 1
        //
        // Params:
        //    inputX      - the target x direction, normalised to [-1, 1]
        //    inputY      - the target y direction, normalised to [-1, 1]
        //    w           - the target radial velocity, normalised to [-1, 1] where 1
        //                  is move full clockwise
        // Return:
        //    None
        void moveToTarget(double inputX, double inputY, double w);


    public:

        // Ctor - sets the two motors for the swerve module
        // 
        // Params:
        //    drive - the drive motor on the module
        //    pivot - the pivot motor on the module
        // Return:
        //    the new instance
        SwerveModule(SparkMaxMC& drive, SparkMaxMC& pivot);


        // updates the position of the swerve module relative
        // to the center of the robot. The center is considered
        // (0, 0) and all measurements are in meters
        //
        // Params:
        //    x - the x position in meters
        //    y - the y position in meters
        // Return:
        //    None
        void updateMountLocation(double x, double y);


        // Moves the robot based on user input. Updates target
        // vectors and calls the motor motion commands
        //
        // Params:
        //    inputX - the target x direction, normalised to [-1, 1]
        //    inputY - the target y direction, normalised to [-1, 1]
        //    w      - the target radial velocity, normalised to [-1, 1] where 1
        //             is move full clockwise
        void moveRobotCentric(double inputX, double inputY, double w);



};


#endif