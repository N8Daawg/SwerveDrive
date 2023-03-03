#include <iostream>

#include "Drive/SwerveModule.hpp"
#include "util/misc.hpp"


// ctor - sets references to motors
SwerveModule::SwerveModule(SparkMaxMC& drive, SparkMaxMC& pivot) {
    driveMotor = &drive;
    pivotMotor = &pivot;
}


// uses vector math to update the targets
void SwerveModule::updateTargets(double inputX, double inputY, double w) {
    cartesian_vector strafeContrib = {inputX, inputY, 0.0};  // contribution to velocity from strafing vector

    cartesian_vector omega = {0.0, 0.0, -w};  // from RHR clockwise means negative z direction
    cartesian_vector position = {xPos_m, yPos_m, 0.0};
    cartesian_vector turnContrib = cross_product(omega, position);  // contribution to velocity from turning vector (w x r)
    
    // normalize turn contrib vector
    // from cross product |x| = |(y1 * z2) - (z1 * y2)|
    // we are taking omega cross position so y1 * z2 will always be 0, 
    // thus the max this term can be is omega=1 times the y position (yPos_m)
    // A similar argument can be made for the maximum y value of this vector,
    // which will just be the x position (xPos_m)
    scale(turnContrib.x, -1.0, 1.0, -yPos_m, yPos_m);
    scale(turnContrib.y, -1.0, 1.0, -xPos_m, xPos_m);

    cartesian_vector target = add_vectors(strafeContrib, turnContrib);  // add the contributions

    // normalize target vector
    // because both bectors are already normalized, the maximum range of values before
    // normalization is [-2, 2], which we want to be [-1, 1]
    scale(target.x, -1.0, 1.0, -2.0, 2.0);
    scale(target.y, -1.0, 1.0, -2.0, 2.0);

    // update the target velocities
    targetVx = target.x;
    targetVy = target.y;

}


// simple setter function
void SwerveModule::updateMountLocation(double x, double y) {
    xPos_m = x;
    yPos_m = y;
}


void SwerveModule::move(double inputX, double inputY, double w) {
    updateTargets(inputX, inputY, w);

    std::cout << "Target Vx: " << targetVx << "    Target Vy: " << targetVy << "\n";
}
