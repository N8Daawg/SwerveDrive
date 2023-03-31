#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "CAN/CANNetwork.hpp"
#include "CAN/CANConnection.hpp"
#include "CAN/SparkMaxMC.hpp"
#include "CAN/can_utils.hpp"
#include "Drive/SwerveModule.hpp"
#include "Drive/SwerveController.hpp"
#include "Joystick/mtJoystick.hpp"
#include "util/misc.hpp"


int main() {        
    // char* jsSource = "/dev/input/js0";
    // if (!startDeviceConnection(jsSource)) {
    //     printf("ERROR: joystick could not be initialized.\n");
    //     return 0;
    // }


    // short axis1 = 0;
    // short axis2 = 0;
    // short axis3 = 0;
    // short axis4 = 0;

    // for(int i = 0; i < 10000; i++) {
    //     getAxisValue(0, &axis1);
    //     getAxisValue(0, &axis2);
    //     getAxisValue(0, &axis3);
    //     getAxisValue(0, &axis4);
    //     std::cout << axis1 << " " << axis2 << " " << axis3 << " " << axis4 << "\n";
    //     sleep(1);
    // }



    // if (!startDeviceConnection("/dev/input/js0")) exit(1);
    // printJoystickInformation();

    setCalibrationCoefficients(0, 0, 0, 0, 0, 255);


    CANConnection canConnection("can0");


    /*********************************************************************/
    /*                                                                   */
    /*                    Full Drive Section                             */
    /*                                                                   */
    /*********************************************************************/

    SparkMaxMC nwDrive(canConnection, 1);
    SparkMaxMC nwPivot(canConnection, 2);
    SparkMaxMC neDrive(canConnection, 3);
    SparkMaxMC nePivot(canConnection, 4);
    SparkMaxMC seDrive(canConnection, 5);
    SparkMaxMC sePivot(canConnection, 6);
    SparkMaxMC swDrive(canConnection, 7);
    SparkMaxMC swPivot(canConnection, 8);

    CANNetwork canNetwork(canConnection);  // start the heartbeat signal
    canNetwork.addDevice(neDrive);
    canNetwork.addDevice(nePivot);
    canNetwork.addDevice(nwDrive);
    canNetwork.addDevice(nwPivot);
    canNetwork.addDevice(seDrive);
    canNetwork.addDevice(sePivot);
    canNetwork.addDevice(swDrive);
    canNetwork.addDevice(swPivot);

    SwerveController drive(neDrive, nePivot, nwDrive, nwPivot, seDrive, sePivot, swDrive, swPivot, false);

    canNetwork.startHearbeat();


    /*********************************************************************/
    /*                                                                   */
    /*                    2 Motor Debug Section                          */
    /*                                                                   */
    /*********************************************************************/

    // SparkMaxMC motor1(canConnection, 1);
    // SparkMaxMC motor2(canConnection, 2);

    // motor1.dutyCycleSet(0);
    // motor2.dutyCycleSet(0);

    // motor1.setToFactoryDefaults();
    // motor2.setToFactoryDefaults();
    // motor2.setAltEncoderMode(false);

    // motor1.setGearRatio(1/5.25);
    // motor1.setTicksPerEncoderRevolution(4096);

    // motor2.setGearRatio(1/(5.3333333 * 10));
    // motor2.setTicksPerEncoderRevolution(42);

    // motor1.setMotorReversed(true);
    // motor2.setMotorReversed(true);

    // motor1.burnFlash();
    // motor2.burnFlash();

    
    // CANNetwork canNetwork(canConnection);  // start the heartbeat signal
    // canNetwork.addDevice(motor1);
    // canNetwork.addDevice(motor2);
    // canNetwork.startHearbeat();

    // sleep(200000);  // wait a little bit for everything to start up

    // motor1.tareEncoder();
    // motor2.tareEncoder();

    // SwerveModule s1(motor1, motor2);
    // s1.setMountLocation(-0.5, 0.5);
    // s1.setUsePWM(true);
    // s1.setSensitivity(0.35);



    // std::cout << "Starting calibration\n";
    // drive.calibrate("config.txt", 1000);
    // std::cout << "Calibration finished\n";
    drive.importCalibration("config.txt");
    drive.setSensitivity(0.3);
    std::cout << "Calibration Set\n";


    while(1) {
        // handleJoystickEvents();

        short lx = 0;
        short ly = 0;
        short rx = 0;
        short ry = 0;
        // getAxisValue(0, &lx);
        // getAxisValue(1, &ly);
        // getAxisValue(3, &rx);
        // getAxisValue(4, &ry);


        double x = (lx - 127) / 127.0;
        double y = (ly - 127) / -127.0;  // axis is reversed, multiply by -1
        double w = (rx - 127) / 127.0;

        //std::cout << "lx: " << lx << " ly: " << ly << " rx: " << rx << " ry: " << ry << "\n";
        // std::cout << "x: " << x << " y: " << y << " w: " << w << "\n";

        // s1.moveRobotCentric(x, y, w, -M_PI / 2);
        // s1.moveRobotCentric(0, 0.5, 0, -M_PI / 2);

        // drive.move(x, y, w);
        drive.move(0, 0.5, 0);

        sleep(5000);
    }




    // while(1) {
    //     s1.moveRobotCentric(1, 0, 0, -M_PI / 2);
    //     //motor1.identify();
    //     //motor2.printFaults(motor2.getFaults());
    //     //std::cout << '\r' << std::left << std::setw(20) << motor2.getAppliedOutput() << std::flush;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(2));
    // }

    return 0;
}
