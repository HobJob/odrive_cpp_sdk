#ifndef ROBOT_TFG_ADRIA
#define ROBOT_TFG_ADRIA

#include <libusb-1.0/libusb.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <endian.h>

#include "channel.h"
#include "ODrive.h"
#include <chrono>

class ODrive;

enum robot_type {SINGLE_PENDULUM, DOUBLE_PENDULUM};

class Robot{
public:
    Robot();
    Robot(robot_type type, double dt);
    ~Robot();

    std::vector<ODrive*> odrives;

    int reconnectODrive(ODrive *oldOdrive);

    void configureODrive(ODrive *&pDrive, Motor *pMotor, Motor *pMotor1);
    
    void moveWithCurrent(std::vector<double> us);
    void moveWithPosition(std::vector<double> xs);
    void set_dt(double dt);

private:
    
    double dt;

    //USB stuff
    libusb_context* libusb_context_;

    robot_type pendulum_type;
    

    int lookAndCreateODrives();
};


#endif //ROBOT_TFG_ADRIA