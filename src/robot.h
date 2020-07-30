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

class Robot{
public:
    Robot();
    ~Robot();

    std::vector<ODrive*> odrives;

    int reconnectODrive(ODrive *oldOdrive);

    void configureODrive(ODrive *&pDrive, Motor *pMotor, Motor *pMotor1);
    
    void executeTrajectoryOpenLoop(std::vector<double> us);
    void moveWithPosition(std::vector<double> xs);
    void set_dt(double dt);

private:

    //USB stuff
    libusb_context* libusb_context_;
    double dt;

    int lookAndCreateODrives();
};


#endif //ROBOT_TFG_ADRIA