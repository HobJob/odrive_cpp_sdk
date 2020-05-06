#ifndef ROBOT_TFG_ADRIA
#define ROBOT_TFG_ADRIA

#include <libusb-1.0/libusb.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <endian.h>

#include "channel.h"
#include "ODrive.h"

class ODrive;

class Robot{
public:
    Robot();
    ~Robot();

    std::vector<ODrive*> odrives;

    int reconnectODrive(ODrive *oldOdrive);

    void configureODrive(ODrive *&pDrive, Motor *pMotor, Motor *pMotor1);

private:

    //USB stuff
    libusb_context* libusb_context_;

    int lookAndCreateODrives();


};


#endif //ROBOT_TFG_ADRIA