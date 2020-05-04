#ifndef ROBOT_TFG_ADRIA
#define ROBOT_TFG_ADRIA

#include <libusb-1.0/libusb.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <endian.h>

#include "channel.h"
#include "ODrive.h"

class Robot{
public:
    Robot();
    ~Robot();

    std::vector<ODrive*> odrives;

private:

    //USB stuff
    libusb_context* libusb_context_;

    int lookAndCreateODrives();

};


#endif //ROBOT_TFG_ADRIA