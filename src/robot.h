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

#include <Eigen/Core>
#include "Graph_Logger.h"

class ODrive;
class Graph_Logger;
class Motor;

class Robot{
public:
    Robot();
    Robot(double dt);
    ~Robot();

    std::vector<ODrive*> odrives;

    int reconnectODrive(ODrive *oldOdrive);

    void configureODrive(ODrive *&pDrive, Motor *pMotor, Motor *pMotor1);
    
    
    void executeTrajectoryOpenLoop(std::vector<Eigen::VectorXd> us, Graph_Logger *graph_logger);

    void moveWithPosition(std::vector<double> xs);
    void set_dt(double dt);

private:
    
    double dt;

    //USB stuff
    libusb_context* libusb_context_;
    
    int lookAndCreateODrives();
};


#endif //ROBOT_TFG_ADRIA