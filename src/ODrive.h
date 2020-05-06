#ifndef ODRIVE_TFG_ADRIA
#define ODRIVE_TFG_ADRIA


//600000 works fine too.
#define REBOOT_SLEEP_TIME 750000

#include <libusb-1.0/libusb.h>
#include "motor.h"
#include "channel.h"
#include "json.hpp"
#include "robot.h"
#include <unistd.h>

class Robot;

class ODrive{
public:

    ODrive(Channel *channel, Robot *pRobot);
    ~ODrive();

    Motor *m0;
    Motor *m1;

    uint64_t serialNumber;

    int readSerialNumber(uint64_t &serial_number);

    void setBrakeResistance(float brake_resistance);
    void configureMotor(Motor *m);
    float inputVoltage();

    void getJSON();

    void saveConfiguration();

    void reboot();

    void dontFreeDeviceHandle();

    void setNewChannel(Channel *pChannel);
    void requested_state(int newState);

    void calibrateMotorsAndEncoders();

    void configStartupSequence();

private:
    float brake_resistance;


    Channel *channel;
    bool freeDeviceHandle;
    Robot *robot;

    float getPosEstimate(MOTOR_NAME m);
};

#endif //ODRIVE_TFG_ADRIA