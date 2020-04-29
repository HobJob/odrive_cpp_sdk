#ifndef ROBOT_TFG_ADRIA
#define ROBOT_TFG_ADRIA

#include <libusb-1.0/libusb.h>
#include <vector>


typedef std::vector<libusb_device_handle *> odrive_device_handles;

enum MOTOR_NAME{
    M0,
    M1
};

//TODO: Maybe more indicated to rename it to Axis.
class Motor{
public:
    Motor(MOTOR_NAME name, int encoder_cpr = 2000, int encoder_max_rpm = 14500,
        uint8_t pole_pairs = 12,
        int max_rpm = 7920,
        int max_current = 25,
        int kV = 330,
        int calibration_current = 5,
        int current_limit = 5);
    ~Motor();

    MOTOR_NAME name;
    
    int encoder_cpr; //2000
    int encoder_max_rpm; //14500

    int pole_pairs; //12
    int max_rpm; //7920
    int max_current; //25
    int kV; //330
    int calibration_current; //5
    int current_limit; //5

};

class ODrive{
public:
    ODrive(libusb_device_handle *handle);
    ~ODrive();
    
    Motor m0;
    Motor m1;
    
    void init(float brake_resistance);
    
    void addMotor(Motor m);
    
    float inputVoltage();

private:
    libusb_device_handle *handle;
    
};

class Robot{
public:
    Robot();
    ~Robot();
    
    //odrive_handles_ = new libusb_device_handle*[num_odrives_] {NULL};
    std::vector<ODrive> odrives;

    odrive_device_handles lookForOdrives();
    void createODrives(odrive_device_handles handles);

};


#endif //ROBOT_TFG_ADRIA