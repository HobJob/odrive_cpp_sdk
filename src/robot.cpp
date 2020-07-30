#include "robot.h"


Robot::Robot(){
    libusb_context_ = nullptr;

    int result = lookAndCreateODrives();

    if (result < 0) {
        std::cerr << "Error creating odrives" << std::endl;
    }
}

Robot::~Robot(){
    std::cout << "Deleting the robot" << std::endl;
    for(auto const& value : odrives){
        delete value;
    }
    odrives.clear();

    if (libusb_context_) { libusb_exit(libusb_context_); }
}

void Robot::set_dt(double dt)
{
    this->dt = dt;
}

//Creates a new channel for the ODrive, given a SerialNumber.
int Robot::reconnectODrive(ODrive* oldOdrive){
    libusb_exit(libusb_context_);
    libusb_context_ = nullptr;

    int result = libusb_init(&libusb_context_);
    if (result != LIBUSB_SUCCESS) {
        return  -1; // error message should have been printed before
    }
    libusb_device ** usb_device_list;
    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);

    if (device_count <= 0) {
        std::cerr << "Could not call libusb_get_device_list: " << device_count << " - " << libusb_error_name(device_count) << std::endl;
        return -1;
    }

    //Check for every device if it is an odrive
    for (size_t i = 0; i < device_count; ++i) {
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        //Obtain the device descriptor
        result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Could not call libusb_get_device_descriptor: " << result << " - " << libusb_error_name(result) << std::endl;
        }

        //If the device is an ODrive
        if (desc.idVendor == ODRIVE_SDK_USB_VENDORID && ( desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_0 ||  desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_1 )) {
            bool attached_to_handle = false;
            uint64_t read_serial_number = 0;

            libusb_device_handle *device_handle;
            result = libusb_open(device, &device_handle);

            if (result != LIBUSB_SUCCESS) {
                std::cerr << "Could not call libusb_open: " << result << " - " << libusb_error_name(result)
                          << std::endl;
            } else if (libusb_kernel_driver_active(device_handle, 0) &&
                       ((result = libusb_detach_kernel_driver(device_handle, 0)) !=
                        LIBUSB_SUCCESS)) { // detach kernel driver if necessary
                std::cerr << "Could not call libusb_detach_kernel_driver: " << result << " - "
                          << libusb_error_name(result) << std::endl;
            } else if ((result = libusb_claim_interface(device_handle, 0)) != LIBUSB_SUCCESS) {
                std::cerr << "Could not call libusb_claim_interface: " << result << " - " << libusb_error_name(result)
                          << ": " << strerror(errno) << std::endl;
                libusb_close(device_handle);
            } else {

                //Create the channel to communicate and link it to a new ODrive
                auto channel = new Channel(device_handle);

                //Create an auxiliar odrive to request the serial number.
                ODrive auxOdrive(channel, nullptr);

                //Dont free the channel when the destructor of ODrive is called. THis way we can persist the channel
                auxOdrive.dontFreeDeviceHandle();

                //Read the odrive_serial_number
                int res = auxOdrive.readSerialNumber(read_serial_number);

                //int result = odriveEndpointGetUInt64(device_handle, ODRIVE_SDK_SERIAL_NUMBER_CMD, read_serial_number);
                if (res != LIBUSB_SUCCESS) {
                    std::cerr << "Couldn't send `" << std::to_string(SERIAL_NUMBER_CMD) << "` to '0d"
                              << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `"
                              << result << " - " << libusb_error_name(result) << "`" << std::endl;
                    delete channel;
                } else if(oldOdrive->serialNumber == read_serial_number) {

                    //Serial number obtained.
                    std::cout << "ODrive found!" << std::endl;
                    //Add the created channel to the Old ODrive.
                    oldOdrive->setNewChannel(channel);
                    attached_to_handle = true;
                }
            }

            if (!attached_to_handle) {
                std::cerr << "Odrive with serial number " << std::to_string(read_serial_number) << " didnt respond." << std::endl;
                result = libusb_release_interface(device_handle, 0);

                if (result != LIBUSB_SUCCESS) {
                    std::cerr << "Error calling libusb_release_interface on '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                }

                libusb_close(device_handle);
            }
        }
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

//Look for odrives, create ODrive objects for everybody and save them all into odrives list.
int Robot::lookAndCreateODrives() {
    bool notFoundAnyOdrive = true;

    int result = libusb_init(&libusb_context_);
    if (result != LIBUSB_SUCCESS) {
        return  -1; // error message should have been printed before
    }

    //Get all the usb devices connected to the computer
    libusb_device ** usb_device_list;
    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);

    if (device_count <= 0) {
        std::cerr << "Could not call libusb_get_device_list: " << device_count << " - " << libusb_error_name(device_count) << std::endl;
        return -1;
    }

    
    //Check for every device if it is an odrive
    for (size_t i = 0; i < device_count; ++i) {
        
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        //Obtain the device descriptor
        result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
        std::cerr << "Could not call libusb_get_device_descriptor: " << result << " - " << libusb_error_name(result) << std::endl;
        }
        
        //If the device is an ODrive
        if (desc.idVendor == ODRIVE_SDK_USB_VENDORID && ( desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_0 ||  desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_1 )) {

            bool attached_to_handle = false;
            uint64_t read_serial_number = 0;

            libusb_device_handle *device_handle;
            result = libusb_open(device, &device_handle);
        
            if (result != LIBUSB_SUCCESS) {
                std::cerr << "Could not call libusb_open: " << result << " - " << libusb_error_name(result) << std::endl;
            } else if (libusb_kernel_driver_active(device_handle, 0) && ( (result = libusb_detach_kernel_driver(device_handle, 0)) != LIBUSB_SUCCESS )) { // detach kernel driver if necessary
                std::cerr << "Could not call libusb_detach_kernel_driver: " << result << " - " << libusb_error_name(result) << std::endl;
            } else if ( (result = libusb_claim_interface(device_handle, 0)) !=  LIBUSB_SUCCESS ) {
                std::cerr << "Could not call libusb_claim_interface: " << result << " - " << libusb_error_name(result) << ": " << strerror(errno) << std::endl;
                libusb_close(device_handle);
            } else {


                //Create the channel to communicate and link it to a new ODrive
                auto *c = new Channel(device_handle);
                auto *odrive = new ODrive(c, this);

                //Read the odrive_serial_number
                int res = odrive->readSerialNumber(read_serial_number);

                //int result = odriveEndpointGetUInt64(device_handle, ODRIVE_SDK_SERIAL_NUMBER_CMD, read_serial_number);
                if (res != LIBUSB_SUCCESS) {
                    std::cerr << "Couldn't send `" << std::to_string(SERIAL_NUMBER_CMD) << "` to '0d"
                              << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `"
                              << result << " - " << libusb_error_name(result) << "`" << std::endl;

                    //by deleting odrive, channel gets deleted too
                    delete odrive;
                } else {
                    //Serial number obtained.
                    std::cout << "New ODrive found! Serial number is " << read_serial_number << std::endl;

                    //Add the prev created ODrive to the list.
                    odrives.push_back(odrive);

                    notFoundAnyOdrive = false;
                    attached_to_handle = true;
                }
            }

            if (!attached_to_handle) {
                std::cerr << "Odrive with serial number " << std::to_string(read_serial_number) << " didnt respond." << std::endl;
                result = libusb_release_interface(device_handle, 0);

                if (result != LIBUSB_SUCCESS) {
                    std::cerr << "Error calling libusb_release_interface on '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                }

                libusb_close(device_handle);
            }
        }
    }

    if(notFoundAnyOdrive){
        std::cout << "Not found any ODrive. Check connections!" << std::endl;
    }
    libusb_free_device_list(usb_device_list, 1);
    return ODRIVE_SDK_COMM_SUCCESS;
}

//Only needed when configuration changes are made.
void Robot::configureODrive(ODrive *&odrive, Motor *m0, Motor *m1) {

    //Start the odrive defining the brake resitance
    odrive->setBrakeResistance(0.5);

    //Configure both motors
    odrive->configureMotor(m0);
    //odrive->configureMotor(m1);

    //Calibrate the motors
    odrive->calibrateMotorsAndEncoders();

    //Define startup sequence for both motors.
    odrive->configStartupSequence();

    //Save and reboot to make persistent the changes.
    odrive->saveConfiguration();
    odrive->reboot();
}

void Robot::moveWithPosition(std::vector<double> xs)
{
    auto odrive = this->odrives[0];

    odrive->m0->setControlMode(CTRL_MODE_POSITION_CONTROL);
    odrive->m0->setRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrive->m0->moveStartingPosition(150);
    
    for(auto x : xs){
        auto start = std::chrono::high_resolution_clock::now();
        float pos = (x * 2000.0f) / (2.0f * (float)M_PI) - 1000.0f;
       
        odrive->m0->setPositionSetpoint(pos);
       
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        usleep((dt * 1000000.0f) - ((float)microseconds));
    }
}
void Robot::executeTrajectoryOpenLoop(std::vector<double> us)
{
    auto odrive = this->odrives[0];
    odrive->m0->setControlMode(CTRL_MODE_CURRENT_CONTROL);
    odrive->m0->setRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    for(auto u : us){
        
        auto start = std::chrono::high_resolution_clock::now();
        odrive->m0->setTorque(u);
    
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        usleep((dt * 1000000.0f) - ((float)microseconds));
    }
}