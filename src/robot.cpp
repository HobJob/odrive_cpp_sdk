#include "robot.h"

Robot::Robot(){
    libusb_context_ = nullptr;


    int result = lookAndCreateODrives();

    if (result < 0) {
        std::cerr << "BIG PAPA" << std::endl;
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
        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
        std::cerr << "Could not call libusb_get_device_descriptor: " << result << " - " << libusb_error_name(result) << std::endl;
        }
        
        //If the device is an ODrive
        if (desc.idVendor == ODRIVE_SDK_USB_VENDORID && ( desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_0 ||  desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_1 )) {
            
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
                
                //Read the odrive_serial_number
                bool attached_to_handle = false;
                uint64_t read_serial_number = 0;

                //Create the channel to communicate and link it to a new ODrive
                auto *c = new Channel(device_handle);
                auto *odrive = new ODrive(c);

                int res = odrive->getSerialNumber(read_serial_number);

                //int result = odriveEndpointGetUInt64(device_handle, ODRIVE_SDK_SERIAL_NUMBER_CMD, read_serial_number);
                if (res != LIBUSB_SUCCESS) {
                    std::cerr << "Couldn't send `" << std::to_string(ODRIVE_SDK_SERIAL_NUMBER_CMD) << "` to '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                } else {
                    //Serial number obtained.
                    std::cout << "New ODrive found! Serial number is " << read_serial_number << std::endl;

                    //Add the prev created ODrive to the list.
                    odrives.push_back(odrive);

                    notFoundAnyOdrive = false;
                    attached_to_handle = true;
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
    }

    if(notFoundAnyOdrive){
        std::cout << "Not found any ODrive. Check connections!" << std::endl;
    }
    libusb_free_device_list(usb_device_list, 1);
    return ODRIVE_SDK_COMM_SUCCESS;
}
