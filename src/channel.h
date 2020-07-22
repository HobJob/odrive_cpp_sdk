#ifndef CHANNEL_TFG_ADRIA
#define CHANNEL_TFG_ADRIA

#define ODRIVE_SDK_USB_VENDORID     4617 //decimal for 0x1209

#define ODRIVE_SDK_USB_PRODUCTID_0     3379 // mac
#define ODRIVE_SDK_USB_PRODUCTID_1     3378 // linux?

#define ODRIVE_SDK_PROTOCOL_VERION 1
#define ODRIVE_SDK_DEFAULT_CRC_VALUE 13145  // found with running odrivetools -v and outputting my own information
#define ODRIVE_SDK_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_SDK_TIMEOUT 1000
#define ODRIVE_SDK_MAX_RESULT_LENGTH 100
#define ODRIVE_SDK_LIBUSB_ISERIAL_LENGTH 256

#define ODRIVE_SDK_SCAN_SUCCESS 0
#define ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND 1
#define ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID 2
#define ODRIVE_SDK_UNEXPECTED_RESPONSE 3
#define ODRIVE_SDK_NOT_INITIALIZED 4
#define ODRIVE_SDK_COMM_SUCCESS 0

/*USB Endpoints*/
#define ODRIVE_SDK_WRITING_ENDPOINT 3 // found with running odrivetools -v
#define ODRIVE_SDK_READING_ENDPOINT 131 // found with running odrivetools -v

/*ENUMS FROM ODRIVE enums.py*/
#define AXIS_STATE_UNDEFINED 0
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_SENSORLESS_CONTROL 5
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define AXIS_STATE_LOCKIN_SPIN 9
#define AXIS_STATE_ENCODER_DIR_FIND 10

#define CTRL_MODE_VOLTAGE_CONTROL 0
#define CTRL_MODE_CURRENT_CONTROL 1
#define CTRL_MODE_VELOCITY_CONTROL 2
#define CTRL_MODE_POSITION_CONTROL 3
#define CTRL_MODE_TRAJECTORY_CONTROL 4

/****Self made enums from .json*****/
/*ODRIVE endpoints*/
#define CONFIG_BRAKE_RESISTANCE 28
#define VBUS_VOLTAGE_CMD 1
#define SERIAL_NUMBER_CMD 2
#define SAVE_CONFIG 355
#define REBOOT 357

/*Axis endpoints*/
#define AXIS0_CONTROLLER_POS_SETPOINT 125
#define AXIS0_CONTROLLER_CONFIG_CONTROL_MODE 131
#define AXIS0_CONTROLLER_CONFIG_VEL_LIMIT 135

#define AXIS1_CONTROLLER_POS_SETPOINT 268
#define AXIS1_CONTROLLER_CONFIG_VEL_LIMIT 278
#define AXIS1_CONTROLLER_CONFIG_CONTROL_MODE 274

/*Motor endpoints*/
#define M0_CURRENT_STATE 54
#define M0_REQUESTED_STATE 55

#define M0_MOTOR_IS_CALIBRATED 79
#define M0_CURRENT_CONTROL_P_GAIN 88
#define M0_CURRENT_CONTROL_I_GAIN 89
#define M0_CURRENT_CONTROL_IQ_SETPOINT 95
#define M0_CURRENT_CONTROL_IQ_MEASURED 96
#define M0_CONFIG_PRE_CALIBRATED 111

#define M0_CONFIG_POLE_PAIRS 112
#define M0_CONFIG_CALIBRATION_CURRENT 113
#define M0_CONFIG_MOTOR_TYPE 118
#define M0_CONFIG_CURRENT_LIM 119
#define M0_CONFIG_MOTOR_REQUESTED_CURRENT_RANGE 122

#define M0_CONTROLLER_CURRENT_SETPOINT 128
#define M0_POS_ESTIMATE 161
#define M0_ENCODER_VEL_ESTIMATE 164
#define M0_ENCODER_CONFIG_USE_INDEX 167
#define M0_ENCODER_CONFIG_PRE_CALIBRATED 169
#define M0_ENCODER_CONFIG_CPR 171

#define M1_CURRENT_STATE 197
#define M1_REQUESTED_STATE 198

#define M1_MOTOR_IS_CALIBRATED 222
#define M1_CURRENT_CONTROL_P_GAIN 231
#define M1_CURRENT_CONTROL_I_GAIN 232
#define M1_CURRENT_CONTROL_IQ_SETPOINT 238
#define M1_CURRENT_CONTROL_IQ_MEASURED 239

#define M1_CONFIG_PRE_CALIBRATED 254
#define M1_CONFIG_POLE_PAIRS 255
#define M1_CONFIG_CALIBRATION_CURRENT 256
#define M1_CONFIG_MOTOR_TYPE 261
#define M1_CONFIG_CURRENT_LIM 262
#define M1_CONFIG_MOTOR_REQUESTED_CURRENT_RANGE 265

#define M1_CONTROLLER_CURRENT_SETPOINT 271
#define M1_POS_ESTIMATE 304
#define M1_ENCODER_CONFIG_USE_INDEX 310
#define M1_ENCODER_CONFIG_PRE_CALIBRATED 312
#define M1_ENCODER_CONFIG_CPR 314
#define M1_ENCODER_VEL_ESTIMATE 329

/**Startup */
#define AXIS0_CONFIG_STARTUP_MOTOR_CALIBRATION 58
#define AXIS0_CONFIG_STARTUP_ENCODER_INDEX_SEARCH 59
#define AXIS0_CONFIG_STARTUP_ENCODER_OFFSET_CALIBRATION 60
#define AXIS0_CONFIG_STARTUP_CLOSED_LOOP_CONTROL 61
#define AXIS0_CONFIG_STARTUP_SENSORLESS_CONTROL 62

#define AXIS1_CONFIG_STARTUP_MOTOR_CALIBRATION 201
#define AXIS1_CONFIG_STARTUP_ENCODER_INDEX_SEARCH 202
#define AXIS1_CONFIG_STARTUP_ENCODER_OFFSET_CALIBRATION 203
#define AXIS1_CONFIG_STARTUP_CLOSED_LOOP_CONTROL 204
#define AXIS1_CONFIG_STARTUP_SENSORLESS_CONTROL 205





#include <iostream>
#include <string.h>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <endian.h>
#include <unistd.h>
#include <mutex>

typedef std::vector<uint8_t> commBuffer;

class Channel {

public:
    //Create a new channel to a specific ODrive
    Channel(libusb_device_handle *device_handle);
    ~Channel();

    libusb_device_handle *device_handle;

    int odriveEndpointGetUInt8(int endpoint_id, uint8_t &value);
    int odriveEndpointGetShort(int endpoint_id, short &value);
    int odriveEndpointGetInt(int endpoint_id, int &value);
    int odriveEndpointGetUInt64(int endpoint_id, uint64_t &value);
    int odriveEndpointGetFloat(int endpoint_id, float &value);
    int odriveEndpointSetFloat(int endpoint_id, const float &value);
    int odriveEndpointSetInt(int endpoint_id, const int &value);


    int getJSON(commBuffer &data);

    int odriveEndpointReset(int endpoint_id, const int &value);

    int odriveEndpointSetUInt8(int endpoint_id, const uint8_t &value);

    void waituint8(int endpoint, uint8_t newValue);
    int odriveEndpointGetUInt16(int endpoint_id, uint16_t &value);

    void waitOperationFinisheduint8(int endpoint, uint8_t newValue);
private:
    /*Adria*/

    std::mutex tx_mutex;

    short outbound_seq_no_; // unique ids for packets send to odrive

    int odriveEndpointRequest(int endpoint_id, commBuffer &received_payload, int &received_length, commBuffer payload,
                              int ack,
                              int length);

    static void serializeCommBufferInt(commBuffer& buf, const int& value);
    static void serializeCommBufferFloat(commBuffer& buf, const float& value);
    static void deserializeCommBufferInt(commBuffer& buf, int& value);
    static void appendShortToCommBuffer(commBuffer& buf, short value);
    static void readShortFromCommBuffer(commBuffer& buf, short& value);
    static void deserializeCommBufferUInt64(commBuffer& buf, uint64_t& value);
    static void deserializeCommBufferUInt8(commBuffer& buf, uint8_t& value);
    static void deserializeCommBufferFloat(commBuffer& buf, float& value);

    static commBuffer createODrivePacket(short seq_no, int endpoint, short response_size, const commBuffer& payload_ref);
    static commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet);


};


#endif //CHANNEL_TFG_ADRIA