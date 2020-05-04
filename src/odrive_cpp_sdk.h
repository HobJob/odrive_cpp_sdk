#ifndef ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_
#define ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_


#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <endian.h>

typedef std::vector<uint8_t> commBuffer;


namespace odrive
{

class Channel {

public:
  //Create a new channel to a specific ODrive
  Channel(const std::string odrive_serial_number);

  Channel(
      const std::string* odrive_serial_numbers,
      const uint8_t num_odrives,
      const std::string* motor_to_odrive_serial_number_map,
      const bool* motor_position_map, // false = slot 0, true = slot 1
      const float* encoder_ticks_per_radian,
      const bool* motor_relative_to_prior_motor, // true if there is influence, like a belt drive
      const uint8_t num_motors
      );
  ~Channel();

  int init(); // start communication
  int runCalibration();
  void setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks); // assumed to match num_motors
  int setGoalMotorPositions(const double* axes_positions_in_radians_array); // assumed to match num_motors
  int readCurrentMotorPositions(double* axes_positions_in_radians_array); // assumed to match num_motors
  int checkErrors(uint8_t* error_codes_array); // assumed to match num_motors
  int readCurrentVBusVoltage(float& voltage);

private:

  // read settings
  uint8_t num_odrives_;
  uint8_t num_motors_;
  float* encoder_ticks_per_radian_;
  int16_t* zeroeth_radian_in_encoder_ticks_;
  bool* motor_position_map_;
  bool* motor_relative_to_prior_motor_;

  // saved for use between creation and init
  std::string* odrive_serial_numbers_;
  std::string* motor_to_odrive_serial_number_map_;

  // for usb
  libusb_device_handle** odrive_handles_;
  libusb_context* libusb_context_;
  uint8_t* motor_to_odrive_handle_index_;
  int initUSBHandlesBySNs();

  short outbound_seq_no_; // unique ids for packets send to odrive

  int odriveEndpointRequest(libusb_device_handle* handle, int endpoint_id, commBuffer& received_payload, int& received_length, const commBuffer payload, const int ack, const int length);
  int odriveEndpointGetShort(libusb_device_handle* handle, int endpoint_id, short& value);
  int odriveEndpointGetInt(libusb_device_handle* handle, int endpoint_id, int& value);
  int odriveEndpointGetUInt8(libusb_device_handle* handle, int endpoint_id, uint8_t& value);
  int odriveEndpointGetUInt64(libusb_device_handle* handle, int endpoint_id, uint64_t& value);
  int odriveEndpointGetFloat(libusb_device_handle* handle, int endpoint_id, float& value);
  int odriveEndpointSetInt(libusb_device_handle* handle, int endpoint_id, const int& value);
  int odriveEndpointSetFloat(libusb_device_handle* handle, int endpoint_id, const float& value);
  void serializeCommBufferInt(commBuffer& buf, const int& value);
  void serializeCommBufferFloat(commBuffer& buf, const float& value);
  void deserializeCommBufferInt(commBuffer& buf, int& value);
  void appendShortToCommBuffer(commBuffer& buf, const short value);
  void readShortFromCommBuffer(commBuffer& buf, short& value);
  void deserializeCommBufferUInt64(commBuffer& buf, uint64_t& value);
  void deserializeCommBufferUInt8(commBuffer& buf, uint8_t& value);
  void deserializeCommBufferFloat(commBuffer& buf, float& value);

  commBuffer createODrivePacket(short seq_no, int endpoint, short response_size, const commBuffer& payload_ref);
  commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet);
};
}

#endif /* ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_ */
