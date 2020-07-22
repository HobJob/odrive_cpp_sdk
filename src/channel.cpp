#include "channel.h"


Channel::Channel(libusb_device_handle *device_handle){
    this->device_handle = device_handle;
    outbound_seq_no_ = 0;
}

Channel::~Channel(){

    int result = libusb_release_interface(device_handle, 0);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Error calling libusb_release_interface on channel destructor." << std::endl;
    }
    libusb_close(device_handle);
}
/*
void Channel::setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks) {
  for (uint8_t i = 0; i < num_motors_; ++i) {
    zeroeth_radian_in_encoder_ticks_[i] = zeroeth_radian_in_encoder_ticks[i];
  }
}

int Channel::runCalibration(){ return -1; }

int Channel::setGoalMotorPositions(const double* axes_positions_in_radians_array) {
  if (! motor_to_odrive_handle_index_) {
    return ODRIVE_SDK_NOT_INITIALIZED;
  }

  int cmd;
  for (uint8_t i = 0; i < num_motors_; ++i) {
    double target_ticks = axes_positions_in_radians_array[i] * encoder_ticks_per_radian_[i];
    if (i != 0 && motor_relative_to_prior_motor_[i]) {
      target_ticks = (axes_positions_in_radians_array[i] - axes_positions_in_radians_array[i-1]) * encoder_ticks_per_radian_[i];
    }
    float position_in_ticks = (int) (zeroeth_radian_in_encoder_ticks_[i] + target_ticks);

    uint8_t handle_index = motor_to_odrive_handle_index_[i];
    cmd = motor_position_map_[i] ? ODRIVE_SDK_SET_GOAL_1_CMD : ODRIVE_SDK_SET_GOAL_0_CMD;

    int result = odriveEndpointSetFloat(odrive_handles_[handle_index], cmd, position_in_ticks);
    if (result != LIBUSB_SUCCESS) {
      std::cerr << "Couldn't send `" << std::to_string(cmd) << " " << std::to_string(position_in_ticks) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
      return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
  }

  return ODRIVE_SDK_COMM_SUCCESS;
}

int Channel::readCurrentMotorPositions(double* axes_positions_in_radians_array) {
  if (! motor_to_odrive_handle_index_) {
    return ODRIVE_SDK_NOT_INITIALIZED;
  }

  int cmd;
  for (uint8_t i = 0; i < num_motors_; ++i) {
    uint8_t handle_index = motor_to_odrive_handle_index_[i];
    cmd = motor_position_map_[i] ? ODRIVE_SDK_GET_ENCODER_1_STATE : ODRIVE_SDK_GET_ENCODER_0_STATE;

    int read_encoder_ticks;
    int result = odriveEndpointGetInt(odrive_handles_[handle_index], cmd, read_encoder_ticks);
    if (result != LIBUSB_SUCCESS) {
      std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
      return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }

    // NOTE!  THIS RELIES ON THE AXIS POSITIONS BEING READ IN ORDER; THAT axes_positions_in_radians_array[i-1] WAS READ FROM ODRIVE ALREADY
    double interpreted_radians = read_encoder_ticks / (double)encoder_ticks_per_radian_[i];
    if (i != 0 && motor_relative_to_prior_motor_[i]) {
      interpreted_radians = axes_positions_in_radians_array[i-1] + (read_encoder_ticks / (double)encoder_ticks_per_radian_[i]);
    }

    axes_positions_in_radians_array[i] =  interpreted_radians - (zeroeth_radian_in_encoder_ticks_[i] / (double)encoder_ticks_per_radian_[i]); // TODO Check math
  }
  return ODRIVE_SDK_COMM_SUCCESS;
}

int Channel::checkErrors(uint8_t* error_codes_array) {
  if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        uint8_t handle_index = motor_to_odrive_handle_index_[i];
        cmd = motor_position_map_[i] ? ODRIVE_SDK_GET_MOTOR_1_ERROR : ODRIVE_SDK_GET_MOTOR_0_ERROR;

        uint8_t motor_error_output;
        int result = odriveEndpointGetUInt8(odrive_handles_[handle_index], cmd, motor_error_output);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "CppSdk::checkErrors couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
        error_codes_array[i] = motor_error_output;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}
int Channel::readCurrentVBusVoltage(float& voltage)
{
  if (! motor_to_odrive_handle_index_) {
    return ODRIVE_SDK_NOT_INITIALIZED;
  }

  int cmd;
  for (uint8_t i = 0; i < num_motors_; ++i) {
    uint8_t handle_index = motor_to_odrive_handle_index_[i];
    cmd = ODRIVE_SDK_VBUS_VOLTAGE_CMD;

    int result = odriveEndpointGetFloat(odrive_handles_[handle_index], cmd, voltage);
    if (result != LIBUSB_SUCCESS) {
      std::cerr << "CppSdk::checkErrors couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
      return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
  }
  return ODRIVE_SDK_COMM_SUCCESS;
}
*/


int Channel::odriveEndpointRequest(int endpoint_id, commBuffer& received_payload, int& received_length, commBuffer payload, int ack, int length) {
  commBuffer send_buffer;
  commBuffer receive_buffer;
  unsigned char receive_bytes[ODRIVE_SDK_MAX_RESULT_LENGTH] = { 0 };
  int sent_bytes = 0;
  int received_bytes = 0;
  short received_seq_no = 0;

  if (ack) {
    endpoint_id |= 0x8000;
  }
  outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
  outbound_seq_no_ |= 0x80; // FIXME, see odrive protocol.py
  short seq_no = outbound_seq_no_;

  // Send the packet
  commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, payload);
  
  std::unique_lock<std::mutex>lck (tx_mutex);

  int result = libusb_bulk_transfer(device_handle, ODRIVE_SDK_WRITING_ENDPOINT, packet.data(), packet.size(), &sent_bytes, 0);
  
  if (result != LIBUSB_SUCCESS) {
    std::cerr << "Could not call libusb_bulk_transfer for writing: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
    return result;
  } else if (packet.size() != sent_bytes) {
    std::cerr << "Could not call libusb_bulk_transfer: only wrote " << std::to_string(sent_bytes) << " of " << std::to_string(packet.size()) << " bytes (wanted to send `" << packet.data() << "`)" << std::endl;
  }

  if (ack) {
    // Immediatly waituint8 for response from Odrive and check if ack (if we asked for one)
    result = libusb_bulk_transfer(device_handle, ODRIVE_SDK_READING_ENDPOINT, receive_bytes, ODRIVE_SDK_MAX_BYTES_TO_RECEIVE, &received_bytes, ODRIVE_SDK_TIMEOUT);
    lck.unlock();
    if (result != LIBUSB_SUCCESS) {
      std::cerr << "Could not call libusb_bulk_transfer for reading: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
      return result;
    }

    for (int i = 0; i < received_bytes; i++) {
      receive_buffer.push_back(receive_bytes[i]);
    }

    received_payload = decodeODrivePacket(receive_buffer, received_seq_no, receive_buffer);

    if (received_seq_no != seq_no) {
      std::cerr << "[ERROR] Recieved packet from odrive for sequence number " << std::to_string(received_seq_no) << " but was expecting to recieve reply for sequence number " << std::to_string(seq_no) << ".  Sequence ordering is not implemented yet!!!" << std::endl;
    }

    // return the response payload
    received_length = received_payload.size();
  }else{
    lck.unlock();
  }


  return LIBUSB_SUCCESS;
}


//TODO: Looks like endpoint 0 returns 0 bytes!
int Channel::getJSON(commBuffer &data){
    commBuffer send_payload;
    //commBuffer receive_payload;

    int received_length;
    int result;

    /*result = odriveEndpointSetInt(0,0);
    if(result != LIBUSB_SUCCESS){
        return result;
    }
     */

    result = odriveEndpointRequest(0, data, received_length, send_payload, 1, 64);

    std::cout << "Obtained : " << received_length << " bytes" << std::endl;


    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    //data = receive_payload;
    return result;
    //data.push_back();
}

int Channel::odriveEndpointGetUInt8(int endpoint_id, uint8_t& value) {
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  int result = odriveEndpointRequest(endpoint_id, receive_payload, received_length, send_payload, 1, 1);
  //std::cout << "Received payload size is " << received_length<< std::endl;
  if (result != LIBUSB_SUCCESS) {
    return result;
  }
  if(received_length != 0)
    deserializeCommBufferUInt8(receive_payload, value);

  return LIBUSB_SUCCESS;
}

int Channel::odriveEndpointGetShort(int endpoint_id, short& value) {
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 2);
  if (result != LIBUSB_SUCCESS) {
    return result;
  }

  readShortFromCommBuffer(receive_payload, value);

  return LIBUSB_SUCCESS;
}
int Channel::odriveEndpointGetInt(int endpoint_id, int& value) {
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 4);
  if (result != LIBUSB_SUCCESS) {
    return result;
  }

  deserializeCommBufferInt(receive_payload, value);
  return LIBUSB_SUCCESS;
}

int Channel::odriveEndpointGetUInt64(int endpoint_id, uint64_t &value) {
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 8);
  if (result != LIBUSB_SUCCESS) {
    return result;
  }
  deserializeCommBufferUInt64(receive_payload, value);
  return LIBUSB_SUCCESS;
}

int Channel::odriveEndpointGetFloat(int endpoint_id, float &value)
{
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 4);
  if (result != LIBUSB_SUCCESS) {
    return result;
  }
  deserializeCommBufferFloat(receive_payload, value);
  return LIBUSB_SUCCESS;
}

int Channel::odriveEndpointSetInt(int endpoint_id, const int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferInt(send_payload, value);
    int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int Channel::odriveEndpointSetUInt8(int endpoint_id, const uint8_t & value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    send_payload.push_back(value);
    int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}
int Channel::odriveEndpointReset(int endpoint_id, const int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferInt(send_payload, value);
    int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 0, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}


int Channel::odriveEndpointSetFloat(int endpoint_id, const float& value) {
  commBuffer send_payload;
  commBuffer receive_payload;
  int received_length;
  serializeCommBufferFloat(send_payload, value);
  int result = odriveEndpointRequest( endpoint_id, receive_payload, received_length, send_payload, 1, 0);
  if (result != ODRIVE_SDK_COMM_SUCCESS) {
    return result;
  }
  return ODRIVE_SDK_COMM_SUCCESS;
}


void Channel::appendShortToCommBuffer(commBuffer& buf, const short value) {
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
}


void Channel::readShortFromCommBuffer(commBuffer& byte_array, short& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < sizeof(short); ++i) {
    value <<= 8;
    value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be16toh(value);
}


void Channel::serializeCommBufferFloat(commBuffer& buf, const float& value) {
  for(int i = 0; i < sizeof(float); i++){
    buf.push_back(((unsigned char*)&value)[i]);
  }
}

void Channel::serializeCommBufferInt(commBuffer& buf, const int& value) {
  buf.push_back((value >> 0) & 0xFF);
  buf.push_back((value >> 8) & 0xFF);
  buf.push_back((value >> 16) & 0xFF);
  buf.push_back((value >> 24) & 0xFF);
}

void Channel::deserializeCommBufferInt(commBuffer& byte_array, int& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < byte_array.size(); ++i) {
    value <<= 8;
    value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be32toh(value);
}

void Channel::deserializeCommBufferUInt64(commBuffer& v, uint64_t& value) {
  value = 0;
  for(int i = 0; i < v.size(); ++i) {
    value <<= 8;
    value |= v[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be64toh(value);
}

void Channel::deserializeCommBufferUInt8(commBuffer& v, uint8_t& value) {
  value = v[0];
}

void Channel::deserializeCommBufferFloat(commBuffer &buf, float &value)
{
  union {
    float f;
    int i;
  }u;
  deserializeCommBufferInt(buf, u.i);
  value = u.f;
}

commBuffer Channel::createODrivePacket(short seq_no, int endpoint_id, short response_size, const commBuffer& input) {
  commBuffer packet;
  short crc = 0;
  if ((endpoint_id & 0x7fff) == 0) {
    crc = ODRIVE_SDK_PROTOCOL_VERION;
  }
  else {
    crc = ODRIVE_SDK_DEFAULT_CRC_VALUE;
  }

  appendShortToCommBuffer(packet, seq_no);
  appendShortToCommBuffer(packet, endpoint_id);
  appendShortToCommBuffer(packet, response_size);

  for (uint8_t b : input) {
    packet.push_back(b);
  }

  appendShortToCommBuffer(packet, crc);

  return packet;
}

commBuffer Channel::decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet) {
  commBuffer payload;
  readShortFromCommBuffer(buf, seq_no); // reads 2 bytes so start next for loop at 2
  seq_no &= 0x7fff;
  for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
    payload.push_back(buf[i]);
  }
  return payload;
}


//Wait for the ODrive to finish an endpoint operation.
void Channel::waituint8(int endpoint, uint8_t newValue) {
    uint8_t endpoint_result;
    bool loopCondition;

    std::cout << "\tWaiting for an operation to start " << std::flush;

    //Initially waituint8 until the ODrive starts the task
    do{
        odriveEndpointGetUInt8(endpoint, endpoint_result);
        loopCondition = endpoint_result == newValue;

        if(loopCondition){
            //Wait some time...
            std::cout<< "." << std::flush;
            usleep(100000);
        }
    }while(loopCondition);

    waitOperationFinisheduint8(endpoint, newValue);
    std::cout << std::endl << "\tWait finished!" << std::endl;
}

void Channel::waitOperationFinisheduint8(int endpoint, uint8_t  newValue){
    uint8_t endpoint_result;
    bool loopCondition;

    std::cout << std::endl << "\tWaiting for an operation to finish " << std::flush;
    //Wait untill it finishes the task

    do {
        odriveEndpointGetUInt8(endpoint, endpoint_result);
        loopCondition = endpoint_result != newValue;

        if(loopCondition){
            //Give some time to the ODrive to boot again.
            std::cout<< "." << std::flush;
            usleep(100000);
        }
    }while (loopCondition);

}
