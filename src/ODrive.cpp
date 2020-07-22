#include "ODrive.h"
#include "robot.h"


using nlohmann::json;

ODrive::ODrive(Channel *channel, Robot *pRobot) {
    this->channel = channel;
    this->robot = pRobot;
    freeDeviceHandle = true;
    this->m0 = nullptr;
    this->m1 = nullptr;
}

ODrive::~ODrive(){
    if (freeDeviceHandle)
        delete this->channel;

    if (this->m0)
        delete this->m0;

    if (this->m1)
        delete this->m1;
}

//returns the serialNumber from the ODrive.
int ODrive::readSerialNumber(uint64_t &serial_number) {
    int result = channel->odriveEndpointGetUInt64(SERIAL_NUMBER_CMD, serial_number);
    this->serialNumber = serial_number;
    return result;
}

//TODO: look at channel->getJSON.
void ODrive::getJSON(){
    commBuffer a;
    channel->getJSON(a);
    //json j = json::parse(a);

    std::cout << "Len is : " << a.size() << std::endl;
    std::cout << "JSON is : " << a[0] << std::endl;
}

void ODrive::addMotor(Motor *m){
    if (m->name == M0){
        this->m0 = m;
        this->m0->channel = channel;
    }else{
        this->m1 = m;
        this->m1->channel = channel;
    }
}


//TODO: REFACTOR ALL THE #DEFINE thing. Use instead the JSON and a helper function to look for the correct endpoint ID.
void ODrive::configureMotor(Motor *m) {
    if (m->name == M0){
        this->m0 = m;
        this->m0->channel = channel;

        //Stop the motor.
        channel->odriveEndpointSetInt(M0_REQUESTED_STATE, AXIS_STATE_IDLE);

        //Configure odrv0.axis0.motor...
        channel->odriveEndpointSetFloat(M0_CONFIG_CURRENT_LIM, m->current_limit);
        channel->odriveEndpointSetFloat(M0_CONFIG_CALIBRATION_CURRENT, m->calibration_current);
        channel->odriveEndpointSetUInt8(M0_CONFIG_MOTOR_TYPE, m->motor_type);
        channel->odriveEndpointSetInt(M0_CONFIG_POLE_PAIRS, m->pole_pairs);
        channel->odriveEndpointSetFloat(M0_CONFIG_MOTOR_REQUESTED_CURRENT_RANGE, m->current_range);

        //Configure odrv0.axis0.encoder...
        channel->odriveEndpointSetInt(M0_ENCODER_CONFIG_CPR,m->encoder_cpr);

        //And limit speed to 20 rev/s
        channel->odriveEndpointSetFloat(AXIS0_CONTROLLER_CONFIG_VEL_LIMIT, ((float)m->encoder_cpr) * 10.0f);

    }else{
        this->m1 = m;
        this->m1->channel = channel;

        //Stop the motor.
        channel->odriveEndpointSetInt(M1_REQUESTED_STATE, AXIS_STATE_IDLE);

        //Configure odrv0.axis1.motor...
        channel->odriveEndpointSetFloat(M1_CONFIG_CURRENT_LIM, m->current_limit);
        channel->odriveEndpointSetFloat(M1_CONFIG_CALIBRATION_CURRENT, m->calibration_current);
        channel->odriveEndpointSetUInt8(M1_CONFIG_MOTOR_TYPE, m->motor_type);
        channel->odriveEndpointSetInt(M1_CONFIG_POLE_PAIRS, m->pole_pairs);
        channel->odriveEndpointSetFloat(M1_CONFIG_MOTOR_REQUESTED_CURRENT_RANGE, m->current_range);

        //Configure odrv0.axis0.encoder...
        channel->odriveEndpointSetInt(M1_ENCODER_CONFIG_CPR,m->encoder_cpr);

        //And limit speed to 20 rev/s
        channel->odriveEndpointSetFloat(AXIS1_CONTROLLER_CONFIG_VEL_LIMIT, m->encoder_cpr * 4);
    }
}


//It is recommended that you mechanically disengage the motor from anything other than the encoder, so it can spin freely
void ODrive::calibrateMotorsAndEncoders(){

    uint8_t isCalibrated;

    if(m0){
        std::cout << "Starting calibration of motor 0" << std::endl;
        //Motor calibration
        channel->odriveEndpointSetInt(M0_REQUESTED_STATE, AXIS_STATE_MOTOR_CALIBRATION);

        //Wait until the calib. ends
        channel->waituint8(M0_CURRENT_STATE, AXIS_STATE_IDLE);

        //Save the motor calibration. This way we can skip this process on boot.
        channel->odriveEndpointSetUInt8(M0_CONFIG_PRE_CALIBRATED, 1);

        std::cout << "Starting calibration of encoder 0" << std::endl;

        //Encoder calibration
        channel->odriveEndpointSetUInt8(M0_ENCODER_CONFIG_USE_INDEX,1);

        channel->odriveEndpointSetInt(M0_REQUESTED_STATE, AXIS_STATE_ENCODER_INDEX_SEARCH);
        //Wait until the calib. ends
        channel->waituint8(M0_CURRENT_STATE, AXIS_STATE_IDLE);

        channel->odriveEndpointSetInt(M0_REQUESTED_STATE, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
        //Wait until the calib. ends
        channel->waituint8(M0_CURRENT_STATE, AXIS_STATE_IDLE);

        //Save the encoder calibration. This way we can skip this process on boot.
        channel->odriveEndpointSetUInt8(M0_ENCODER_CONFIG_PRE_CALIBRATED, 1);

        //Check if calibration went well
        channel->odriveEndpointGetUInt8(M0_ENCODER_CONFIG_PRE_CALIBRATED, isCalibrated);
        std::cout << "Calibration of motor 0 " << (isCalibrated == 1 ? "successful" : "failed") << std::endl;

    }

    if(m1){
        std::cout << "Starting calibration of motor 1" << std::endl;
        //Motor calibration
        channel->odriveEndpointSetInt(M1_REQUESTED_STATE, AXIS_STATE_MOTOR_CALIBRATION);
        //Wait until the calib. ends
        channel->waituint8(M1_CURRENT_STATE, AXIS_STATE_IDLE);

        //Save the motor calibration. This way we can skip this process on boot.
        channel->odriveEndpointSetUInt8(M1_CONFIG_PRE_CALIBRATED, 1);

        std::cout << "Starting calibration of encoder 1" << std::endl;
        //Encoder calibration
        channel->odriveEndpointSetUInt8(M1_ENCODER_CONFIG_USE_INDEX,1);
        channel->odriveEndpointSetInt(M1_REQUESTED_STATE, AXIS_STATE_ENCODER_INDEX_SEARCH);
        //Wait until the calib. ends
        channel->waituint8(M1_CURRENT_STATE, AXIS_STATE_IDLE);

        channel->odriveEndpointSetInt(M1_REQUESTED_STATE, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
        //Wait until the calib. ends
        channel->waituint8(M1_CURRENT_STATE, AXIS_STATE_IDLE);

        //Save the encoder calibration. This way we can skip this process on boot.
        channel->odriveEndpointSetUInt8(M1_ENCODER_CONFIG_PRE_CALIBRATED, 1);

        uint8_t isCalibrated;
        //Check if calibration went well
        channel->odriveEndpointGetUInt8(M1_ENCODER_CONFIG_PRE_CALIBRATED, isCalibrated);
        std::cout << "Calibration of motor 1 " << (isCalibrated == 1 ? "successful" : "failed") << std::endl;
    }
}

void ODrive::configStartupSequence(){
    if(m0){
        channel->odriveEndpointSetUInt8(AXIS0_CONFIG_STARTUP_MOTOR_CALIBRATION, 0);
        channel->odriveEndpointSetUInt8(AXIS0_CONFIG_STARTUP_ENCODER_INDEX_SEARCH, 1);
        channel->odriveEndpointSetUInt8(AXIS0_CONFIG_STARTUP_ENCODER_OFFSET_CALIBRATION, 0);
        channel->odriveEndpointSetUInt8(AXIS0_CONFIG_STARTUP_CLOSED_LOOP_CONTROL, 0);
        channel->odriveEndpointSetUInt8(AXIS0_CONFIG_STARTUP_SENSORLESS_CONTROL, 0);
    }

    if(m1){
        channel->odriveEndpointSetUInt8(AXIS1_CONFIG_STARTUP_MOTOR_CALIBRATION, 0);
        channel->odriveEndpointSetUInt8(AXIS1_CONFIG_STARTUP_ENCODER_INDEX_SEARCH, 1);
        channel->odriveEndpointSetUInt8(AXIS1_CONFIG_STARTUP_ENCODER_OFFSET_CALIBRATION, 0);
        channel->odriveEndpointSetUInt8(AXIS1_CONFIG_STARTUP_CLOSED_LOOP_CONTROL, 0);
        channel->odriveEndpointSetUInt8(AXIS1_CONFIG_STARTUP_SENSORLESS_CONTROL, 0);
    }
}


void ODrive::saveConfiguration(){
    //Execute the function
    channel->odriveEndpointSetInt(SAVE_CONFIG,0);
}

void ODrive::reboot(){
    channel->odriveEndpointReset(REBOOT,0);
    delete channel;

    //Give some time to the ODrive to boot again.
    std::cout<< "Rebooting ODrive... Sleeping for " << REBOOT_SLEEP_TIME <<std::endl;
    usleep(REBOOT_SLEEP_TIME);
    robot->reconnectODrive(this);
}


void ODrive::setBrakeResistance(float brake) {
    this->brake_resistance = brake;

    //Configure the brake resistance
    channel->odriveEndpointSetFloat(CONFIG_BRAKE_RESISTANCE, this->brake_resistance);
}

float ODrive::inputVoltage() {
    float voltage;
    channel->odriveEndpointGetFloat(VBUS_VOLTAGE_CMD,voltage);
    return voltage;
}

void ODrive::dontFreeDeviceHandle() {
    this->freeDeviceHandle = false;
}

void ODrive::setNewChannel(Channel *pChannel) {
    channel = pChannel;
}
