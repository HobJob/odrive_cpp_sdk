#include "motor.h"


Motor::Motor(){
    
}

Motor::Motor(MOTOR_NAME name, int encoder_cpr, int encoder_max_rpm, uint8_t pole_pairs, int max_rpm, int max_current, float kV, float calibration_current, float current_limit, int motor_type){
    this->name = name;
    this->encoder_cpr = encoder_cpr;
    this->encoder_max_rpm = encoder_max_rpm;
    this->pole_pairs = pole_pairs;
    this->max_rpm = max_rpm;
    this->max_current = max_current;
    this->kV = kV;
    this->calibration_current = calibration_current;
    this->current_limit = current_limit;
    this->motor_type = motor_type;
}

Motor::~Motor(){

}

void Motor::setCurrentSetpoint(float newSetpoint){
    channel->odriveEndpointSetFloat(this->name == M0 ? M0_CURRENT_CONTROL_IQ_SETPOINT : M1_CURRENT_CONTROL_IQ_SETPOINT, newSetpoint);
}

void Motor::setCurrentControllerPGain(float newGain){
    channel->odriveEndpointSetFloat(this->name == M0 ? M0_CURRENT_CONTROL_P_GAIN : M1_CURRENT_CONTROL_P_GAIN, newGain);
}

void Motor::setCurrentControllerIGain(float newGain){
    channel->odriveEndpointSetFloat(this->name == M0 ? M0_CURRENT_CONTROL_I_GAIN : M1_CURRENT_CONTROL_I_GAIN, newGain);
}

float Motor::getVelEstimate(){
    float vel;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_ENCODER_VEL_ESTIMATE : M1_ENCODER_VEL_ESTIMATE, vel);
    return vel;
}

float Motor::getCurrentSetpoint(){
    float currentSetpoint;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_CURRENT_CONTROL_IQ_SETPOINT : M1_CURRENT_CONTROL_IQ_SETPOINT, currentSetpoint);
    return currentSetpoint;
}

float Motor::getCurrent(){
    float current;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_CURRENT_CONTROL_IQ_MEASURED : M1_CURRENT_CONTROL_IQ_MEASURED, current);
    return current;
}

float Motor::getTorque(){
    float current = getCurrent();
    return 8.27f * current / this->kV;
}

float Motor::getPosEstimate(){
    float pos;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE,pos);
    return pos;
}


void Motor::setRequestedState(int newState) {
    this->requestedState = newState;
    channel->odriveEndpointSetInt(this->name == M0 ? M0_REQUESTED_STATE : M1_REQUESTED_STATE, newState);
}

void Motor::setControlMode(int controlMode){
    channel->odriveEndpointSetUInt8(this->name == M0 ? AXIS0_CONTROLLER_CONFIG_CONTROL_MODE : AXIS1_CONTROLLER_CONFIG_CONTROL_MODE, controlMode);
}

void Motor::enable() {
    channel->odriveEndpointSetInt(this->name == M0 ? M0_REQUESTED_STATE : M1_REQUESTED_STATE, this->requestedState);
}

void Motor::disable() {
    channel->odriveEndpointSetInt(this->name == M0 ? M0_REQUESTED_STATE : M1_REQUESTED_STATE, AXIS_STATE_IDLE);
}

void Motor::setPositionSetpoint(float newSetpoint) {
    channel->odriveEndpointSetFloat(this->name == M0 ? AXIS0_CONTROLLER_POS_SETPOINT : AXIS1_CONTROLLER_POS_SETPOINT, newSetpoint);
}
