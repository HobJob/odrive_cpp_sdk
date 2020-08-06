#include "motor.h"


Motor::Motor(){
    
}

Motor::Motor(MOTOR_NAME name, int encoder_cpr, int encoder_max_rpm, uint8_t pole_pairs, int max_rpm, int motor_max_current, float kV, float calibration_current, float current_limit, float current_range, int motor_type){
    this->name = name;
    this->encoder_cpr = encoder_cpr;
    this->encoder_max_rpm = encoder_max_rpm;
    this->pole_pairs = pole_pairs;
    this->max_rpm = max_rpm;
    this->motor_max_current = motor_max_current;
    this->kV = kV;
    this->calibration_current = calibration_current;
    this->current_limit = current_limit;
    this->current_range = current_range;
    this->motor_type = motor_type;
    this->zeroOffset = 0;
}

Motor::~Motor(){

}

void Motor::moveStartingPosition(double delayBetweenSteps, bool * signalFlag)
{
    this->setControlMode(CTRL_MODE_POSITION_CONTROL);

    float pos;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE, pos);   
    

    int sign = (pos > 0) - (pos < 0);

    int steps = (abs(pos) / this->encoder_cpr) * 8;
    std::cout << "Initial position is " << pos << " Sign is " << sign << " NSteps are " << steps << std::endl;
    
    if(steps == 0){
        channel->odriveEndpointSetFloat(this->name == M0 ? AXIS0_CONTROLLER_POS_SETPOINT : AXIS1_CONTROLLER_POS_SETPOINT, 0.0f);
        return;   
    }

    float newPos = 0;
    for(auto i = steps; i >= 0; i--){
        
        newPos = (pos * i) / (steps);
        std::cout << "Sending a newpos is " << newPos<<  std::endl;
        channel->odriveEndpointSetFloat(this->name == M0 ? AXIS0_CONTROLLER_POS_SETPOINT : AXIS1_CONTROLLER_POS_SETPOINT, newPos);   
        usleep((delayBetweenSteps * 1000.0f));
        channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE,pos);   
        if(abs(pos - newPos) > 75){
            std::cout << "Repeating the command. Goal wasn't reached! " << std::endl;
            i++;
        }
        if(*signalFlag){
            break;
        }
    }
}

//Addded for backward compatibility
void Motor::moveStartingPosition(double delayBetweenSteps)
{
    this->setControlMode(CTRL_MODE_POSITION_CONTROL);

    float pos;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE, pos);   
    

    int sign = (pos > 0) - (pos < 0);

    int steps = (abs(pos) / this->encoder_cpr) * 8;
    std::cout << "Initial position is " << pos << " Sign is " << sign << " NSteps are " << steps << std::endl;
    
    if(steps == 0){
        channel->odriveEndpointSetFloat(this->name == M0 ? AXIS0_CONTROLLER_POS_SETPOINT : AXIS1_CONTROLLER_POS_SETPOINT, 0.0f);
        return;   
    }

    float newPos = 0;
    for(auto i = steps; i >= 0; i--){
        
        newPos = (pos * i) / (steps);
        std::cout << "Sending a newpos is " << newPos<<  std::endl;
        channel->odriveEndpointSetFloat(this->name == M0 ? AXIS0_CONTROLLER_POS_SETPOINT : AXIS1_CONTROLLER_POS_SETPOINT, newPos);   
        usleep((delayBetweenSteps * 1000.0f));
        channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE,pos);   
        if(abs(pos - newPos) > 75){
            std::cout << "Repeating the command. Goal wasn't reached! " << std::endl;
            i++;
        }
    }
}

void Motor::zeroPosition()
{
    float poscpr = getPosEstimate();
    this->zeroOffset = poscpr;
}

void Motor::setCurrentSetpoint(float newSetpoint){
    channel->odriveEndpointSetFloat(this->name == M0 ? M0_CONTROLLER_CURRENT_SETPOINT : M1_CONTROLLER_CURRENT_SETPOINT, newSetpoint);
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

float Motor::castCPRToRad(float cpr)
{
    return  (2.0f *  (float)M_PI * (cpr + 1000.0f)) / 2000.0f;
}

float Motor::castCPRToRads(float cpr)
{
    return  (2.0f *  (float)M_PI * (cpr)) / 2000.0f;
}
float Motor::castRadsToCPR(float rads)
{
    return (rads * 2000.0f) / (2.0f * (float)M_PI) - 1000.0f;
}

float Motor::castTorqueToCurrent(float torque)
{
    return (torque * this->kV) / 8.27;
}

float Motor::castCurrentToTorque(float current)
{
    return (8.27f * current) / this->kV;   
}

float Motor::getTorque(){
    float current = getCurrent();
    return 8.27f * current / this->kV;
}

float Motor::getPosEstimate(){
    float pos;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE, pos);
    return pos - zeroOffset;
}
float Motor::getPosEstimateInRad()
{
    float pos;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_POS_ESTIMATE : M1_POS_ESTIMATE, pos);
    return (pos - zeroOffset) * M_PI_DIVIDED_1000 + M_PI;
}


float Motor::getPosEstimateInRadDoublePendulum()
{
 if(this->name == M0){
        //Motor0 -> Endpoint. Must modify alpha
        float theta, alfa;
        
        channel->odriveEndpointGetFloat(M0_POS_ESTIMATE, alfa);
        channel->odriveEndpointGetFloat(M1_POS_ESTIMATE, theta);
        
        alfa  -= zeroOffset;
        theta -= otherMotor->zeroOffset;

        return -1 * (alfa + theta) * M_PI_DIVIDED_1000;
    }else{
        float pos;
        //Motor1 -> BaseLink. The Thetha is the same in odrive and crocoddyl.
        channel->odriveEndpointGetFloat(M1_POS_ESTIMATE, pos);
        return (pos - zeroOffset) * M_PI_DIVIDED_1000 + M_PI;
    }  
}

float Motor::getVelEstimateInRads()
{
    float vel;
    channel->odriveEndpointGetFloat(this->name == M0 ? M0_ENCODER_VEL_ESTIMATE : M1_ENCODER_VEL_ESTIMATE, vel);
    return vel * M_PI_DIVIDED_1000;
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

void Motor::setTorque(float torque) {
    float current = torque * this->kV / 8.27f;
    channel->odriveEndpointSetFloat(this->name == M0 ? M0_CONTROLLER_CURRENT_SETPOINT : M1_CONTROLLER_CURRENT_SETPOINT, current);
}

void Motor::setTorqueDoublePendulum(float torque)
{
    if(this->name == M0){
        float current = -torque * this->kV / 8.27f;
        channel->odriveEndpointSetFloat(M0_CONTROLLER_CURRENT_SETPOINT, current);
    }else{
        float current = torque * this->kV / 8.27f;
        channel->odriveEndpointSetFloat(M1_CONTROLLER_CURRENT_SETPOINT, current);
    }
}


void Motor::setOtherMotor(Motor * otherMotor)
{
    this->otherMotor = otherMotor;
}