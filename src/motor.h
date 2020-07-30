#ifndef MOTOR_TFG_ADRIA
#define MOTOR_TFG_ADRIA

#include <stdint.h>
#include "channel.h"
#include <stdlib.h> 

#include <math.h>

//From enums.py
#define MOTOR_TYPE_HIGH_CURRENT 0
#define M_PI_DIVIDED_1000 (M_PI / 1000.0f)



enum MOTOR_NAME{
    M0,
    M1
};

class Channel;

class Motor{

public:
    Motor();
    Motor(MOTOR_NAME name, int encoder_cpr = 2000, int encoder_max_rpm = 14500,
        uint8_t pole_pairs = 12,
        int max_rpm = 7920,
        int max_current = 25,
        float kV = 330.0f,
        float calibration_current = 5.0f,
        float current_limit = 5.0f,
        float current_range = 6.0f,
        int motor_type = MOTOR_TYPE_HIGH_CURRENT);
    
    ~Motor();

    MOTOR_NAME name;
    Channel *channel;
    
    int encoder_cpr; //2000
    int encoder_max_rpm; //14500

    int32_t pole_pairs; //12
    int max_rpm; //7920
    int max_current; //25
    float kV; //330
    float calibration_current; //5
    float current_limit; //5
    float current_range; //6

    int motor_type;

    float zeroOffset;

    void disable();

    void setCurrentSetpoint(float newSetpoint);

    float getVelEstimate();

    float getCurrentSetpoint();

    float getCurrent();

    float getTorque();

    void setCurrentControllerPGain(float newGain);

    void setCurrentControllerIGain(float newGain);

    void setRequestedState(int newState);

    float getPosEstimate();

    void setControlMode(int controlMode);

    void enable();
    
    void moveStartingPosition(double delayBetweenSteps);

    int requestedState;

    void setPositionSetpoint(float newSetpoint);

    void setTorque(float d);

    float castCPRToRad(float cpr);
    float castCPRToRads(float cpr);
    float castRadsToCPR(float rads);
    

    float castCurrentToTorque(float current);
    float castTorqueToCurrent(float torque);

    float getPosEstimateInRad();
    float getVelEstimateInRads();
    void zeroPosition();
};

#endif //MOTOR_TFG_ADRIA