#ifndef MOTOR_TFG_ADRIA
#define MOTOR_TFG_ADRIA

#include <stdint.h>
#include <stdlib.h> 
#include <math.h>

#include "channel.h"
#include <stdlib.h> 

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
        int motor_max_current = 25,
        float kV = 330.0f,
        float calibration_current = 10.0f,
        float current_limit = 5.0f,
        float current_range = 6.0f,
        int motor_type = MOTOR_TYPE_HIGH_CURRENT);
    
    ~Motor();

    MOTOR_NAME name;
    Channel *channel;
    Motor *otherMotor;

    int encoder_cpr; //2000
    int encoder_max_rpm; //14500

    int32_t pole_pairs; //12
    int max_rpm; //7920
    int motor_max_current; //25
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
    void moveStartingPosition(double delayBetweenSteps, bool * signalFlag);

    int requestedState;

    void setPositionSetpoint(float newSetpoint);

<<<<<<< HEAD
    uint16_t help();

    void setTorque(float d);

    float castRadsToCPR(float rads);
    

    float castCurrentToTorque(float current);
    float castTorqueToCurrent(float torque);

    float getPosEstimateInRad();
    float getVelEstimateInRads();

    float getPosEstimateInRadDoublePendulum();

    void zeroPosition();

    void setOtherMotor(Motor * otherMotor);
>>>>>>> 1959d3619023fc5336c6d61448a30ecbac49514e
};

#endif //MOTOR_TFG_ADRIA