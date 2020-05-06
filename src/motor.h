#ifndef MOTOR_TFG_ADRIA
#define MOTOR_TFG_ADRIA

#include <stdint.h>
#include "channel.h"

//From enums.py
#define MOTOR_TYPE_HIGH_CURRENT 0

enum MOTOR_NAME{
    M0,
    M1
};

class Channel;
//TODO: Maybe more indicated to rename it to Axis.
class Motor{

public:
    Motor();
    Motor(MOTOR_NAME name, int encoder_cpr = 2000, int encoder_max_rpm = 14500,
        uint8_t pole_pairs = 12,
        int max_rpm = 7920,
        int max_current = 25,
        float kV = 330.0f,
        float calibration_current = 5,
        float current_limit = 5,
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
    void disable();

    int motor_type;

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

    int requestedState;

    void setPositionSetpoint(float newSetpoint);
};

#endif //MOTOR_TFG_ADRIA