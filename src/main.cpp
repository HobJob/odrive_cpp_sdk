#include <iostream>
#include "robot.h"
#include "dualshock.h"
#include <thread>



//valgrind --leak-check=full --show-leak-kinds=all ./odrive_test
int main(int argc, const char * argv[]){

    std::thread remote_thread(&dualshock_main);

    Robot r = Robot();
    auto *m0 = new Motor(M0);
    auto *m1 = new Motor(M1);

    //Check if there is any odrive connected.
    if(r.odrives.empty()) return -1;
    auto odrive = r.odrives[0];

    //Load the default configuration for 2 TAROT MT4008 motors.
    //r.configureODrive(odrive, m0, m1);

    //Configure both motors
    odrive->configureMotor(m0);
    odrive->configureMotor(m1);

    float voltage = odrive->inputVoltage();
    std::cout <<"Voltage after is " << voltage << std::endl;

    odrive->m0->setControlMode(CTRL_MODE_POSITION_CONTROL);
    odrive->m0->setRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL);

    bool stopProgram;
    bool stuff = false;
    float setpoint = 0.0f;
    float speed = 1.0f;
    bool oneTime = true;

    do{
        bool stopMotors = getStopMotors();

        if(stopMotors){
            odrive->m0->disable();
            odrive->m1->disable();
            oneTime = true;
        }else{
            if(oneTime){
                odrive->m0->enable();
                odrive->m1->enable();
                oneTime = false;
            }
        }

        stopProgram     = getStopProgram();
        odrive->m0->setPositionSetpoint(setpoint);
        setpoint += speed;
        if(setpoint >= 2000.0f|| setpoint <= 0.0f){
            speed *= -1;
        }
        //float poseEstimate = odrive->m0->getPosEstimate();
        //std::cout <<"Pose estimate" << poseEstimate << std::endl;
        usleep(10000);
    }while(!stopProgram);

    odrive->m0->disable();
    odrive->m1->disable();

    remote_thread.join();
    return 0;
}


/*
int main(int argc, const char * argv[]) {
  std::string odrive_serial_numbers[1] = {"35700616219979"};
  std::string odrive_serial_numbers_map[2] = {"35700616219979","35700616219979"};
  int16_t zeroeth_radian_in_encoder_ticks_[2] = { -200, 0 };

  bool odrive_position_per_motor[2] = {false, true};
  bool motor_relative_to_prior_motor[2] = {false, false};
  // odrive_encoder_ticks_per_radian_per_motor lets us account for any gear reductions...
  float odrive_encoder_ticks_per_radian_per_motor[2] = { 57.2958 * (2048 * 4) / 360.0, 57.2958 * (2048 * 4) / 360.0 };
  odrive::Channel odrive_cpp_sdk(
        odrive_serial_numbers,
        1,
        odrive_serial_numbers_map,
        odrive_position_per_motor,
        odrive_encoder_ticks_per_radian_per_motor,
        motor_relative_to_prior_motor,
        2
        );
  std::cout << "odrive_cpp_sdk constructed" << std::endl;

  odrive_cpp_sdk.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

  int result = odrive_cpp_sdk.init();
  std::cout << "odrive_cpp_sdk.init got: " << result << std::endl;

  if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
    std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    return EXIT_FAILURE;
  }

  float voltage;
  result = odrive_cpp_sdk.readCurrentVBusVoltage(voltage);
  std::cout << "odrive_cpp_sdk.readCurrentVBUsVoltage got result: " << result << " and value: [" << std::to_string(voltage) << "]" << std::endl;

  //    uint8_t odrive_motor_current_errors[2] = {0};
  //    result = odrive_cpp_sdk.checkErrors(odrive_motor_current_errors);
  //    std::cout << "odrive_cpp_sdk.checkErrors got result:" << result << " and value: [" << std::to_string(odrive_motor_current_errors[0]) << "," << std::to_string(odrive_motor_current_errors[1]) << "]" << std::endl;

  //    double odrive_motor_current_positions[2] = {0};
  //    result = odrive_cpp_sdk.readCurrentMotorPositions(odrive_motor_current_positions);
  //    std::cout << "odrive_cpp_sdk.readCurrentMotorPositions got result:" << result << " and value: [" << std::to_string(odrive_motor_current_positions[0]) << "," << std::to_string(odrive_motor_current_positions[1]) << "]" << std::endl;

  //    double odrive_motor_cmd_positions[2] = { 3.14159 / 2.0, 3.14159 / 2.0 };
  //    result = odrive_cpp_sdk.setGoalMotorPositions(odrive_motor_cmd_positions);
  //    std::cout << "odrive_cpp_sdk.setGoalMotorPositions got result:" << result << std::endl;

  //    odrive_motor_current_errors[0] = 0;
  //    odrive_motor_current_errors[1] = 0;
  //    result = odrive_cpp_sdk.checkErrors(odrive_motor_current_errors);
  //    std::cout << "odrive_cpp_sdk.checkErrors got result:" << result << " and value: [" << std::to_string(odrive_motor_current_errors[0]) << "," << std::to_string(odrive_motor_current_errors[1]) << "]" << std::endl;

  //    odrive_motor_current_positions[0] = 0;
  //    odrive_motor_current_positions[1] = 0;
  //    result = odrive_cpp_sdk.readCurrentMotorPositions(odrive_motor_current_positions);
  //    std::cout << "odrive_cpp_sdk.readCurrentMotorPositions got result:" << result << " and value: [" << std::to_string(odrive_motor_current_positions[0]) << "," << std::to_string(odrive_motor_current_positions[1]) << "]" << std::endl;
  }
*/