add_library(odrive_cpp src/robot.cpp src/robot.h src/motor.cpp src/motor.h src/ODrive.cpp 
src/ODrive.h src/channel.cpp src/channel.h src/json.hpp src/dualshock.h)

target_include_directories(odrive_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} plot)

target_link_libraries(odrive_cpp LINK_PUBLIC ${LIBUSB1_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_THREAD_LIBS_INIT} plot DoublePendulumMPC)
