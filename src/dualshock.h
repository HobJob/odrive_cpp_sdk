//
// Created by adria on 5/5/20.
//

#ifndef TFG_ADRIA_DUALSHOCK_H
#define TFG_ADRIA_DUALSHOCK_H

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <unistd.h>

#include <mutex>

#define JOY_PATH   "/dev/input/js0"
#define TOUCH_PATH "/dev/input/mouse1"


struct {
    bool stopMotors = false;
    bool stopProgram = false;
}Remote;

std::mutex jsMutex;

bool getStopMotors(){
    std::lock_guard<std::mutex>guard(jsMutex);
    return Remote.stopMotors;
}
bool getStopProgram() {
    std::lock_guard<std::mutex>guard(jsMutex);
    return Remote.stopProgram;
}

void setStopMotors(bool val){
    std::lock_guard<std::mutex>guard(jsMutex);
    Remote.stopMotors = val;
}
void setStopProgram(bool stopProgram) {
    std::lock_guard<std::mutex>guard(jsMutex);
    Remote.stopProgram = stopProgram;
}



void decodeAxis(struct js_event event);

int dualshock_main()
{
    struct js_event event;
    struct input_event ie;
    unsigned char *ptr = (unsigned char*)&ie;

    int js, mouse;

    //Touch-Panel vars
    unsigned char button, bLeft, bMiddle, bRight;
    char Ax,Ay;
    int absolute_x = 0,absolute_y = 0;

    unsigned char exitProgram = 0;

    if ((js = open(JOY_PATH, O_RDONLY | O_NONBLOCK )) == -1){
        printf("Impossible to open joystick\n");
        return -1;
    }

    if((mouse = open(TOUCH_PATH, O_RDONLY | O_NONBLOCK )) == -1)
    {
        printf("Impossible to open touchPanel");
        return -1;
    }

    while(true)
    {
        if(read(js, &event, sizeof(struct js_event)) != -1)
        {

            switch (event.type)
            {
                case JS_EVENT_BUTTON:
                    //printf("Button %u %s %d\n", event.number, event.value ? "pressed" : "released",event.value);
                    if(event.number == 10){
                        setStopProgram(true);
                        exitProgram = 1;
                    }
                    if(event.number == 0) setStopMotors(event.value);
                    break;
                case JS_EVENT_AXIS:
                    //decodeAxis(event);

                    //printf("JOY %u %d\n", event.number, event.value);
                    break;
                default:
                    /* Ignore init events. */
                    //printf("Not recognized %u val: %d\n", event.number, event.value);
                    break;
            }
        }

        if(read(mouse, &ie, sizeof(struct input_event))!=-1)
        {
            button = ptr[0];
            bLeft = button & 0x1;
            bMiddle = ( button & 0x4 ) > 0;
            bRight = ( button & 0x2 ) > 0;
            Ax = (char) ptr[1];
            Ay = (char) ptr[2];

            absolute_x += Ax;
            absolute_y -= Ay;

            if (bLeft == 1)
            {
                absolute_x = 0;
                absolute_y = 0;
               // printf("Absolute x,y coords origin recorded\n");
            }
            // show it!
            printf("X%i Y%i\n",absolute_x,absolute_y);
            fflush(stdout);
        }

        if(mouse < 0 || js < 0 || exitProgram) break; // Funciona?
    }

    close(js);
    close(mouse);
    return 0;
}

void decodeAxis(struct js_event event){
    switch (event.number) {
        case 0: //joystick esquerra direccio esquerra minim

            break;
        case 1://joystick esquerra direccio amunt min
            break;
        case 3://joystick dreta direccio esquerra min
            break;
        case 4://joystick dreta direccio amunt min
            break;
        case 6://Gamepad direccio esquerra min
            break;
        case 7://Gamepad direccio amunt min
            break;
        case 2://Gatillo esquerra max apretat es positiu +32767
            break;
        case 5://Gatillo dreta igual
            break;
        default:
            printf("Mando desconegut!\n");
    }
}

#endif //ODRIVE_CPP_SDK_DUALSHOCK_H
