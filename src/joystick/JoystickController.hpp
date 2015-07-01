#ifndef HEADER_JOYSTICKCONTROLLER_HPP
#define HEADER_JOYSTICKCONTROLLER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "joystick.h"

//std::mutex mlv, mlh, mrv, mrh, mroll, mpitch, myaw;
class JoystickController
{
private:
    std::string default_path, device_path;
    bool device_path_set;

    void axisCallback(const EventJoystick);
    void buttonCallback(const EventJoystick);

    int l_vert, l_horiz, r_vert, r_horiz,
        roll, pitch, yaw, r_trigger, l_trigger;
    pthread_mutex_t mlv, mlh, mrv, mrh, mroll, mpitch, myaw, mlt, mrt;

public:
    JoystickController();
    ~JoystickController();

    void setDevicePath(std::string);

    void startJoystick();

    float getLVert();
    float getLHoriz();
    float getRVert();
    float getRHoriz();
    float getRoll();
    float getPitch();
    float getYaw();

    float getLTrigger();
    float getRTrigger();
};
#endif
