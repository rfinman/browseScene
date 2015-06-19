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
        roll, pitch, yaw;
    pthread_mutex_t mlv, mlh, mrv, mrh, mroll, mpitch, myaw;

public:
    JoystickController();
    ~JoystickController();

    void setDevicePath(std::string);

    void startJoystick();

    int getLVert();
    int getLHoriz();
    int getRVert();
    int getRHoriz();
    int getRoll();
    int getPitch();
    int getYaw();
};
#endif
