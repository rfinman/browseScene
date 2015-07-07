#include "JoystickController.hpp"

JoystickController::JoystickController():device_path_set(false),
                                         default_path("/dev/input/js0"),
                                         l_vert(0),
                                         l_horiz(0),
                                         r_vert(0),
                                         r_horiz(0),
                                         roll(0),
                                         pitch(0),
                                         yaw(0),
                                         l_trigger(0),
                                         r_trigger(0)
{
    pthread_mutex_init(&mlv, NULL);    
    pthread_mutex_init(&mlh, NULL);    
    pthread_mutex_init(&mrv, NULL);    
    pthread_mutex_init(&mrh, NULL);    
    pthread_mutex_init(&mroll, NULL);    
    pthread_mutex_init(&mpitch, NULL);    
    pthread_mutex_init(&myaw, NULL);

    pthread_mutex_init(&mlt, NULL);
    pthread_mutex_init(&mrt, NULL);
}

JoystickController::~JoystickController()
{
    pthread_mutex_destroy(&mlv);    
    pthread_mutex_destroy(&mlh);    
    pthread_mutex_destroy(&mrv);    
    pthread_mutex_destroy(&mrh);    
    pthread_mutex_destroy(&mroll);    
    pthread_mutex_destroy(&mpitch);    
    pthread_mutex_destroy(&myaw);    

    pthread_mutex_destroy(&mlt);    
    pthread_mutex_destroy(&mrt);    
}

void JoystickController::setDevicePath(std::string path)
{
    device_path.clear();
    device_path.insert(0,path);
    device_path_set = true;
}

void JoystickController::buttonCallback(const EventJoystick eventJoy)
{
    switch (eventJoy.number)
    {
        case LEFT_TRIGGER:
            pthread_mutex_lock(&mlt);
            l_trigger = eventJoy.value;
            pthread_mutex_unlock(&mlt);
            break;
        case RIGHT_TRIGGER:
            pthread_mutex_lock(&mrt);
            r_trigger = eventJoy.value;
            pthread_mutex_unlock(&mrt);
            break;
    }

}

void JoystickController::axisCallback(const EventJoystick eventJoy)
{
    //eventJoy.time is the time
    //enentJoy.value is the value of the axis
    //eventJoy.number is the axes number
    switch (eventJoy.number)
    {
        case LEFT_VERTICAL:
            pthread_mutex_lock(&mlv);
            l_vert = eventJoy.value;
            pthread_mutex_unlock(&mlv);
            break;
        case LEFT_HORIZONTAL:
            pthread_mutex_lock(&mlh);
            l_horiz = eventJoy.value;
            pthread_mutex_unlock(&mlh);
            break;
        case RIGHT_HORIZONTAL:
            pthread_mutex_lock(&mrh);
            r_horiz = eventJoy.value;
            pthread_mutex_unlock(&mrh);
            break;
        case RIGHT_VERTICAL:
            pthread_mutex_lock(&mrv);
            r_vert = eventJoy.value;
            pthread_mutex_unlock(&mrv);
            break;
        case IMU_ROLL:
            pthread_mutex_lock(&mroll);
            roll = eventJoy.value;
            pthread_mutex_unlock(&mroll);
            break;
        case IMU_PITCH:
            pthread_mutex_lock(&mpitch);
            pitch = eventJoy.value;
            pthread_mutex_unlock(&mpitch);
            break;
        case IMU_YAW:
            pthread_mutex_lock(&myaw);
            yaw = eventJoy.value;
            pthread_mutex_unlock(&myaw);
            break;
    }
}


void JoystickController::startJoystick()
{
    if (!device_path_set)
    {
        std::cout<<"No path set, using default "<<default_path<<std::endl;
        this->setDevicePath(default_path);
    }

    Joystick joystick;

    EventJoystick m_eventButton;
    EventJoystick m_eventAxis;

    joystick.open(device_path);

    int numButtons = joystick.getNumberOfButtons();
    int numAxis = joystick.getNumberOfAxes();
    std::cout << "Joystick: "<<joystick.getIdentifier()<<std::endl;

    joystick.signalAxis.connect(sigc::mem_fun(*this, &JoystickController::axisCallback));
    joystick.signalButton.connect(sigc::mem_fun(*this, 
    &JoystickController::buttonCallback));

    pause();
}

float JoystickController::getLVert()
{
    int val;
    pthread_mutex_lock(&mlv);
    val = l_vert;
    pthread_mutex_unlock(&mlv);
    return (float)val/32768.0;
}

float JoystickController::getLHoriz()
{
    int val;
    pthread_mutex_lock(&mlh);
    val = l_horiz;
    pthread_mutex_unlock(&mlh);
    return (float)val/32768.0;
}

float JoystickController::getRVert()
{
    int val;
    pthread_mutex_lock(&mrv);
    val = r_vert;
    pthread_mutex_unlock(&mrv);
    return (float)val/32768.0;
}

float JoystickController::getRHoriz()
{
    int val;
    pthread_mutex_lock(&mrh);
    val = r_horiz;
    pthread_mutex_unlock(&mrh);
    return (float)val/32768.0;
}

float JoystickController::getRoll()
{
    int val;
    pthread_mutex_lock(&mroll);
    val = roll;
    pthread_mutex_unlock(&mroll);
    return (float)val/32768.0;
}

float JoystickController::getPitch()
{
    int val;
    pthread_mutex_lock(&mpitch);
    val = pitch;
    pthread_mutex_unlock(&mpitch);
    return (float)val/32768.0;
}

float JoystickController::getYaw()
{
    int val;
    pthread_mutex_lock(&myaw);
    val = yaw;
    pthread_mutex_unlock(&myaw);
    return (float)val/32768.0;
}

float JoystickController::getLTrigger()
{
    int val;
    pthread_mutex_lock(&mlt);
    val = l_trigger;
    pthread_mutex_unlock(&mlt);
    return (float)val;
}

float JoystickController::getRTrigger()
{
    int val;
    pthread_mutex_lock(&mrt);
    val = r_trigger;
    pthread_mutex_unlock(&mrt);
    return (float)val;
}
