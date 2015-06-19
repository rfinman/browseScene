#include "JoystickController.hpp"

JoystickController::JoystickController():device_path_set(false),
                                         default_path("/dev/input/js0"),
                                         l_vert(0),
                                         l_horiz(0),
                                         r_vert(0),
                                         r_horiz(0),
                                         roll(0),
                                         pitch(0),
                                         yaw(0)
{
    pthread_mutex_init(&mlv, NULL);    
    pthread_mutex_init(&mlh, NULL);    
    pthread_mutex_init(&mrv, NULL);    
    pthread_mutex_init(&mrh, NULL);    
    pthread_mutex_init(&mroll, NULL);    
    pthread_mutex_init(&mpitch, NULL);    
    pthread_mutex_init(&myaw, NULL);
   /* 
    std::cout << "Starting sixad"<<std::endl;
    if (system("sixad --start") <= 0)
    {
        std::cout << "Running sixad --start failed, " <<
                     "joystick not working" << std::endl;
        exit(1);
    }
    std::cout <<"Joystick set up" <<std::endl;
    */
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
}

void JoystickController::setDevicePath(std::string path)
{
    device_path.clear();
    device_path.insert(0,path);
    device_path_set = true;
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
            //std::cout <<"l_vert = "<<l_vert<<std::endl;
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
    // TODO Set up buttons if needbe

    pause();
}

int JoystickController::getLVert()
{
    int val;
    pthread_mutex_lock(&mlv);
    val = l_vert;
    pthread_mutex_unlock(&mlv);
    //std::cout<<l_vert<<" val = "<<val<<" and "<<(float)val/32768000.0<<std::endl;
    return val;
}

int JoystickController::getLHoriz()
{
    int val;
    pthread_mutex_lock(&mlh);
    val = l_horiz;
    pthread_mutex_unlock(&mlh);
    return val;
}

int JoystickController::getRVert()
{
    int val;
    pthread_mutex_lock(&mrv);
    val = r_vert;
    pthread_mutex_unlock(&mrv);
    return val;
}

int JoystickController::getRHoriz()
{
    int val;
    pthread_mutex_lock(&mrh);
    val = r_horiz;
    pthread_mutex_unlock(&mrh);
    return val;
}

int JoystickController::getRoll()
{
    int val;
    pthread_mutex_lock(&mroll);
    val = roll;
    pthread_mutex_unlock(&mroll);
    return val;
}

int JoystickController::getPitch()
{
    int val;
    pthread_mutex_lock(&mpitch);
    val = pitch;
    pthread_mutex_unlock(&mpitch);
    return val;
}

int JoystickController::getYaw()
{
    int val;
    pthread_mutex_lock(&myaw);
    val = yaw;
    pthread_mutex_unlock(&myaw);
    return val;
}
