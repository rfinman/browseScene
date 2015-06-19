#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <linux/joystick.h>
#include <string>
#include <sigc++/sigc++.h>
#include <glibmm-2.4/glibmm/thread.h>

using std::string;

enum axes{
    LEFT_HORIZONTAL, 
    LEFT_VERTICAL,
    RIGHT_HORIZONTAL,
    RIGHT_VERTICAL,
    IMU_ROLL,
    IMU_PITCH,
    IMU_YAW
};

enum buttons{
    // TODO: Fill in if needed
};

struct EventJoystick
{
    int32_t time;
    int16_t value;
    int8_t number;
    bool synthetic;
};

class Joystick 
{
    public:
        sigc::signal <void, const EventJoystick&> signalAxis;
        sigc::signal <void, const EventJoystick&> signalButton;

        Joystick ();
        virtual ~Joystick ();

        bool open (const string &device);
        bool close ();

        int getNumberOfButtons ();
        int getNumberOfAxes ();
        const string &getIdentifier ();

    private: 
        Joystick             (const Joystick&);
        Joystick& operator = (const Joystick&);

    private:
        struct js_event joy_event;
        int m_fd;
        Glib::Thread *thread;
        bool m_init;
        int m_axes;
        int m_buttons;
        string m_name;
        bool m_run;

        void loop ();
};

#endif // JOYSTICK_H
