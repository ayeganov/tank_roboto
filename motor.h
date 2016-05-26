#include "BrickPi.h"

#ifndef __MOTOR__H__
#define __MOTOR__H__

namespace robot
{
    class Motor
    {
    public:
        Motor(int port);
        void set_speed(int speed);
        int get_speed() const;
        void enable();
        void disable();
        bool is_enabled();
    private:
        int m_port;
        int m_speed;
        bool m_enabled;
    };
};
#endif
