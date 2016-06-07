#include <algorithm>

#include "BrickPi.h"
#include "motor.h"

namespace robot
{
    Motor::Motor(int port)
    : m_port(port),
      m_speed(0),
      m_enabled(false)
    {
        enable();
    }

    void Motor::enable()
    {
        get_brick().MotorEnable[m_port] = true;
        m_enabled = true;
    }

    void Motor::disable()
    {
        get_brick().MotorEnable[m_port] = false;
        m_enabled = false;
    }

    void Motor::set_speed(int speed)
    {
        speed = std::min(255, std::max(speed, -255));
        get_brick().MotorSpeed[m_port] = speed;
        m_speed = speed;
    }

    int Motor::get_speed() const
    {
        return m_speed;
    }

    bool Motor::is_enabled()
    {
        return m_enabled;
    }
};
