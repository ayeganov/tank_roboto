#include <cassert>

#include "BrickPi.h"
#include "motor.h"

namespace robot
{
    Motor::Motor(MotorPort port)
    : m_port(port),
      m_speed(0),
      m_enabled(false)
    {
        enable();
    }

    void Motor::enable()
    {
        get_brick().MotorEnable[static_cast<int>(m_port)] = true;
        m_enabled = true;
    }

    void Motor::disable()
    {
        get_brick().MotorEnable[static_cast<int>(m_port)] = false;
        m_enabled = false;
    }

    void Motor::set_speed(int speed)
    {
        assert(speed <= 255 && speed >= -255);
        get_brick().MotorSpeed[static_cast<int>(m_port)] = speed;
        m_speed = speed;
    }

    bool Motor::is_enabled()
    {
        return m_enabled;
    }
};
