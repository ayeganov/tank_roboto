#include "BrickPi.h"

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
