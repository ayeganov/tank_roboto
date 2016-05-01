namespace robot
{
    enum class MotorPort
    {
        PORT_A = 0,
        PORT_B,
        PORT_C,
        PORT_D
    };

    class Motor
    {
    public:
        Motor(MotorPort port);
        void set_speed(int speed);
        void enable();
        void disable();
        bool is_enabled();
    private:
        MotorPort m_port;
        int m_speed;
        bool m_enabled;
    };
};
