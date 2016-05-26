#include <boost/asio.hpp>
#include <azmq/socket.hpp>
#include <string>


class ITank
{
public:
    virtual void set_left_track_speed(int) = 0;
    virtual void set_right_track_speed(int) = 0;
};


class IController
{
    public:
        virtual void process_cmd() = 0;
};


class ZmqController : public IController
{
    public:
        ZmqController(boost::asio::io_service& loop, std::string s)
        : m_stream_sock(loop)
        {
            m_stream_sock.connect(s);
        }

    private:
        azmq::stream_socket m_stream_sock;
};


class Tank : public ITank
{
    public:
        Tank(int left_track_motor, int right_track_motor)
         : m_left_track_motor(left_track_motor),
           m_right_track_motor(right_track_motor)
        {

        }

        void process_cmd()
        {

        }

    private:
        int m_left_track_motor;
        int m_right_track_motor;
};
