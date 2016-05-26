#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <azmq/socket.hpp>
#include <string>
#include <array>
#include <iostream>

#include "periodic_callback.hpp"

namespace pt = boost::posix_time;
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
        : m_stream_sock(loop),
          m_pc(pt::seconds(2), loop)
        {
            m_stream_sock.connect(s);
            m_stream_sock.async_receive(boost::asio::buffer(m_receive_buffer), boost::bind(&ZmqController::handle_receive, this, asio::placeholders::error));

        }

        void handle_receive(boost::system::error_code ec, size_t bytes_transferred)
        {
            if(ec)
            {
                return;
            }
            std::cout << "Got message: " << m_receive_buffer.data() << '\n';
        }

    private:
        azmq::pair_socket m_stream_sock;
        std::array<256, char> m_receive_buffer;
        robot::PeriodicCallback m_pc;
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
