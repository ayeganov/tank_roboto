#ifndef _COMM_CONTROLLERS_
#define _COMM_CONTROLLERS_

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <azmq/socket.hpp>
#include <string>
#include <array>
#include <iostream>

#include "robo_utils.hpp"

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
        ZmqController(boost::asio::io_service& loop, std::string address)
        : m_socket(loop)
        {
            m_socket.connect(address);
            m_socket.on_recv([](std::string&& data)
            {
                std::cout << "Got some data: " << data << std::endl;
            });
        }

        void process_cmd()
        {

        }

    private:
        roboutils::AzmqSock<azmq::pair_socket, 256> m_socket;
};


class Tank : public ITank
{
    public:
        Tank(int left_track_motor, int right_track_motor)
         : m_left_track_motor(left_track_motor),
           m_right_track_motor(right_track_motor)
        {

        }

    private:
        int m_left_track_motor;
        int m_right_track_motor;
};
#endif
