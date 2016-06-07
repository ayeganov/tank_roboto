#ifndef _COMM_CONTROLLERS_
#define _COMM_CONTROLLERS_

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <azmq/socket.hpp>
#include <string>
#include <array>
#include <iostream>

#include "robo_utils.hpp"
#include "motor.h"
#include "BrickPi.h"


namespace pt = boost::posix_time;

const int TANK_SPEED = 200;

class ITank
{
public:
    virtual void set_left_track_speed(int) = 0;
    virtual void set_right_track_speed(int) = 0;
    virtual void stop() = 0;
};


class IController
{
    public:
        virtual void process_cmd(std::string cmd) = 0;
        virtual pt::ptime get_last_cmd_time() const = 0;
};


class ZmqController : public IController
{
    public:
        ZmqController(boost::asio::io_service& loop, std::string address, ITank* tank)
        : m_socket(loop),
          m_tank(tank),
          m_last_cmd_time(pt::microsec_clock::local_time())
        {
            m_socket.connect(address);
            m_socket.get_socket().set_option(azmq::socket::subscribe());
            m_socket.on_recv(std::bind(&ZmqController::process_cmd, this, std::placeholders::_1));
        }

        void process_cmd(std::string cmd)
        {
            if(cmd.find('w') != std::string::npos)
            {
                m_tank->set_left_track_speed(TANK_SPEED);
                m_tank->set_right_track_speed(TANK_SPEED);
            }
            else if(cmd.find('a') != std::string::npos)
            {
                m_tank->set_left_track_speed(-TANK_SPEED);
                m_tank->set_right_track_speed(TANK_SPEED);
            }
            else if(cmd.find('d') != std::string::npos)
            {
                m_tank->set_left_track_speed(TANK_SPEED);
                m_tank->set_right_track_speed(-TANK_SPEED);
            }
            else if(cmd.find('s') != std::string::npos)
            {
                m_tank->set_left_track_speed(-TANK_SPEED);
                m_tank->set_right_track_speed(-TANK_SPEED);
            }
            else
            {
                std::cerr << "Unknown command: " << cmd << std::endl;
            }
            m_last_cmd_time = pt::microsec_clock::local_time();
        }

        pt::ptime get_last_cmd_time() const
        {
            return m_last_cmd_time;
        }

    private:
        roboutils::AzmqSock<azmq::sub_socket, 256> m_socket;
        ITank* m_tank;
        pt::ptime m_last_cmd_time;
};


class Tank : public ITank
{
    public:
        Tank(int left_track_motor, int right_track_motor)
         : m_left_motor(left_track_motor),
           m_right_motor(right_track_motor)
        {}

        void set_left_track_speed(int speed)
        {
            m_left_motor.set_speed(speed);
        }

        void set_right_track_speed(int speed)
        {
            m_right_motor.set_speed(speed);
        }

        void stop()
        {
            m_right_motor.set_speed(0);
            m_left_motor.set_speed(0);
        }

    private:
        robot::Motor m_left_motor;
        robot::Motor m_right_motor;
};
#endif
