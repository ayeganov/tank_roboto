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

const int TANK_SPEED = 255;

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
};


class ZmqController : public IController
{
    static const pt::millisec STOP_TANK_THRESHOLD;

    public:
        ZmqController(boost::asio::io_service& loop, std::string address, ITank* tank)
        : m_socket(loop),
          m_tank(tank),
          m_last_cmd_time(pt::microsec_clock::local_time()),
          m_stop_tank_cb(pt::milliseconds(50), loop)
        {
            m_socket.connect(address);
            m_socket.get_socket().set_option(azmq::socket::subscribe());
            m_socket.on_recv(std::bind(&ZmqController::process_cmd, this, std::placeholders::_1));
            m_stop_tank_cb.start([this](){ stop_tank_check(); } );
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

    private:
        roboutils::AzmqSock<azmq::sub_socket, 256> m_socket;
        ITank* m_tank;
        pt::ptime m_last_cmd_time;
        roboutils::PeriodicCallback m_stop_tank_cb;

        void stop_tank_check()
        {
            pt::ptime current_time = pt::microsec_clock::local_time();
            pt::time_duration td = current_time - m_last_cmd_time;
            if(m_tank && ZmqController::STOP_TANK_THRESHOLD <= td)
            {
                m_tank->stop();
            }
        }
};
const pt::millisec ZmqController::STOP_TANK_THRESHOLD = pt::millisec(100);


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
