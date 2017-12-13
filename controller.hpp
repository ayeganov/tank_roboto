#ifndef _COMM_CONTROLLERS_
#define _COMM_CONTROLLERS_

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <azmq/socket.hpp>
#include <neatnet/genalg.h>
#include <neatnet/phenotype.h>

#include "json.hpp"

#include "brick_state.hpp"
#include "motor.h"
#include "robo_utils.hpp"
#include "sensor.hpp"


using namespace boost;


const int MAX_SPEED = 255;
static const std::string FORWARD = "w";
static const std::string BACKWARD = "s";
static const std::string LEFT = "a";
static const std::string RIGHT = "d";
const std::string PORT_NAME = "/dev/ttyACM0";
const int BAUD_RATE = 115200;

const int USONIC_MAX = 4;
const int HEADING = 5;
const int RATE_GYR_X = 6;
const int RATE_GYR_Y = 7;
const int RATE_GYR_Z = 8;
const int GYRO_ANGLE_X = 9;
const int GYRO_ANGLE_Y = 10;
const int GYRO_ANGLE_Z = 11;
const int ACC_ANGLE_X = 12;
const int ACC_ANGLE_Y = 13;
const int CF_ANGLE_X = 14;
const int CF_ANGLE_Y = 15;


class ITank
{
public:
    virtual void set_left_track_speed(int) = 0;
    virtual void set_right_track_speed(int) = 0;
    virtual void set_left_track_speed(double) = 0;
    virtual void set_right_track_speed(double) = 0;
    virtual void stop() = 0;
};


class IController
{
    public:
        template <typename INPUT>
        void process_cmd(INPUT cmd);
};


class ZmqController : public IController
{
    static const posix_time::millisec STOP_TANK_THRESHOLD;

    public:
        ZmqController(boost::asio::io_service& loop, std::string address, ITank& tank)
        : m_socket(loop),
          m_tank(tank),
          m_last_cmd_time(posix_time::microsec_clock::local_time()),
          m_stop_tank_cb(posix_time::milliseconds(50), loop)
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
                m_tank.set_left_track_speed(MAX_SPEED);
                m_tank.set_right_track_speed(MAX_SPEED);
            }
            else if(cmd.find('a') != std::string::npos)
            {
                m_tank.set_left_track_speed(-MAX_SPEED);
                m_tank.set_right_track_speed(MAX_SPEED);
            }
            else if(cmd.find('d') != std::string::npos)
            {
                m_tank.set_left_track_speed(MAX_SPEED);
                m_tank.set_right_track_speed(-MAX_SPEED);
            }
            else if(cmd.find('s') != std::string::npos)
            {
                m_tank.set_left_track_speed(-MAX_SPEED);
                m_tank.set_right_track_speed(-MAX_SPEED);
            }
            else
            {
                std::cerr << "Unknown command: " << cmd << std::endl;
            }
            m_last_cmd_time = posix_time::microsec_clock::local_time();
        }

    private:
        roboutils::AzmqSock<azmq::sub_socket, 256> m_socket;
        ITank& m_tank;
        posix_time::ptime m_last_cmd_time;
        roboutils::PeriodicCallback m_stop_tank_cb;

        void stop_tank_check()
        {
            posix_time::ptime current_time = posix_time::microsec_clock::local_time();
            posix_time::time_duration td = current_time - m_last_cmd_time;
            if(ZmqController::STOP_TANK_THRESHOLD <= td)
            {
                m_tank.stop();
            }
        }
};
const posix_time::millisec ZmqController::STOP_TANK_THRESHOLD = posix_time::millisec(100);


class SensorController : public IController
{
    public:

        SensorController(roboutils::UltraSonicSensor& sensor,
                         ITank& tank,
                         roboutils::BrickState& brick_state)
        : m_sensor(sensor),
          m_tank(tank),
          m_prev_cmd("")
        {
            brick_state.on_state_update(roboutils::NotifyType::CONTINUOUS,
                                        &SensorController::control_robot,
                                        this);
        }

    private:
        void control_robot()
        {
            m_sensor.update_reading();
            double value = m_sensor.get_average_value();
            if(value > 35)
            {
                process_cmd(FORWARD);
            }
            else
            {
                process_cmd(RIGHT);
            }
        }

        void process_cmd(std::string cmd)
        {
            if(cmd == FORWARD)
            {
                if(cmd != m_prev_cmd)
                {
                    std::cout << "Moving forward.\n";
                    std::cout << "Distance: " << m_sensor.get_average_value() << '\n';
                }
                m_tank.set_left_track_speed(1.0);
                m_tank.set_right_track_speed(1.0);
                m_prev_cmd = cmd;
            }
            else if(cmd == RIGHT)
            {
                if(cmd != m_prev_cmd)
                {
                    std::cout << "Turning right.\n";
                    std::cout << "Distance: " << m_sensor.get_average_value() << '\n';
                }
                m_tank.set_left_track_speed(1.0);
                m_tank.set_right_track_speed(-1.0);
                m_prev_cmd = cmd;
            }
        }

        roboutils::UltraSonicSensor& m_sensor;
        ITank& m_tank;
        std::string m_prev_cmd;
};


class NeuralController : public IController
{
    public:
        NeuralController(const std::string& net_path, ITank& tank, boost::asio::io_service& loop,
                         roboutils::BrickState& brick_state)
        : m_serial_ultra_sensor(loop),
          m_tank(tank)
        {
            m_neural_net = std::make_shared<neat::NeuralNet>(net_path);
            brick_state.on_state_update(roboutils::NotifyType::CONTINUOUS,
                                        &NeuralController::control_robot,
                                        this);
            m_serial_ultra_sensor.connect();
        }

        void control_robot()
        {
            std::vector<double> readings;
            for(int sensor_id = 0; sensor_id < 5; ++sensor_id)
            {
                readings.push_back(m_serial_ultra_sensor.sensor_reading(sensor_id));
            }
            // simulation model numbering is in revers to hardware
            std::reverse(readings.begin(), readings.end());
            process_cmd(readings);
        }

        void process_cmd(std::vector<double> cmd)
        {
            auto input = convert_to_net_input(cmd);
            auto output = m_neural_net->Update(input);
            std::cout << "Left: " << output[0] << ", Right: " << output[1] << std::endl;
            m_tank.set_left_track_speed(output[0]);
            m_tank.set_right_track_speed(output[1]);
        }

    private:
        sensor::SerialUltrasonicSensor<5> m_serial_ultra_sensor;
        ITank& m_tank;
        neat::SNeuralNetPtr m_neural_net;

        ///=========== methods ============
        std::vector<double> convert_to_net_input(std::vector<double>readings)
        {
            double collision_start = 25;
            double collision_end = 10;
            double range = collision_start - collision_end;
            double collided = 0;
            std::vector<double> net_input;

            for(double r : readings)
            {
                if(r <= collision_end)
                {
                    collided = 1;
                    net_input.push_back(1);
                }
                else if(r <= collision_start)
                {
                    auto depth = (r - collision_end) / range;
                    net_input.push_back(depth);
                }
                else
                {
                    net_input.push_back(-1);
                }
                // no need to add -1 for feelers
//                net_input.push_back(-1);
            }
            net_input.push_back(collided);
            net_input.push_back(0);
            std::cout << "input: ";
            for(auto v : net_input)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
            return net_input;
        }
};


class SensorPublisher
{
    public:
        SensorPublisher(const std::string& odom_address,
                        const std::string& imu_address,
                        boost::asio::io_service& loop,
                        roboutils::BrickState& brick_state)
        : m_prev_left(0),
          m_prev_right(0),
          m_odom_pub{loop},
          m_imu_pub{loop},
          m_serial_ultra_sensor{loop}
        {
            m_prev_left = get_brick().Encoder[PORT_D];
            m_prev_right = get_brick().Encoder[PORT_A];
            m_odom_pub.bind(odom_address);
            m_imu_pub.bind(imu_address);
            brick_state.on_state_update(roboutils::NotifyType::CONTINUOUS,
                                        &SensorPublisher::publish_readings,
                                        this);
            m_serial_ultra_sensor.connect();
        }

        void publish_readings()
        {
            try
            {
                long left_track = get_brick().Encoder[PORT_D];
                long right_track = get_brick().Encoder[PORT_A];

                if((left_track == m_prev_left && right_track == m_prev_right)
                   || (left_track == 0 || right_track == 0))
                {
                    return;
                }

                nlohmann::json msg;
                nlohmann::json ultra_sonic;
                nlohmann::json odometry;
                nlohmann::json imu;

                for(int sensor_id = 0; sensor_id <= USONIC_MAX; ++sensor_id)
                {
                    ultra_sonic[sensor_id] = m_serial_ultra_sensor.sensor_reading(sensor_id);
                }

                odometry["left"] = m_prev_left = left_track;
                odometry["right"] = m_prev_right = right_track;
                imu["heading"] = m_serial_ultra_sensor.sensor_reading(HEADING);
                imu["rate_gyr_x"] = m_serial_ultra_sensor.sensor_reading(RATE_GYR_X);
                imu["rate_gyr_y"] = m_serial_ultra_sensor.sensor_reading(RATE_GYR_Y);
                imu["rate_gyr_z"] = m_serial_ultra_sensor.sensor_reading(RATE_GYR_Z);
                imu["gyro_angle_x"] = m_serial_ultra_sensor.sensor_reading(GYRO_ANGLE_X);
                imu["gyro_angle_y"] = m_serial_ultra_sensor.sensor_reading(GYRO_ANGLE_Y);
                imu["gyro_angle_z"] = m_serial_ultra_sensor.sensor_reading(GYRO_ANGLE_Z);
                imu["acc_angle_x"] = m_serial_ultra_sensor.sensor_reading(ACC_ANGLE_X);
                imu["acc_angle_y"] = m_serial_ultra_sensor.sensor_reading(ACC_ANGLE_Y);
                imu["cf_angle_x"] = m_serial_ultra_sensor.sensor_reading(CF_ANGLE_X);
                imu["cf_angle_y"] = m_serial_ultra_sensor.sensor_reading(CF_ANGLE_Y);

                msg["ultrasonic"] = ultra_sonic;
                msg["odometry"] = odometry;

                m_odom_pub.get_socket().send(boost::asio::buffer(msg.dump()));
                m_imu_pub.get_socket().send(boost::asio::buffer(imu.dump()));
            }
            catch(std::exception& e)
            {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }

    private:
        const static int NUM_SENSORS = 16;
        long m_prev_left;
        long m_prev_right;
        roboutils::AzmqSock<azmq::pub_socket, 256> m_odom_pub;
        roboutils::AzmqSock<azmq::pub_socket, 256> m_imu_pub;
        sensor::SerialUltrasonicSensor<NUM_SENSORS> m_serial_ultra_sensor;
};

template<typename T>
T max(T l, T r)
{
    return l > r ? l : r;
}


template<typename T>
T min(T l, T r)
{
    return l < r ? l : r;
}


class Tank : public ITank
{
    public:
        Tank(int left_track_motor, int right_track_motor)
         : m_left_motor(left_track_motor),
           m_right_motor(right_track_motor)
        {}

        void set_left_track_speed(int speed) override
        {
            m_left_motor.set_speed(speed);
        }

        void set_right_track_speed(int speed) override
        {
            m_right_motor.set_speed(speed);
        }

        void set_left_track_speed(double speed) override
        {
            speed = min(1.0, max(speed, -1.0));
            m_left_motor.set_speed(speed * MAX_SPEED);
        }

        void set_right_track_speed(double speed) override
        {
            speed = min(1.0, max(speed, -1.0));
            m_right_motor.set_speed(speed * MAX_SPEED);
        }

        void stop() override
        {
            m_right_motor.set_speed(0);
            m_left_motor.set_speed(0);
        }

    private:
        robot::Motor m_left_motor;
        robot::Motor m_right_motor;
};
#endif
