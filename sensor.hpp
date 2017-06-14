#ifndef __SENSOR__H__
#define __SENSOR__H__

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "robo_utils.hpp"


using namespace boost::asio;

namespace sensor
{

const std::string PORT_NAME = "/dev/ttyACM0";
const int BAUD_RATE = 115200;


constexpr double v(double x) { return 0; }
template <std::size_t... I>
constexpr std::array<double, sizeof...(I)> fill_array(std::index_sequence<I...>)
{
    return std::array<double, sizeof...(I)>{ v(I)... };
}

template <std::size_t N>
constexpr std::array<double, N> fill_array()
{
    return fill_array(std::make_index_sequence<N>{});
}

template<std::size_t N>
class SerialUltrasonicSensor
{
public:
    SerialUltrasonicSensor(io_service& loop, double alpha=0.4)
    : m_loop(loop),
      m_alpha(alpha),
      m_port(loop),
      m_sensors(fill_array<N>())
    {
        m_port.on_recv(std::bind(&SerialUltrasonicSensor::handle_data, this, std::placeholders::_1));
    }

    bool connect(const std::string& port = PORT_NAME, int baud_rate = BAUD_RATE)
    {
        return m_port.connect(port, baud_rate);
    }

    double sensor_reading(int idx)
    {
        if(idx < 0 || idx >= N)
        {
            throw new std::invalid_argument("Sensor index must fall within range [0," + std::to_string(N) + ")");
        }
        return m_sensors[idx];
    }

    bool is_connected() const
    {
        return m_port.is_open();
    }

private:
    io_service& m_loop;
    double m_alpha;
    roboutils::SerialPort m_port;
    std::array<double, N> m_sensors;

    void handle_data(std::string data)
    {
        std::vector<std::string> readings;
        boost::split(readings, data, boost::is_any_of(" "));

        if(readings.size() < N)
        {
            return;
        }

        for(auto reading : readings)
        {
            std::vector<std::string> idx_reading_tuple;
            boost::split(idx_reading_tuple, reading, boost::is_any_of("="));

            if(idx_reading_tuple.size() < 2)
            {
                return;
            }

            try
            {
                int idx = std::stoi(idx_reading_tuple[0]);
                double value = std::stod(idx_reading_tuple[1]);
                if(idx >= N || idx < 0)
                {
                    std::cerr << "Invalid sensor idx supplied: "
                              << idx
                              << ". Valid range: 0 to "
                              << N << std::endl;
                    return;
                }
                m_sensors[idx] = (value * m_alpha) + (1.0 - m_alpha) * m_sensors[idx];
                std::cout << idx << ": " << m_sensors[idx] << " ";
            }
            catch(...)
            {
                std::cerr << "Unable to parse data: " << reading << std::endl;
            }
        }
        std::cout << std::endl;
    }
};

};

#endif
