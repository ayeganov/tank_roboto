#ifndef __BRICK_STATE_H__
#define __BRICK_STATE_H__

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <functional>
#include <vector>
#include <utility>
#include <tuple>
#include "robo_utils.hpp"
#include "BrickPi.h"


namespace roboutils
{

    enum NotifyType
    {
        ONCE,
        CONTINUOUS
    };

class BrickState
{
    public:
        typedef std::pair<std::function<void()>, NotifyType> Callback ;
        BrickState(boost::posix_time::time_duration td, asio::io_service& loop)
        : m_state_update(td, loop)
        {
            m_state_update.start(&BrickState::update_state, this);
        }

        template<class F, class... Args>
        void on_state_update(NotifyType notify, F&& f, Args&&... args)
        {
            std::function<void()> func = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
            Callback p = std::make_pair(func, notify);
            m_state_callbacks.emplace_back(p);
        }

    private:
        roboutils::PeriodicCallback m_state_update;
        std::vector<Callback> m_state_callbacks;

        void update_state()
        {
            BrickPiUpdateValues();
            std::function<void()> func;
            NotifyType notify;
            size_t num_cbs = m_state_callbacks.size();
            size_t counter = 0;
            while(!m_state_callbacks.empty() && counter < num_cbs)
            {
                ++counter;
                std::tie(func, notify) = m_state_callbacks.back();
                if(notify == NotifyType::ONCE)
                {
                    m_state_callbacks.pop_back();
                }
                func();
            }
        }
};


class UltraSonicSensor
{
    public:
        UltraSonicSensor(int port, double alpha=0.4)
            : m_port(port),
              m_immediate_value(0.0),
              m_moving_average(0.0),
              m_alpha(alpha)
        {
            if(!is_valid_port(port))
            {
                throw std::invalid_argument("Invalid port number value.");
            }
        }

        void update_reading()
        {
            long value = get_brick().Sensor[m_port];
            if(value > -1)
            {
                m_immediate_value = value;
                m_moving_average = (m_alpha * m_immediate_value) + (1.0 - m_alpha) * m_moving_average;
            }
        }

        int get_immediate_value()
        {
            return m_immediate_value;
        }

        double get_average_value()
        {
            return m_moving_average;
        }

    private:
        bool is_valid_port(int port) const
        {
            return port > 0 && port < sizeof(get_brick().Sensor) / sizeof(long);
        }

        int m_port;
        long m_immediate_value;
        double m_moving_average;
        double m_alpha;
};


}

#endif
