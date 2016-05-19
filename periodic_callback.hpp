#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <future>


using namespace boost;

namespace robot
{

/**
 * Schedules the given callback to be called periodically.
 *
 * The callback is called every duration (posix_time::seconds, or others).
 *
 * If the callback runs for longer than duration, subsequent
 * invocations will be skipped to get back on schedule.
 *
 * Start must be called after the PeriodicCallback is created.
 */
class PeriodicCallback
{
public:
    PeriodicCallback(posix_time::time_duration duration, asio::io_service& io)
    : m_timer(io),
      m_duration(duration)
    {
    }

    template<class F, class... Args>
    void start(F&& f, Args&&... args)
    {
        m_callback = std::bind(f, std::forward<Args>(args)...);
        schedule_callback();
    }

private:

    void schedule_callback()
    {
        m_timer.expires_from_now(m_duration);
        m_timer.async_wait(bind(&PeriodicCallback::handle_timeout, this, asio::placeholders::error));
    }

    void handle_timeout(system::error_code const& error)
    {
        if(error.value() == asio::error::operation_aborted)
        {
            return;
        }

        m_callback();
        schedule_callback();
    }

    asio::deadline_timer m_timer;
    posix_time::time_duration m_duration;
    std::function<void()> m_callback;
};
};
