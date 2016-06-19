#include <iostream>
#include <math.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <azmq/socket.hpp>

#include "tick.h"
#include "BrickPi.h"
#include "brick_state.hpp"
#include "controller.hpp"

#include "motor.h"
#include "robo_utils.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>


namespace po = boost::program_options;
namespace pt = boost::posix_time;


po::variables_map process_command_args(int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "This program controls the tanko roboto over the given network interface.")
        ("address,a", po::value<std::string>()->required(), "Address of the remote controller")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << desc << '\n';
        exit(EXIT_SUCCESS);
    }

    return vm;
}


int main(int argc, char* argv[])
{
    ClearTick();

    try
    {
        po::variables_map args = process_command_args(argc, argv);
        std::string control_address = args["address"].as<std::string>();
        std::cout << "Connecting to '" << control_address << "' to receive commands.\n";

        int error;
        error = BrickPiSetup();

        if(error)
        {
            std::cerr << "BrickPiSetup failed\n";
            return -1;
        }

        BrickPiStruct& brick = get_brick();
        brick.Address[0] = 1;
        brick.Address[1] = 2;
        brick.SensorType[PORT_2] = TYPE_SENSOR_ULTRASONIC_CONT;

        asio::io_service loop;
        asio::signal_set signals(loop, SIGINT, SIGTERM);
        signals.async_wait(boost::bind(&boost::asio::io_service::stop, &loop));

        error = BrickPiSetupSensors();
        if(error)
        {
            std::cerr << "BrickPiSetupSensors failed.\n";
            return -1;
        }

        Tank tank{PORT_D, PORT_A};

        roboutils::BrickState state{pt::millisec(10), loop};
        roboutils::UltraSonicSensor uss{PORT_2};

        ZmqController zc{loop, control_address, tank};
        SensorController sc{uss, tank, state};

        loop.run();
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return -1;
    }
    catch(...)
    {
        std::cerr << "Unknown exception\n";
        return -1;
    }

    std::cout << "Bye Bye, you control freak.\n";
    return 0;
}
