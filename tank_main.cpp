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
#include "sensor.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>


namespace po = boost::program_options;
namespace pt = boost::posix_time;

const std::string SENSOR = "sensor";
const std::string REMOTE = "remote";


po::variables_map process_command_args(int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "This program controls the tanko roboto over the given network interface.")
        ("address,a", po::value<std::string>(), "Address of the remote controller")
        ("network,n", po::value<std::string>(), "Neural network file to use for controlling the bot")
        ("controller,c", po::value<std::string>()->required(), "Type of controller to use: sensor, remote")
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


void handle_serial(std::string data)
{
    std::cout << data << '\n';
}


int main(int argc, char* argv[])
{
    ClearTick();

    try
    {
        po::variables_map args = process_command_args(argc, argv);

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

        std::shared_ptr<IController> controller;
        std::shared_ptr<roboutils::UltraSonicSensor> uss;
        std::string control_type = args["controller"].as<std::string>();
        if(control_type == REMOTE)
        {
            auto entry = args.find("address");
            if(entry == args.end())
            {
                throw std::invalid_argument("You must specify address when using remote controller.");
            }
            std::string control_address = args["address"].as<std::string>();
            std::cout << "Connecting to '" << control_address << "' to receive commands.\n";
            controller = std::make_shared<ZmqController>(loop, control_address, tank);
        }
        else if(control_type == SENSOR)
        {
            auto entry = args.find("network");
            if(entry == args.end())
            {
                throw std::invalid_argument("You must specify network when using sensor controller.");
            }
            std::string net_path = args["network"].as<std::string>();
            controller = std::make_shared<NeuralController>(net_path, tank, loop, state);
        }
        else
        {
            std::string errmsg = "Unknown controller type " + control_type;
            throw std::invalid_argument(errmsg);
        }

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
