/*
*  Jaikrishna
*  t.s.jaikrishna<at>gmail.com
*  Initial date: June 20, 2013
*  Updated:  Feb 17, 2015 (John)
*  Based on Matthew Richardson's Example for testing BrickPi
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a program for testing the RPi BrickPi driver with Lego Motor on Port1
*/

#include <iostream>
#include <math.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "tick.h"
#include "BrickPi.h"
#include "controller.hpp"

#include "motor.h"
#include "robo_utils.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>

#include "controller.hpp"
#include <azmq/socket.hpp>

// Compile Using:
// sudo gcc -o program "LEGO - Motor Test.c" -lrt -lm
// Run the compiled program using:
// sudo ./program

int result,v,f;


int main() {
    ClearTick();

    result = BrickPiSetup();

    printf("BrickPiSetup: %d\n", result);
    if(result)
      return 0;

    BrickPiStruct& brick = get_brick();
    brick.Address[0] = 1;
    brick.Address[1] = 2;

    asio::io_service loop;
    roboutils::PeriodicCallback pc(posix_time::seconds(2), loop);
    roboutils::PeriodicCallback update_values(posix_time::milliseconds(10), loop);

    robot::Motor motor{PORT_A};

    result = BrickPiSetupSensors();
    motor.set_speed(200);

    update_values.start(BrickPiUpdateValues);
    pc.start(
        [&motor]()
        {
            std::cout << "Changing speed from " << motor.get_speed() << " to " << -motor.get_speed() << std::endl;
            BrickPiUpdateValues();
            motor.set_speed(-motor.get_speed());
        }
    );


    azmq::pair_socket pub_sock{loop};
    pub_sock.bind("inproc://test");

    roboutils::PeriodicCallback publisher{boost::posix_time::seconds(2), loop};
    std::string send_data{"Test"};
    publisher.start([&]() {
        std::cout << "Sending some stuff..." << std::endl;
        pub_sock.send(asio::buffer(send_data));
        if(send_data == "Test")
        {
            send_data = "s";
        }
        else
        {
            send_data += send_data;
        }
    });

    ZmqController zc{loop, "inproc://test"};

    loop.run();

    return 0;
}
