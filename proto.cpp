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

#include "motor.h"
#include "periodic_callback.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>

// Compile Using:
// sudo gcc -o program "LEGO - Motor Test.c" -lrt -lm
// Run the compiled program using:
// sudo ./program

namespace basio = boost::asio;

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
  //  printf("BrickPi address %p\n", &brick);

    basio::io_service loop;
    robot::PeriodicCallback pc(posix_time::seconds(5), loop);

    robot::Motor motor{PORT_A};

    result = BrickPiSetupSensors();
    motor.set_speed(200);

    pc.start(
        [&motor]()
        {
            std::cout << "Changing speed from " << motor.get_speed() << " to " << -motor.get_speed() << std::endl;
            BrickPiUpdateValues();
            motor.set_speed(-motor.get_speed());
        }
    );

    loop.run();

//    printf("BrickPiSetupSensors: %d\n", result);
//    v=0;
//    f=1;
//    if(!result){
//      usleep(10000);
//      while(1){
////        result = BrickPiUpdateValues();
//        if(!result){
//          printf("%d\n",v);
//          if(f==1) {
//              if(++v > 300) f=0;
//              if(motor.get_speed() != 200)
//              {
//                  std::cout << "Changing to 200\n";
//                  motor.set_speed(200);
////              BrickPiUpdateValues();
//              }
//          }
//          else{
//              if(--v<0) f=1;
//              if(motor.get_speed() != -200)
//              {
//                  std::cout << "Changing to -200\n";
//                  motor.set_speed(-200);
////              BrickPiUpdateValues();
//              }
//          }
//         }
//        usleep(10000);
//      }
//    }
    return 0;
}
