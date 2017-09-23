cmake_minimum_required (VERSION 2.8)
project (TankoRoboto CXX)

add_definitions(-std=c++14)

set(RASP_TOOLS /home/aleks/space/raspberrypi_toolchain)
set(Boost_INCLUDE_DIR ${RASP_TOOLS}/raspb/lib/boost/include)
set(Boost_LIBRARY_DIR ${RASP_TOOLS}/raspb/lib/boost/lib)
set(azmq_LIBRARY_DIR ${RASP_TOOLS}/raspb/lib)
set(zmq_LIBRARY_DIR ${RASP_TOOLS}/raspb/lib/zmq/lib)
set(zmq_INCLUDE_DIR ${RASP_TOOLS}/raspb/lib/zmq/include)
set(NeatNet_LIBRARY_DIR ${RASP_TOOLS}/raspb/lib/neatnet/lib)
set(NeatNet_INCLUDE_DIR ${RASP_TOOLS}/raspb/lib/neatnet/include)
set(NEATNET_LIB ${NeatNet_LIBRARY_DIR}/libNeatNet_1.0.0.so)

find_library(ZMQ_LIB zmq PATHS ${zmq_LIBRARY_DIR})

find_package(Boost COMPONENTS system filesystem regex program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${azmq_LIBRARY_DIR})
include_directories(${zmq_INCLUDE_DIR})
include_directories(${NeatNet_INCLUDE_DIR})

#version number
set (TankoRoboto 0)
set (TankoRoboto 1)

add_executable(TankoRoboto motor.cpp BrickPi.cpp tank_main.cpp)
target_link_libraries(TankoRoboto ${NEATNET_LIB})
target_link_libraries(TankoRoboto rt)
target_link_libraries(TankoRoboto m)
target_link_libraries(TankoRoboto pthread)
target_link_libraries(TankoRoboto ${Boost_LIBRARIES})
target_link_libraries(TankoRoboto ${ZMQ_LIB})

install(TARGETS TankoRoboto DESTINATION bin)
