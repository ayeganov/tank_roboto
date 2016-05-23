cmake_minimum_required (VERSION 2.8)
project (TankoRoboto CXX)

set(Boost_INCLUDE_DIR /raspb/lib/boost/include)
set(Boost_LIBRARY_DIR /raspb/lib/boost/lib)
set(azmq_LIBRARY_DIR /raspb/lib)
set(zmq_LIBRARY_DIR /raspb/lib/zmq/lib)
set(zmq_INCLUDE_DIR /raspb/lib/zmq/include)

find_library(ZMQ_LIB zmq PATHS ${zmq_LIBRARY_DIR})

find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${azmq_LIBRARY_DIR})
include_directories(${zmq_INCLUDE_DIR})

#version number
set (TankoRoboto 0)
set (TankoRoboto 1)

add_executable(TankoRoboto motor.cpp BrickPi.cpp proto.cpp)
#add_executable(TankoRoboto test.cpp)
target_link_libraries(TankoRoboto rt)
target_link_libraries(TankoRoboto m)
target_link_libraries(TankoRoboto pthread)
target_link_libraries(TankoRoboto ${Boost_LIBRARIES})
target_link_libraries(TankoRoboto ${ZMQ_LIB})

add_definitions(-std=c++0x)
#set(CMAKE_CXX_FLAGS_DEBUG, "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

install(TARGETS TankoRoboto DESTINATION bin)
