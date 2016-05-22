cmake_minimum_required (VERSION 2.8)
project (TankoRoboto CXX)

set(Boost_INCLUDE_DIR /raspb/boost/include)
set(Boost_LIBRARY_DIR /raspb/boost/lib)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#version number
set (TankoRoboto 0)
set (TankoRoboto 1)

add_executable(TankoRoboto BrickPi.cpp motor.cpp proto.cpp)
#add_executable(TankoRoboto test.cpp)
target_link_libraries(TankoRoboto rt)
target_link_libraries(TankoRoboto m)
target_link_libraries(TankoRoboto pthread)
target_link_libraries(TankoRoboto ${Boost_LIBRARIES})

add_definitions(-std=c++0x)
#set(CMAKE_CXX_FLAGS_DEBUG, "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

install(TARGETS TankoRoboto DESTINATION bin)
