cmake_minimum_required(VERSION 4.1)
project(SVLocoIO)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(SVLocoIO)

add_executable(SVLocoIO
        SVLocoIO/SVLocoIO.ino
        SVLocoIO/SVLocoIO.ino
        SVLocoIO/SVLocoIOFunc.cpp)
