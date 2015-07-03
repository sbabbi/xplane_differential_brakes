cmake_minimum_required(VERSION 2.6)
project(differential_brakes)

add_executable(differential_brakes main.cpp)

install(TARGETS differential_brakes RUNTIME DESTINATION bin)
