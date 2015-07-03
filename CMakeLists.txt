cmake_minimum_required(VERSION 2.6)
project(differential_brakes)

set(SDK_XPLANE "" CACHE PATH "Path to X-Plane SDK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(differential_brakes SHARED differential_brakes.cpp)
target_include_directories(differential_brakes SYSTEM PUBLIC ${SDK_XPLANE}/CHeaders ${SDK_XPLANE}/CHeaders/XPLM)

set_target_properties(differential_brakes
    PROPERTIES LINK_FLAGS "-Wl,--version-script=${CMAKE_SOURCE_DIR}/version-script.txt -shared -rdynamic -nodefaultlibs -undefined_warning -fPIC"
    COMPILE_FLAGS "-Wall -O2 -pipe -fPIC -DLIN -DXPLM200 -DXPLM210 -fvisibility=hidden"
    LIBRARY_OUTPUT_NAME "lin.xpl"
    PREFIX "" SUFFIX "")
