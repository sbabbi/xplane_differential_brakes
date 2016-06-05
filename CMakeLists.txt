cmake_minimum_required(VERSION 2.6)
project(differential_brakes)

set(SDK_XPLANE "" CACHE PATH "Path to X-Plane SDK")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(differential_brakes SHARED differential_brakes.cpp)
target_include_directories(differential_brakes SYSTEM PUBLIC ${SDK_XPLANE}/CHeaders ${SDK_XPLANE}/CHeaders/XPLM)

if (UNIX)
	set_target_properties(differential_brakes
		PROPERTIES LINK_FLAGS "-Wl,--version-script=${CMAKE_SOURCE_DIR}/version-script.txt -shared -rdynamic -nodefaultlibs -undefined_warning -fPIC"
		COMPILE_FLAGS "-Wall -O2 -pipe -fPIC -DLIN -DXPLM200 -DXPLM210 -fvisibility=hidden"
		LIBRARY_OUTPUT_NAME "lin.xpl"
		PREFIX "" SUFFIX "")
elseif(WIN32)
	set_target_properties(differential_brakes
		PROPERTIES
		COMPILE_FLAGS "-DIBM -DXPLM200 -DXPLM210 -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS"
		RUNTIME_OUTPUT_NAME "win.xpl"
		PREFIX "" SUFFIX "")

	if( CMAKE_SIZEOF_VOID_P EQUAL 8)
		target_link_libraries(differential_brakes ${SDK_XPLANE}/Libraries/Win/XPLM_64.lib)
	else()
		target_link_libraries(differential_brakes ${SDK_XPLANE}/Libraries/Win/XPLM.lib)
	endif()
else()
	message(ERROR "Unsupported platform")
endif()

add_executable(diff_brake_test differential_brakes.cpp)
target_compile_definitions(diff_brake_test PRIVATE DIFF_BRAKES_TEST)
