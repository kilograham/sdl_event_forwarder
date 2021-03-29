cmake_minimum_required(VERSION 3.12)

project(sdl_event_forwarder CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(SDL2 REQUIRED)

add_executable(sdl_event_forwarder
	main.cpp
)

set(SDL2_INCLUDE_DIR /usr/local/include)

target_include_directories(sdl_event_forwarder PRIVATE
		${SDL2_INCLUDE_DIR}
)

target_link_libraries(sdl_event_forwarder PRIVATE
		${SDL2_LIBRARIES}
)
