cmake_minimum_required(VERSION 3.12)

project(sdl_event_forwarder CXX)
set(CMAKE_CXX_STANDARD 17)

INCLUDE(cmake/FindSDL2.cmake)

add_executable(sdl_event_forwarder
	main.cpp
)


message("-- SDL2 include dir =  ${SDL2_INCLUDE_DIR} ")

target_include_directories(sdl_event_forwarder PRIVATE
		${SDL2_INCLUDE_DIR}
)

target_link_libraries(sdl_event_forwarder PRIVATE
		${SDL2_LIBRARIES}
)
