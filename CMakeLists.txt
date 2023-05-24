cmake_minimum_required(VERSION 2.8)

project(serial_test)
set(APP_NAME ${PROJECT_NAME})

find_package (Threads)

# Enable C++11 standard
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb -std=c++11 -Wall -Wextra")

add_executable(${PROJECT_NAME} main.cpp SerialModule.cpp SerialComm.cpp CommFactory.cpp)
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
