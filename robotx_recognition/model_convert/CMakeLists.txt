cmake_minimum_required(VERSION 2.8)
add_definitions("-Wall -std=c++11")
add_executable(plan plan.cpp)
target_link_libraries(plan nvinfer nvparsers)
