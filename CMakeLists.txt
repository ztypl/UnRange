cmake_minimum_required(VERSION 3.6)
project(UnRange)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES main.cpp Graph.h Graph.cpp utility.h utility.cpp DatasetGenerator.cpp DatasetGenerator.h)
add_executable(UnRange ${SOURCE_FILES})