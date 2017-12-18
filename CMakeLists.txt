cmake_minimum_required(VERSION 3.6)
project(UnRange)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCE_FILES main.cpp GPTree.h GPTree.cpp utility.h utility.cpp DatasetGenerator.cpp DatasetGenerator.h HisGraph.cpp HisGraph.h)
add_executable(UnRange ${SOURCE_FILES})