project(HybridEstimator)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g") #the -g option is implicit...
SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}") #Default build mode is release mode
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( ${PROJECT_NAME})

