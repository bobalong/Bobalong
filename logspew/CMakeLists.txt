cmake_minimum_required(VERSION 2.6)

project(LOGSPEW)

set(SOURCES logspew.cpp udpclient.cpp)
add_executable(logspew ${SOURCES})
target_link_libraries(logspew rt)


