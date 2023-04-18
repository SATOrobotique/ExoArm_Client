cmake_minimum_required(VERSION 3.0.2)
project(exoarm_api)

add_subdirectory(lib/ExoArm_API/proto)

include_directories(include)

add_executable(exoarm_client src/main.cpp src/exoarm_client.cpp)

target_link_libraries(exoarm_client proto)