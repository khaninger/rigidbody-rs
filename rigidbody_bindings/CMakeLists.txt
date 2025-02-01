cmake_minimum_required(VERSION 3.10)
project(PinocchioBenchmark)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(pinocchio REQUIRED)

add_executable(main main.cpp)

target_link_libraries(main PRIVATE pinocchio::pinocchio rigidbody_bindings)

install(TARGETS main
        RUNTIME DESTINATION bin)
