cmake_minimum_required(VERSION 3.10)
project(PinocchioBenchmark)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(pinocchio REQUIRED)

add_executable(kinematics kinematics.cpp)
add_executable(rnea rnea.cpp)

target_link_libraries(kinematics PRIVATE pinocchio::pinocchio)
target_link_libraries(rnea PRIVATE pinocchio::pinocchio)

install(TARGETS kinematics rnea
        RUNTIME DESTINATION bin)
