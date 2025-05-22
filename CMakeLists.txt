cmake_minimum_required(VERSION 3.18)
project(MPC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_subdirectory(eigen-3.4.0)
add_subdirectory(osqp-eigen)
add_subdirectory(osqp)

add_executable(${PROJECT_NAME}
    main.cpp
    MPC.cpp
    MPC.hpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp Eigen3::Eigen OsqpEigen)