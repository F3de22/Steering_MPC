cmake_minimum_required(VERSION 3.18)
project(MPC)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

target_include_directories(eigen-3.4.0)

add_subdirectory(osqp)

# Add your source files
add_executable(${PROJECT_NAME}
    main.cpp
    MPC.cpp
    MPC.hpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)
find_package(mpc++ CONFIG REQUIRED)
find_package(NLopt REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE mpc++)
target_link_libraries(${PROJECT_NAME} PRIVATE NLopt::nlopt)