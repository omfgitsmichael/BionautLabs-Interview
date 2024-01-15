cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

project(astranisInterview VERSION 1.0.0)

set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Add a list of subdirectories #
add_subdirectory(thirdParty)
add_subdirectory(src)

add_executable(sim simulation/main.cpp)
target_link_libraries(sim PRIVATE astranisInterview-libs)
