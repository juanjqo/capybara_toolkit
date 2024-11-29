cmake_minimum_required(VERSION 3.5)

project(examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(USE_CAPY_DQROBOTICS true)
set (CAPYBARA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../capybara_toolkit/)
include(${CAPYBARA_DIR}/capybara_library.cmake)

add_executable(examples main.cpp)
target_link_libraries(examples capybara)

add_executable(example2 example2.cpp)
target_link_libraries(example2 capybara dqrobotics)
