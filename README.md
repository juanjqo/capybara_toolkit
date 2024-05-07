![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/capybara_toolkit)![Static Badge](https://img.shields.io/badge/status-experimental-red)

# Capybara Toolkit

<img src=https://github.com/juanjqo/capybara_toolkit/assets/23158313/2e0dbd2d-9b12-4930-9ffe-511d8270de03 width='250'>

A library for C++17 with an incomplete collection of useful classes and functions based on Eigen and more.


#### Include capybara_toolkit via CMake 

```Cmake
include(FetchContent)
FetchContent_Declare(capybara
    GIT_REPOSITORY https://github.com/juanjqo/capybara_toolkit
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/capybara
    GIT_TAG PUT_HERE_THE_COMMIT_YOU_WANT # Ex:  GIT_TAG 31df12a692c64b29ffcb8311ceaab1853d8f0338
)
FetchContent_MakeAvailable(capybara)
set (CAPYBARA_DIR ${capybara_SOURCE_DIR})
include(${capybara_SOURCE_DIR}/capybara_library.cmake)

add_executable(${CMAKE_PROJECT_NAME} main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}
                        capybara)
```


