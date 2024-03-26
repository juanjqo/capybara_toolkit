if (WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

set(CAPYBARA_HEADERS
    ${CAPYBARA_DIR}/include/capybara/checkers.h
    ${CAPYBARA_DIR}/include/capybara/capynum.h)

set(CAPYBARA_SOURCES
    ${CAPYBARA_DIR}/src/checkers.cpp
    ${CAPYBARA_DIR}/src/capynum.cpp)

add_library(capybara ${CAPYBARA_HEADERS} ${CAPYBARA_SOURCES})
include_directories(${CAPYBARA_DIR}/include/)

