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
    LINK_DIRECTORIES(
        /usr/local/lib/
        /opt/homebrew/lib/
        )
endif()

set(CAPYBARA_HEADERS
    ${CAPYBARA_DIR}/include/capybara/checkers.hpp
    ${CAPYBARA_DIR}/include/capybara/numpy.hpp
    ${CAPYBARA_DIR}/include/capybara/conversions.hpp
    ${CAPYBARA_DIR}/include/capybara/numerical.hpp
    ${CAPYBARA_DIR}/include/capybara/constraints_manager.hpp
    ${CAPYBARA_DIR}/include/capybara/capytypes.hpp
    ${CAPYBARA_DIR}/include/capybara/utils.hpp
    ${CAPYBARA_DIR}/include/capybara/cronos.hpp
    ${CAPYBARA_DIR}/include/capybara/data_logger.hpp
)

set(CAPYBARA_SOURCES
    ${CAPYBARA_DIR}/src/checkers.cpp
    ${CAPYBARA_DIR}/src/numpy.cpp
    ${CAPYBARA_DIR}/src/conversions.cpp
    ${CAPYBARA_DIR}/src/numerical.cpp
    ${CAPYBARA_DIR}/src/constraints_manager.cpp
    ${CAPYBARA_DIR}/src/utils.cpp
    ${CAPYBARA_DIR}/src/cronos.cpp
    ${CAPYBARA_DIR}/src/data_logger.cpp
)


if(NOT USE_CAPY_DQROBOTICS)
    message(AUTHOR_WARNING "Environment variable USE_CAPY_DQROBOTICS is not set.
        Functionalities of Capybara and DQ robotics not enabled.")
    set(CAPY_DQROBOTICS_SOURCES)
    set(CAPY_DQROBOTICS_HEADERS)
else()
    message(NOTICE "Environment variable USE_CAPY_DQROBOTICS is set.
        Functionalities of Capybara and DQ robotics enabled!")
    set(CAPY_DQROBOTICS_HEADERS
        ${CAPYBARA_DIR}/include/capybara/dqrobotics/rigid_motions.hpp
        ${CAPYBARA_DIR}/include/capybara/dqrobotics/custom_controllers.hpp
    )
    set(CAPY_DQROBOTICS_SOURCES
        ${CAPYBARA_DIR}/src/rigid_motions.cpp
        ${CAPYBARA_DIR}/src/custom_controllers.cpp
    )
endif()


if (NOT USE_EXPERIMENTAL_FEATURES)
    message(AUTHOR_WARNING "Environment variable USE_EXPERIMENTAL_FEATURES is not set.
        Experimental functionalities of capybara not enabled!")
    set(CAPYROBOT_CONSTRAINT_MANAGER_SOURCES)
    set(CAPYROBOT_CONSTRAINT_MANAGER_HEADERS)
else()
    set(CAPYROBOT_CONSTRAINT_MANAGER_SOURCES
        ${CAPYBARA_DIR}/include/capybara/experimental/vfi_manager.hpp
        ${CAPYBARA_DIR}/include/capybara/experimental/robot_constraint_manager.hpp
    )
    set(CAPYROBOT_CONSTRAINT_MANAGER_HEADERS
        ${CAPYBARA_DIR}/src/vfi_manager.cpp
        ${CAPYBARA_DIR}/src/robot_constraints_manager.cpp
    )
endif()

add_library(capybara ${CAPYBARA_HEADERS}
                     ${CAPYBARA_SOURCES}
                     ${CAPY_DQROBOTICS_SOURCES}
                     ${CAPY_DQROBOTICS_HEADERS}
                     ${CAPYROBOT_CONSTRAINT_MANAGER_HEADERS}
                     ${CAPYROBOT_CONSTRAINT_MANAGER_SOURCES}
                 )
#if(USE_CAPY_DQROBOTICS)
#    target_link_libraries(capybara dqrobotics)
#endif()
include_directories(${CAPYBARA_DIR}/include/)


