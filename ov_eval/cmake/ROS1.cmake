cmake_minimum_required(VERSION 3.3)
project(ov_eval)

find_package(catkin QUIET COMPONENTS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core)
find_package(Boost REQUIRED COMPONENTS filesystem system)

if (catkin_FOUND AND ENABLE_ROS)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core
            INCLUDE_DIRS src/
            LIBRARIES ov_eval_lib
    )
else ()
    add_definitions(-DROS_AVAILABLE=0)
    message(WARNING "BUILDING WITHOUT ROS!")
    include(GNUInstallDirs)
    set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
    set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/open_vins/")
endif ()

include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

if (NOT catkin_FOUND OR NOT ENABLE_ROS)
    message(STATUS "MANUALLY LINKING TO OV_CORE LIBRARY....")
    file(GLOB_RECURSE OVCORE_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_core/src/*.cpp")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_profile\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_webcam\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_tracking\\.cpp$")
    list(APPEND LIBRARY_SOURCES ${OVCORE_LIBRARY_SOURCES})
    include_directories(${CMAKE_SOURCE_DIR}/../ov_core/src/)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_core/src/
            DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
endif ()

list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/alignment/AlignTrajectory.cpp
        src/alignment/AlignUtils.cpp
        src/calc/ResultTrajectory.cpp
        src/calc/ResultSimulation.cpp
        src/utils/Loader.cpp
)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_eval_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(ov_eval_lib ${thirdparty_libraries})
target_include_directories(ov_eval_lib PUBLIC src/)

install(TARGETS ov_eval_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

macro(ov_add_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} ov_eval_lib ${thirdparty_libraries})
    install(TARGETS ${name}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endmacro()

if (catkin_FOUND AND ENABLE_ROS)
    ov_add_executable(pose_to_file src/pose_to_file.cpp)
    ov_add_executable(live_align_trajectory src/live_align_trajectory.cpp)
    catkin_install_python(PROGRAMS python/pid_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    catkin_install_python(PROGRAMS python/pid_sys.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
endif ()

ov_add_executable(format_converter src/format_converter.cpp)
ov_add_executable(error_comparison src/error_comparison.cpp)
ov_add_executable(error_dataset src/error_dataset.cpp)
ov_add_executable(error_singlerun src/error_singlerun.cpp)
ov_add_executable(error_simulation src/error_simulation.cpp)
ov_add_executable(timing_comparison src/timing_comparison.cpp)
ov_add_executable(timing_flamegraph src/timing_flamegraph.cpp)
ov_add_executable(timing_histogram src/timing_histogram.cpp)
ov_add_executable(timing_percentages src/timing_percentages.cpp)
ov_add_executable(plot_trajectories src/plot_trajectories.cpp)
