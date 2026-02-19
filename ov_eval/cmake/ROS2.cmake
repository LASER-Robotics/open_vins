cmake_minimum_required(VERSION 3.5)
project(ov_eval)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ov_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_definitions(-DROS2)

option(ENABLE_ROS "Enable or disable building with ROS" ON)

include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

list(APPEND LIBRARY_SOURCES
    src/dummy.cpp
    src/alignment/AlignTrajectory.cpp
    src/alignment/AlignUtils.cpp
    src/calc/ResultTrajectory.cpp
    src/calc/ResultSimulation.cpp
    src/utils/Loader.cpp
)

add_library(ov_eval_lib SHARED ${LIBRARY_SOURCES})
target_compile_definitions(ov_eval_lib PUBLIC ROS2)
ament_target_dependencies(ov_eval_lib rclcpp ov_core geometry_msgs nav_msgs)
target_link_libraries(ov_eval_lib ${Boost_LIBRARIES})

install(TARGETS ov_eval_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

macro(ov_add_executable name source)
    add_executable(${name} ${source})
    target_compile_definitions(${name} PRIVATE ROS2)
    ament_target_dependencies(${name} rclcpp ov_core geometry_msgs nav_msgs)
    target_link_libraries(${name} ov_eval_lib)
    install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endmacro()

if (ENABLE_ROS)
    ov_add_executable(pose_to_file src/pose_to_file.cpp)
    ov_add_executable(live_align_trajectory src/live_align_trajectory.cpp)
endif()

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

install(PROGRAMS
    python/pid_ros.py
    python/pid_sys.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ov_eval_lib)
ament_package()
