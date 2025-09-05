cmake_minimum_required(VERSION 3.3)

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  ov_core
  ov_init
)

set(THIRDPARTY_LIBRARIES
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Build with ROS1.cmake if you don't have ROS.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

# Include our header files
include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/core/VioManagerHelper.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
)
list(APPEND LIBRARY_SOURCES src/ros/ROS2Visualizer.cpp src/ros/ROSVisualizerHelper.cpp)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
ament_target_dependencies(ov_msckf_lib ${DEPENDENCIES})
target_link_libraries(ov_msckf_lib ${THIRDPARTY_LIBRARIES})
# target_include_directories(ov_msckf_lib PUBLIC src/)
install(TARGETS ov_msckf_lib
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_msckf_lib)

##################################################
# Make binary files!
##################################################

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
ament_target_dependencies(run_subscribe_msckf ${DEPENDENCIES})
target_link_libraries(run_subscribe_msckf ov_msckf_lib ${THIRDPARTY_LIBRARIES})
install(TARGETS run_subscribe_msckf DESTINATION lib/${PROJECT_NAME})

add_executable(run_simulation src/run_simulation.cpp)
ament_target_dependencies(run_simulation ${DEPENDENCIES})
target_link_libraries(run_simulation ov_msckf_lib ${THIRDPARTY_LIBRARIES})
install(TARGETS run_simulation DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_meas src/test_sim_meas.cpp)
ament_target_dependencies(test_sim_meas ${DEPENDENCIES})
target_link_libraries(test_sim_meas ov_msckf_lib ${THIRDPARTY_LIBRARIES})
install(TARGETS test_sim_meas DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
ament_target_dependencies(test_sim_repeat ${DEPENDENCIES})
target_link_libraries(test_sim_repeat ov_msckf_lib ${THIRDPARTY_LIBRARIES})
install(TARGETS test_sim_repeat DESTINATION lib/${PROJECT_NAME})

##################################################
# Add composable node
##################################################

add_library(run_subscriber_msckf SHARED src/run_subscriber_msckf.cpp)
ament_target_dependencies(run_subscriber_msckf ${DEPENDENCIES})
target_link_libraries(run_subscriber_msckf ov_msckf_lib ${THIRDPARTY_LIBRARIES})

# Register component
rclcpp_components_register_nodes(run_subscriber_msckf PLUGIN "ov_msckf::RunSubscriberMsckf")

install(TARGETS run_subscriber_msckf
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch and config directories
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

# finally define this as the package
ament_package()
