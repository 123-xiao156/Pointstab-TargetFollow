cmake_minimum_required(VERSION 2.8.3...3.5)
project(iimt_local_planner)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lm")
SET(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lm")
find_package(catkin REQUIRED
    COMPONENTS
    base_local_planner
    costmap_2d
    costmap_converter
    dynamic_reconfigure
    nav_core
    nav_msgs
    pluginlib
    pcl_conversions
    roscpp
    tf2
    robot_msgs
    sensor_msgs
    apriltag_ros
    angles
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(casadi REQUIRED)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
if(UNIX AND NOT APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/include/coin)
elseif(APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
endif()
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
    message("PFL NOT YAMLCPP_FOUND")
    find_package(yaml-cpp 0.6 REQUIRED)
    set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
    set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
    message("PFL YAMLCPP_FOUND: ", ${YAMLCPP_VERSION}, ${YAMLCPP_LIBRARY_DIRS})
    if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
        add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
    endif()
    link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()
# # set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)
set(EXTERNAL_INCLUDE_DIRS ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

# find casadi library
find_library(CASADI_LIBRARY
        NAMES casadi
        HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EXTERNAL_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    SYSTEM ${IPOPT_INCLUDE_DIRS}
    SYSTEM ${CASADI_INCLUDE_DIR}
    /usr/include/jsoncpp
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/IIMTPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
    LIBRARIES iimt_local_planner ${EXTERNAL_LIBS}
    CATKIN_DEPENDS
    dynamic_reconfigure
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    DEPENDS SUITESPARSE G2O
)

add_library(iimt_local_planner
    src/iimt_planner_ros.cpp
    src/common/common_algorithm.cpp
    src/common/visualizer_util.cpp
    src/common/path_manager.cpp
    src/common/robot_param_setting.cpp
    src/common/state_manager.cpp
    src/common/common_params.cpp

    # behavior
    src/behavior/behavior_base.cpp
    src/behavior/behavior_rotate.cpp
    src/behavior/behavior_recovery.cpp
    src/behavior/behavior_fix_follow.cpp
    src/behavior/behavior_free_nav.cpp
    src/behavior/behavior_relocalization.cpp
    src/behavior/behavior_charging.cpp
    src/behavior/behavior_point_stab.cpp
    src/behavior/behavior_target_follow.cpp #20260804

    # collision detector
    src/collision_detector/costmap_collision_detector.cpp

    # controller
    src/controller/pp_controller.cpp
    src/controller/dwa_controller.cpp
    src/controller/mpc_controller.cpp
    src/controller/teb_local_planner/obstacles.cpp
    src/controller/teb_local_planner/optimal_planner.cpp
    src/controller/teb_local_planner/recovery_behaviors.cpp
    src/controller/teb_local_planner/teb_config.cpp
    src/controller/teb_local_planner/teb_local_planner_controller.cpp
    src/controller/teb_local_planner/timed_elastic_band.cpp
    src/controller/teb_local_planner/visualization.cpp
    
    src/controller/lattice_planner/lattice_planner.cpp
    src/controller/graceful_controller/graceful_controller.cpp
    
    # env processor
    src/env_processor/env_oscillation_proc.cpp
    src/env_processor/env_narrow_proc.cpp
    src/controller/charge_controller/charge_controller.cpp
    src/controller/charge_controller/tag_odom_location.cpp
)
add_dependencies(iimt_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
message(${catkin_LIBRARIES})
target_link_libraries(iimt_local_planner
    ${catkin_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    ${CASADI_LIBRARIES} ipopt
    ${EXTERNAL_LIBS}
    ${PCL_LIBRARIES}
)

install(TARGETS iimt_local_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
