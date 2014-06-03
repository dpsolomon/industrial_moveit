cmake_minimum_required(VERSION 2.8.3)
project(constrained_ik)

find_package(catkin REQUIRED COMPONENTS kdl_parser roscpp urdf eigen_conversions)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES constrained_ik constrained_ik_plugin
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRECTORIES})

## Declare a cpp library
add_library(constrained_ik
            src/basic_kin.cpp
            src/constrained_ik.cpp
            src/constraint.cpp
            src/constraint_group.cpp
            src/constraints/avoid_joint_limits.cpp
            src/constraints/goal_orientation.cpp
            src/constraints/goal_position.cpp
            src/constraints/tool_position.cpp
            src/constraints/goal_tool_orientation.cpp
            src/constraints/goal_minimize_change.cpp
            src/constraints/goal_mid_joint.cpp
            src/constraints/goal_zero_jvel.cpp
            src/constraints/avoid_singularities.cpp
            src/constraints/joint_vel_limits.cpp
            src/solver_state.cpp
)
target_link_libraries(constrained_ik ${catkin_LIBRARIES})

add_library(constrained_ik_plugin 
            src/constrained_ik_plugin.cpp
)
target_link_libraries(constrained_ik_plugin constrained_ik ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(test_BasicKin test/test_BasicKin.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(test_BasicKin constrained_ik)

catkin_add_gtest(test_BasicIK test/test_BasicIK.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(test_BasicIK constrained_ik)
