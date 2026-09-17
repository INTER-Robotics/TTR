cmake_minimum_required(VERSION 2.8.3)
project(ttr)

add_definitions(-std=c++14 -Wall -Wextra)

add_compile_options(-w)

find_package(catkin REQUIRED COMPONENTS
  # octomap REQUIRED
  # octomap_msgs
  # octomap_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  pcl_conversions
  voxblox
  voxblox_ros
  srv_mav_behaviours
  nodelet
)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(voxblox REQUIRED)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  MakePlan.srv
  PathLengths.srv
)

add_message_files(
  FILES
  node.msg
  branch.msg
  treeGraph.msg
  frontiers.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package( 

CATKIN_DEPENDS message_runtime pcl_conversions voxblox voxblox_ros
DEPENDS PCL
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS})

## Libraries

# Persistent Tree
add_library(persistent_tree 
        src/persistent_tree.cpp
        src/common_utils.cpp)
target_link_libraries(persistent_tree
      ${catkin_LIBRARIES}
      ${OCTOMAP_LIBRARIES}
      ${PCL_LIBRARIES}
      ${Boost_LIBRARIES}
      )
add_dependencies(persistent_tree
      ${catkin_EXPORTED_TARGETS}
      ${OCTOMAP_EXPORTED_TARGETS}
      ${Boost_EXPORTED_TARGETS}
      ttr_generate_messages_cpp)

# Temporal Tree
add_library(local_tree 
      src/local_tree.cpp
      src/common_utils.cpp)
target_link_libraries(local_tree
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )
add_dependencies(local_tree
    ${catkin_EXPORTED_TARGETS}
    ${OCTOMAP_EXPORTED_TARGETS}
    ${Boost_EXPORTED_TARGETS}
    ttr_generate_messages_cpp)

# Validator
add_library(validator 
    src/validator.cpp
    src/common_utils.cpp)
target_link_libraries(validator
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )
add_dependencies(validator
  ${catkin_EXPORTED_TARGETS}
  ${OCTOMAP_EXPORTED_TARGETS}
  ${Boost_EXPORTED_TARGETS}
  ttr_generate_messages_cpp)

# Decision Maker
add_library(decision_maker 
      src/decision_maker.cpp
      src/common_utils.cpp)
target_link_libraries(decision_maker
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )
add_dependencies(decision_maker
    ${catkin_EXPORTED_TARGETS}
    ${Boost_EXPORTED_TARGETS}
    ttr_generate_messages_cpp)

# Planner
add_library(planner 
    src/planner.cpp
    src/common_utils.cpp)
target_link_libraries(planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )
add_dependencies(planner
  ${catkin_EXPORTED_TARGETS}
  ${Boost_EXPORTED_TARGETS}
  ttr_generate_messages_cpp)


## Executables

# Persistent Tree
add_executable(persistent_tree_node
  src/persistent_tree_node.cpp)
target_link_libraries(persistent_tree_node
          persistent_tree
          ${catkin_LIBRARIES}
          ${OCTOMAP_LIBRARIES}
          ${Boost_LIBRARIES}
          )
add_dependencies(persistent_tree_node 
          ${PROJECT_NAME}_gencfg) 

# Temporal Tree
add_executable(local_tree_node
  src/local_tree_node.cpp)
target_link_libraries(local_tree_node
          local_tree
          ${catkin_LIBRARIES}
          ${OCTOMAP_LIBRARIES}
          ${Boost_LIBRARIES}
          )
add_dependencies(local_tree_node 
          ${PROJECT_NAME}_gencfg)

# Validator
add_executable(validator_node
  src/validator_node.cpp)
target_link_libraries(validator_node
          validator
          ${catkin_LIBRARIES}
          ${OCTOMAP_LIBRARIES}
          ${Boost_LIBRARIES}
          )
add_dependencies(validator_node 
          ${PROJECT_NAME}_gencfg) 

# Decision Maker
add_executable(decision_maker_node
  src/decision_maker_node.cpp)
target_link_libraries(decision_maker_node
          decision_maker
          ${catkin_LIBRARIES}
          ${Boost_LIBRARIES}
          )
add_dependencies(decision_maker_node 
          ${PROJECT_NAME}_gencfg)

# Planner
add_executable(planner_node
  src/planner_node.cpp)
target_link_libraries(planner_node
          planner
          ${catkin_LIBRARIES}
          ${Boost_LIBRARIES}
          )
add_dependencies(planner_node 
          ${PROJECT_NAME}_gencfg)
