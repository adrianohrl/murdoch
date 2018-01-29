cmake_minimum_required(VERSION 2.8.3)
project(murdoch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  talmech
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include ${talmech_INCLUDE_DIRS}
  LIBRARIES murdoch talmech
  CATKIN_DEPENDS roscpp talmech
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${talmech_INCLUDE_DIRS}
)

set(CPP_FILES
  src/utilities/exception.cpp
  src/utilities/ros_node.cpp
)

set(H_FILES
  include/murdoch/agent_node.h
  include/utilities/exception.h
  include/utilities/ros_node.h
)

add_executable(agent_node
  src/murdoch/agent_node.cpp
  src/agent_node.cpp
  ${CPP_FILES}
)

add_dependencies(agent_node
  ${catkin_EXPORTED_TARGETS}
  ${talmech_EXPORTED_TARGETS}
)

target_link_libraries(agent_node
  ${catkin_LIBRARIES}
  ${talmech_LIBRARIES}
)
