cmake_minimum_required(VERSION 2.8.3)
project(murdoch)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")
endif()

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
  src/utilities/noisy_duration.cpp
  src/utilities/noisy_time.cpp
  src/utilities/probability_density_function.cpp
  src/utilities/task_generator.cpp
)

set(H_FILES
  include/murdoch/agent_node.h
  include/utilities/exception.h
  include/utilities/interval.h
  include/utilities/noisy.h
  include/utilities/noisy_bool.h
  include/utilities/noisy_double.h
  include/utilities/noisy_duration.h
  include/utilities/noisy_long.h
  include/utilities/noisy_time.h
  include/utilities/probability_density_function.h
  include/utilities/random_generator.h
  include/utilities/task_generator.h
)

add_executable(task_executor_node
  src/murdoch/task_executor_node.cpp
  src/task_executor_node.cpp
  ${CPP_FILES}
)

add_dependencies(task_executor_node
  ${catkin_EXPORTED_TARGETS}
  ${talmech_EXPORTED_TARGETS}
)

target_link_libraries(task_executor_node
  ${catkin_LIBRARIES}
  ${talmech_LIBRARIES}
)

add_executable(task_generator_node
  src/murdoch/task_generator_node.cpp
  src/task_generator_node.cpp
  ${CPP_FILES}
)

add_dependencies(task_generator_node
  ${catkin_EXPORTED_TARGETS}
  ${talmech_EXPORTED_TARGETS}
)

target_link_libraries(task_generator_node
  ${catkin_LIBRARIES}
  ${talmech_LIBRARIES}
)
