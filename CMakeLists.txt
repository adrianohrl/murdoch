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
)

set(H_FILES
)

add_executable(auctioneer_node
	src/murdoch/auctioneer_node.cpp
	src/auctioneer_node.cpp 
	${CPP_FILES}
)

add_dependencies(auctioneer_node
	${catkin_EXPORTED_TARGETS}
	${talmech_EXPORTED_TARGETS} 
)

target_link_libraries(auctioneer_node
   ${catkin_LIBRARIES}
   ${talmech_LIBRARIES}
)

add_executable(bidder_node 
	src/murdoch/bidder_node.cpp
	src/bidder_node.cpp 
	${CPP_FILES}
)

add_dependencies(bidder_node
	${catkin_EXPORTED_TARGETS}
	${talmech_EXPORTED_TARGETS} 
)

target_link_libraries(bidder_node
   ${catkin_LIBRARIES}
   ${talmech_LIBRARIES}
)