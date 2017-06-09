set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_bci)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation cnbiros_core)
find_package(PkgConfig)
pkg_search_module(CNBILOOP REQUIRED cnbiloop)
pkg_search_module(TOBICORE REQUIRED tobicore)
pkg_search_module(TOBIID REQUIRED tobiid)
pkg_search_module(TOBIIC REQUIRED tobiic)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
					${CNBILOOP_INCLUDE_DIRS} 
					${TOBICORE_INCLUDE_DIRS} 
					${TOBIID_INCLUDE_DIRS} 
					${TOBIIC_INCLUDE_DIRS} 
					include)

add_message_files(
	FILES
	TidMessage.msg
	TicClass.msg
	TicClassifier.msg
	TicMessage.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
	${CNBILOOP_INCLUDE_DIRS}
	${TOBICORE_INCLUDE_DIRS}
	${TOBIID_INCLUDE_DIRS}
	${TOBIIC_INCLUDE_DIRS}
  LIBRARIES 
    ${CNBILOOP_LIBRARIES}
    ${TOBICORE_LIBRARIES}
    ${TOBIID_LIBRARIES}
    ${TOBIIC_LIBRARIES}
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	message_runtime
  DEPENDS
	CNBILOOP
	TOBICORE
	TOBIID
	TOBIIC
)

add_library(cnbiros_bci
			src/TobiInterface.cpp
			src/TidClientSet.cpp
			src/TidInterface.cpp
			src/TidTools.cpp
			src/TicClientSet.cpp
			src/TicInterface.cpp
			src/TicTools.cpp
			)
		target_link_libraries(cnbiros_bci ${catkin_LIBRARIES} 
										  ${CNBILOOP_LIBRARIES} 
										  ${TOBICORE_LIBRARIES}
										  ${TOBIID_LIBRARIES}
										  ${TOBIIC_LIBRARIES})

add_executable(rostid src/rostid.cpp)
target_link_libraries(rostid cnbiros_bci)

add_executable(rostic src/rostic.cpp)
target_link_libraries(rostic cnbiros_bci)

add_executable(ros_ticsender src/ros_ticsender.cpp)
target_link_libraries(ros_ticsender cnbiros_bci)

add_executable(example_cnbiloop examples/example_cnbiloop.cpp)
target_link_libraries(example_cnbiloop cnbiros_bci)

add_executable(example_tidclset examples/example_tidclset.cpp)
target_link_libraries(example_tidclset cnbiros_bci)

add_executable(example_tid examples/example_tid.cpp)
target_link_libraries(example_tid cnbiros_bci)

add_executable(example_tic examples/example_tic.cpp)
target_link_libraries(example_tic cnbiros_bci)

