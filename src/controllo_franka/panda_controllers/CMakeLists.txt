cmake_minimum_required(VERSION 2.8.3)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	controller_interface
	dynamic_reconfigure
	franka_hw
	geometry_msgs
	franka_msgs
	hardware_interface
	message_generation
	pluginlib
	realtime_tools
	roscpp
	rospy
  	std_msgs
  	std_srvs  
  	tf
  	tf_conversions
  	eigen_conversions
  	sensor_msgs
  	cv_bridge
)

#########################################################
############ sensore #####################################


##################################################################

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

#add_message_files(FILES
# )

# generate_messages(
# 	DEPENDENCIES##
# 	franka_msgs##
# 	std_msgs
# 	geometry_msgs
# )




include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	controller_interface
	dynamic_reconfigure
	franka_hw
	geometry_msgs
	franka_msgs
	hardware_interface
	message_runtime
	pluginlib
	realtime_tools
	roscpp
	DEPENDS Franka
)

add_library(${PROJECT_NAME}   
	src/computed_torque.cpp #Computed Torque controller
	src/cartesian_impedance_controller_softbots.cpp #CartesianImpendanceSoftBots controller
	src/cartesian_impedance_controller_softbots2.cpp
	src/hybrid.cpp # Hybrid controller
)

# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   panda_controllers ${catkin_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	panda_controllers ${catkin_EXPORTED_TARGETS}
)

add_executable(position src/position.cpp)
add_executable(human_like_planning src/human_like_planning.cpp)
add_executable(human_like_planning2 src/human_like_planning2.cpp)
add_executable(human_like_planning3 src/human_like_planning3.cpp)

target_link_libraries(position ${catkin_LIBRARIES})
target_link_libraries(human_like_planning ${catkin_LIBRARIES} )
target_link_libraries(human_like_planning2 ${catkin_LIBRARIES} )
target_link_libraries(human_like_planning3 ${catkin_LIBRARIES} )

target_link_libraries(${PROJECT_NAME} PUBLIC
	${Franka_LIBRARIES}
	${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	${Franka_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
	include
)

## Installation
install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
