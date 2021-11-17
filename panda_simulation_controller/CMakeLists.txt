cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(jsoncpp REQUIRED)

include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS

  roscpp
  std_msgs
  genmsg
  message_generation
franka_core_msgs
gazebo_msgs
franka_moveit
panda_sim_moveit
panda_gazebo

)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
endif()
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  src/examples_common.cpp
)
add_library(Controller STATIC
  src/Controller.cpp
)
target_link_libraries(examples_common 
PUBLIC Franka::Franka 
PUBLIC ${Eigen_INCLUDE_DIRS} 
PUBLIC ${ROS_LIBRARIES})
target_include_directories(examples_common PUBLIC  include )

target_link_libraries(Controller 
PUBLIC Franka::Franka 
PUBLIC ${Eigen_INCLUDE_DIRS} 
PUBLIC ${ROS_LIBRARIES})
target_include_directories(Controller PUBLIC  include )


set(EXAMPLES
	main
)

foreach(example ${EXAMPLES})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin")
  add_executable(${example} "src/${example}.cpp" )
  target_link_libraries(${example} Franka::Franka examples_common Controller ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES}  ${ROS_LIBRARIES} jsoncpp)
find_package(catkin REQUIRED COMPONENTS
franka_core_msgs
  roscpp
  std_msgs
  genmsg
  message_generation
  franka_core_msgs
gazebo_msgs
franka_moveit
panda_sim_moveit
panda_gazebo
  
)
target_include_directories(${example} PUBLIC  include )
target_include_directories(${example} PUBLIC  ${catkin_INCLUDE_DIRS} )


endforeach()

#target_link_libraries(joint_impedance_control Threads::Threads)
#target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
