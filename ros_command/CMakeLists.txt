cmake_minimum_required(VERSION 3.5)
project(ros_command)

set(CMAKE_CXX_STANDARD 17)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  eigen_conversions
  tf_conversions
  roslib
  sensor_msgs
  ifopt
  tf
  OpenCV
  image_transport
  cv_bridge
)

find_package(iDynTree REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

find_package( ifopt REQUIRED )

find_package(gazebo REQUIRED)

find_package(Threads REQUIRED)

#find_package(OpenCV REQUIRED)

#find_package(PkgConfig REQUIRED)
#pkg_search_module(ZBAR REQUIRED zbar)

include_directories(/usr/include/eigen3)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${ZBAR_INCLUDE_DIRS})
link_directories(${ZBAR_LIBRARY_DIRS})







## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dogbot_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




SET( ALGLIB_SRC 
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
)

SET ( TOWR_SRC 
  src/towr/nlp_formulation.cc
  src/towr/parameters.cc
  src/towr/nodes_variables.cc
  src/towr/nodes_variables_all.cc
  src/towr/nodes_variables_phase_based.cc
  src/towr/phase_durations.cc
  src/towr/robot_model.cc
  src/towr/dynamic_model.cc
  src/towr/single_rigid_body_dynamics.cc
  src/towr/time_discretization_constraint.cc
  src/towr/base_motion_constraint.cc
  src/towr/terrain_constraint.cc
  src/towr/swing_constraint.cc
  src/towr/force_constraint.cc
  src/towr/total_duration_constraint.cc
  src/towr/dynamic_constraint.cc
  src/towr/range_of_motion_constraint.cc
  src/towr/spline_acc_constraint.cc
  src/towr/linear_constraint.cc
  src/towr/node_cost.cc
  src/towr/soft_constraint.cc
  src/towr/gait_generator.cc
  src/towr/monoped_gait_generator.cc
  src/towr/biped_gait_generator.cc
  src/towr/quadruped_gait_generator.cc
  src/towr/height_map_examples.cc
  src/towr/height_map.cc
  src/towr/state.cc
  src/towr/polynomial.cc
  src/towr/spline.cc
  src/towr/phase_spline.cc
  src/towr/node_spline.cc
  src/towr/nodes_observer.cc
  src/towr/spline_holder.cc
  src/towr/euler_converter.cc
  src/towr/phase_durations_observer.cc
)




## Declare a cpp library
# add_library(dogbot_description
#   src/${PROJECT_NAME}/dogbot_description.cpp
# )

add_library( optimal src/optimal.cpp ${ALGLIB_SRC} )
target_link_libraries( optimal ${catkin_LIBRARIES}  )

add_library( planning src/planning.cpp ${TOWR_SRC} ) 
target_link_libraries( planning ${catkin_LIBRARIES})


add_library( scanner src/scanner.cpp ) 
target_link_libraries( scanner ${catkin_LIBRARIES})





## Declare a cpp executable
# add_executable(dogbot_description_node src/dogbot_description_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dogbot_description_node dogbot_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(dogbot_description_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dogbot_description dogbot_description_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dogbot_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#SET(folder_source forward_dynamics.cpp)

#source_group("Source Files" FILES ${folder_source})

#add_executable(${PROJECT_NAME} ${folder_source})
add_executable(ros_publisher src/for_dyn.cpp )  #insert scanner.cpp

message(STATUS "iDynTree_LIBRARIES : ${iDynTree_LIBRARIES}")

#target_include_directories(ros_publisher PRIVATE ${OpenCV_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS})
#target_link_directories(ros_publisher PRIVATE ${ZBAR_LIBRARY_DIRS})

target_link_libraries(ros_publisher optimal planning scanner  ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} Eigen3::Eigen ${GAZEBO_LIBRARIES}  ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZBAR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})



target_compile_features(ros_publisher PRIVATE cxx_std_17)

