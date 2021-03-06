cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
# project(pwcnet_ros)
project(pwcnet_ros CUDA CXX)

add_compile_options(-std=c++11)

# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  message_runtime
  message_generation
  actionlib
  actionlib_msgs
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

# Generate actions in the 'action' folder
# add_action_files(
#   FILES
#     opticalflow.action
# )

generate_dynamic_reconfigure_options(
  cfg/pwcnet.cfg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs dynamic_reconfigure
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# Path for flownet2 with PWC-Net's patch
set(CAFFE_PATH /root/flownet2)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CAFFE_PATH}/include
  ${CAFFE_PATH}/build/src
  /root/catkin_ws/devel/include
)

add_library(${PROJECT_NAME} src/pwc_net_ros.cu)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CAFFE_PATH}/build/lib/libcaffe.so
)

add_executable(sample_node
  src/sample_node.cpp
  src/sample_node_main.cpp
)
# add_dependencies(sample_node ${PROJECT_NAME})
target_link_libraries(sample_node ${PROJECT_NAME})
add_dependencies(sample_node ${PROJECT_NAME}_gencfg)

# Action server
add_executable(pwcnet_action_server
  src/action_server.cpp
  src/action_server_node.cpp
)
# add_dependencies(pwcnet_action_server ${PROJECT_NAME})
target_link_libraries(pwcnet_action_server ${PROJECT_NAME})
add_dependencies(pwcnet_action_server ${PROJECT_NAME}_gencfg)

# Action client
add_executable(pwcnet_action_client
  src/action_client.cpp
  src/action_client_node.cpp
)
# add_dependencies(pwcnet_action_client ${PROJECT_NAME})
target_link_libraries(pwcnet_action_client ${PROJECT_NAME})
add_dependencies(pwcnet_action_client ${PROJECT_NAME}_gencfg)


