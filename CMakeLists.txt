cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(pwc_net CUDA CXX)

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
)
find_package(OpenCV REQUIRED)

# Generate actions in the 'action' folder
add_action_files(
  FILES
    opticalflow.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
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
)

add_library(${PROJECT_NAME} src/pwc_net.cu)
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
add_dependencies(sample_node ${PROJECT_NAME})
target_link_libraries(sample_node ${PROJECT_NAME})

# Action server
add_executable(action_server
  src/action_server.cpp
  src/action_server_node.cpp
)
add_dependencies(action_server ${PROJECT_NAME})
target_link_libraries(action_server ${PROJECT_NAME})

# Action client
add_executable(action_client
  src/action_client.cpp
  src/action_client_node.cpp
)
add_dependencies(action_client ${PROJECT_NAME})
target_link_libraries(action_client ${PROJECT_NAME})


